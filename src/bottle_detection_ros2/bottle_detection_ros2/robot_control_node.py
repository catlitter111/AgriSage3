#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人控制节点
负责控制机器人底盘的移动，包括前进、后退、转向等
通过串口与底层控制器通信
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from bottle_detection_msgs.msg import RobotCommand, RobotStatus
import serial
import threading
import json
import time

# 命令类型常量
CMD_SET_DIRECTION = 0x01
CMD_SET_SPEED = 0x02
CMD_SET_MOTOR = 0x03
CMD_REQUEST_STATUS = 0x04
CMD_SET_POSITION = 0x05

# 方向常量
DIR_FORWARD = 0x00
DIR_BACKWARD = 0x01
DIR_LEFT = 0x02
DIR_RIGHT = 0x03
DIR_STOP = 0x04


class RobotControlNode(Node):
    """机器人控制节点类"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyS3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('status_publish_rate', 2.0)  # Hz
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.status_rate = self.get_parameter('status_publish_rate').value
        
        # 初始化串口
        self.serial = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # 创建订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.robot_cmd_sub = self.create_subscription(
            RobotCommand,
            'robot/command',
            self.robot_command_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            PoseStamped,
            'robot/set_position',
            self.set_position_callback,
            10
        )
        
        # 创建发布者
        self.status_pub = self.create_publisher(
            RobotStatus,
            'robot/status',
            10
        )
        
        # 状态变量
        self.current_speed = 0
        self.current_direction = DIR_STOP
        self.position = {'x': 0.0, 'y': 0.0, 'latitude': 0.0, 'longitude': 0.0}
        self.battery_level = 85.0
        self.cpu_usage = 0.0
        self.harvested_count = 0
        
        # 创建定时器发布状态
        self.create_timer(1.0 / self.status_rate, self.publish_status)
        
        # 紧急停止标志
        self.emergency_stop = False
        
        self.get_logger().info(
            f'机器人控制节点已启动\n'
            f'串口: {self.serial_port}\n'
            f'波特率: {self.baudrate}'
        )
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'成功连接到串口: {self.serial_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'连接串口失败: {e}')
            # 尝试自动检测串口
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    port = ports[0].device
                    self.get_logger().info(f'尝试自动选择串口: {port}')
                    self.serial_port = port
                    self.serial = serial.Serial(
                        port=self.serial_port,
                        baudrate=self.baudrate,
                        timeout=self.timeout
                    )
                    self.get_logger().info(f'成功连接到串口: {self.serial_port}')
                    return True
            except Exception as e2:
                self.get_logger().error(f'自动连接串口失败: {e2}')
            return False
    
    def generate_packet(self, cmd, data=None):
        """生成通信数据包"""
        if data is None:
            data = []
        
        # 计算校验和
        checksum = cmd + len(data)
        for byte in data:
            checksum += byte
        checksum &= 0xFF  # 取低8位
        
        # 构建数据包
        packet = [0xAA, 0x55, cmd, len(data)] + data + [checksum]
        return bytes(packet)
    
    def send_command(self, command):
        """发送命令到机器人"""
        with self.serial_lock:
            if not self.serial or not self.serial.is_open:
                self.get_logger().error('串口未连接，无法发送命令')
                return False
            
            try:
                self.serial.write(command)
                # 记录发送的命令（十六进制格式）
                hex_command = ' '.join([f"{b:02X}" for b in command])
                self.get_logger().debug(f'发送命令: {hex_command}')
                return True
            except Exception as e:
                self.get_logger().error(f'发送命令失败: {e}')
                return False
    
    def move(self, direction, speed):
        """控制机器人移动"""
        # 确保速度在0-100范围内
        speed = max(0, min(100, int(speed * 100)))
        
        # 生成方向和速度命令
        command = self.generate_packet(CMD_SET_DIRECTION, [direction, speed])
        
        # 更新状态
        self.current_direction = direction
        self.current_speed = speed
        
        return self.send_command(command)
    
    def stop(self):
        """停止机器人"""
        command = self.generate_packet(CMD_SET_DIRECTION, [DIR_STOP, 0])
        self.current_direction = DIR_STOP
        self.current_speed = 0
        return self.send_command(command)
    
    def set_position(self, latitude, longitude):
        """设置机器人位置"""
        # 将浮点数经纬度转换为整数（乘以10^6）
        lat_int = int(latitude * 1000000)
        lon_int = int(longitude * 1000000)
        
        # 将32位整数分解为4个字节
        position_data = [
            (lat_int >> 24) & 0xFF,
            (lat_int >> 16) & 0xFF,
            (lat_int >> 8) & 0xFF,
            lat_int & 0xFF,
            (lon_int >> 24) & 0xFF,
            (lon_int >> 16) & 0xFF,
            (lon_int >> 8) & 0xFF,
            lon_int & 0xFF
        ]
        
        command = self.generate_packet(CMD_SET_POSITION, position_data)
        
        # 更新本地位置
        self.position['latitude'] = latitude
        self.position['longitude'] = longitude
        
        return self.send_command(command)
    
    def cmd_vel_callback(self, msg):
        """Twist命令回调"""
        if self.emergency_stop:
            self.get_logger().warn('紧急停止状态，忽略移动命令')
            return
        
        # 解析Twist消息
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 根据速度判断方向
        if abs(linear_x) > abs(angular_z):
            # 直线运动为主
            if linear_x > 0.01:
                direction = DIR_FORWARD
                speed = abs(linear_x) / self.max_linear_speed
            elif linear_x < -0.01:
                direction = DIR_BACKWARD
                speed = abs(linear_x) / self.max_linear_speed
            else:
                direction = DIR_STOP
                speed = 0
        else:
            # 旋转运动为主
            if angular_z > 0.01:
                direction = DIR_LEFT
                speed = abs(angular_z) / self.max_angular_speed
            elif angular_z < -0.01:
                direction = DIR_RIGHT
                speed = abs(angular_z) / self.max_angular_speed
            else:
                direction = DIR_STOP
                speed = 0
        
        # 发送移动命令
        self.move(direction, speed)
    
    def robot_command_callback(self, msg):
        """机器人命令回调"""
        if msg.emergency_stop:
            self.emergency_stop = True
            self.stop()
            self.get_logger().warn('收到紧急停止命令！')
            # 3秒后解除紧急停止
            self.create_timer(3.0, self.clear_emergency_stop)
            return
        
        if self.emergency_stop:
            self.get_logger().warn('紧急停止状态，忽略命令')
            return
        
        # 处理其他命令
        cmd = msg.command
        speed = msg.speed
        
        if cmd == "forward":
            self.move(DIR_FORWARD, speed)
        elif cmd == "backward":
            self.move(DIR_BACKWARD, speed)
        elif cmd == "left":
            self.move(DIR_LEFT, speed)
        elif cmd == "right":
            self.move(DIR_RIGHT, speed)
        elif cmd == "stop":
            self.stop()
    
    def set_position_callback(self, msg):
        """设置位置回调"""
        # 从PoseStamped消息提取经纬度
        longitude = msg.pose.position.x
        latitude = msg.pose.position.y
        
        self.set_position(latitude, longitude)
        
        self.get_logger().info(f'设置位置: 纬度={latitude}, 经度={longitude}')
    
    def clear_emergency_stop(self):
        """清除紧急停止状态"""
        self.emergency_stop = False
        self.get_logger().info('紧急停止已解除')
    
    def publish_status(self):
        """发布机器人状态"""
        # 模拟一些状态数据（实际应用中应从传感器读取）
        import psutil
        
        status_msg = RobotStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = "base_link"
        
        # 基本状态
        status_msg.battery_level = self.battery_level
        status_msg.cpu_usage = psutil.cpu_percent(interval=0.1)
        status_msg.current_speed = self.current_speed
        status_msg.current_direction = self.current_direction
        
        # 位置信息
        status_msg.position_x = self.position['x']
        status_msg.position_y = self.position['y']
        status_msg.latitude = self.position['latitude']
        status_msg.longitude = self.position['longitude']
        
        # 采摘统计
        status_msg.harvested_count = self.harvested_count
        
        # 其他状态
        status_msg.emergency_stop = self.emergency_stop
        status_msg.is_moving = (self.current_direction != DIR_STOP)
        
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """清理资源"""
        # 停止机器人
        self.stop()
        
        # 关闭串口
        if self.serial and self.serial.is_open:
            self.serial.close()
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()