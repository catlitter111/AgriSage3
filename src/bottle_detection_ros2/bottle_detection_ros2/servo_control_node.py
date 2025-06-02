#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机控制节点
负责控制机械臂舵机，实现目标跟踪和采摘动作
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool, String
from geometry_msgs.msg import Point
from bottle_detection_msgs.msg import HarvestCommand, ServoCommand, ServoStatus
from serial import Serial
import threading
import time
import json

# 舵机控制常量
DEFAULT_SERVO_ID = 0
CENTER_POSITION = 1500
MIN_POSITION = 500
MAX_POSITION = 2500
SERVO_MODE = 3  # 180度顺时针模式

# 机械臂采摘动作指令
ARM_COMMANDS = {
    "rt_start": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T1500!#005P1200T2000!",
    "rt_catch1": "#000P1250T2000!#001P0900T2000!#002P1750T2000!#003P1200T2000!#004P1500T2000!#005P1850T2000!",
    "rt_catch2": "#000P2500T2000!#001P1400T2000!#002P1850T2000!#003P1700T2000!#004P1500T2000!#005P1850T2000!",
    "rt_catch3": "#000P2500T1500!#001P1300T1500!#002P2000T1500!#003P1700T1500!#004P1500T1500!#005P1200T1500!",
    "rt_catch4": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T2000!#005P1200T2000!"
}

# 采摘状态
HARVEST_IDLE = 0
HARVEST_STARTED = 1
HARVEST_STEP1 = 2
HARVEST_STEP2 = 3
HARVEST_STEP3 = 4
HARVEST_STEP4 = 5
HARVEST_COMPLETE = 6


class ServoControlNode(Node):
    """舵机控制节点类"""
    
    def __init__(self):
        super().__init__('servo_control_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyS9')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('tracking_deadzone', 30)  # 像素
        self.declare_parameter('tracking_speed', 7.5)
        self.declare_parameter('enable_tracking', True)
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.tracking_deadzone = self.get_parameter('tracking_deadzone').value
        self.tracking_speed = self.get_parameter('tracking_speed').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        
        # 初始化串口
        self.serial = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # 创建订阅者
        self.servo_cmd_sub = self.create_subscription(
            ServoCommand,
            'servo/command',
            self.servo_command_callback,
            10
        )
        
        self.harvest_cmd_sub = self.create_subscription(
            HarvestCommand,
            'robot/harvest_command',
            self.harvest_command_callback,
            10
        )
        
        self.tracking_target_sub = self.create_subscription(
            Point,
            'servo/tracking_target',
            self.tracking_target_callback,
            10
        )
        
        # 创建发布者
        self.servo_status_pub = self.create_publisher(
            ServoStatus,
            'servo/status',
            10
        )
        
        self.harvest_status_pub = self.create_publisher(
            String,
            'harvest/status',
            10
        )
        
        # 状态变量
        self.current_positions = [CENTER_POSITION] * 6  # 6个舵机
        self.harvest_state = HARVEST_IDLE
        self.harvest_step_time = 0
        self.harvested_count = 0
        
        # 跟踪相关变量
        self.tracking_active = False
        self.stop_flag_x = 1
        self.read_flag_x = 1
        self.send_left = 1
        self.send_right = 1
        self.pid_start_x = 0
        
        self.stop_flag_y = 1
        self.read_flag_y = 1
        self.send_up = 1
        self.send_down = 1
        self.pid_start_y = 0
        
        # 初始化舵机
        self.initialize_servos()
        
        # 创建定时器处理采摘状态机
        self.create_timer(0.1, self.harvest_state_machine)
        
        # 创建状态发布定时器
        self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info(
            f'舵机控制节点已启动\n'
            f'串口: {self.serial_port}\n'
            f'波特率: {self.baudrate}'
        )
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial = Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'成功连接到舵机串口: {self.serial_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'连接舵机串口失败: {e}')
            return False
    
    def send_command(self, command, wait_for_response=False):
        """发送命令到舵机"""
        with self.serial_lock:
            if not self.serial or not self.serial.is_open:
                self.get_logger().error('串口未连接，无法发送命令')
                return None
            
            try:
                self.serial.write(command.encode())
                self.get_logger().debug(f'发送命令: {command}')
                
                if wait_for_response:
                    time.sleep(0.1)
                    if self.serial.in_waiting:
                        response = self.serial.read(self.serial.in_waiting).decode().strip()
                        self.get_logger().debug(f'舵机响应: {response}')
                        return response
                return True
            except Exception as e:
                self.get_logger().error(f'发送命令失败: {e}')
                return None
    
    def move_servo(self, servo_id, position, time_ms=1000):
        """控制舵机移动到指定位置"""
        # 确保参数在有效范围内
        servo_id = max(0, min(254, servo_id))
        position = max(MIN_POSITION, min(MAX_POSITION, position))
        time_ms = max(0, min(9999, time_ms))
        
        # 更新位置记录
        if servo_id < len(self.current_positions):
            self.current_positions[servo_id] = position
        
        # 构造舵机控制命令
        command = f"#{servo_id:03d}P{position:04d}T{time_ms:04d}!"
        return self.send_command(command)
    
    def set_mode(self, servo_id, mode):
        """设置舵机工作模式"""
        mode = max(1, min(8, mode))
        command = f"#{servo_id:03d}PMOD{mode}!"
        response = self.send_command(command, wait_for_response=True)
        return response and response == "#OK!"
    
    def stop_servo(self, servo_id):
        """立即停止舵机"""
        command = f"#{servo_id:03d}PDST!"
        return self.send_command(command)
    
    def read_position(self, servo_id):
        """读取舵机当前位置"""
        command = f"#{servo_id:03d}PRAD!"
        response = self.send_command(command, wait_for_response=True)
        
        if response and response.startswith(f"#{servo_id:03d}P") and response.endswith("!"):
            try:
                position_str = response[5:-1]
                position = int(position_str)
                return position
            except Exception as e:
                self.get_logger().error(f'解析舵机位置错误: {e}')
                return None
        return None
    
    def initialize_servos(self):
        """初始化所有舵机"""
        if not self.serial:
            return
        
        # 设置主舵机为180度顺时针模式
        self.set_mode(DEFAULT_SERVO_ID, SERVO_MODE)
        
        # 将所有舵机移动到初始位置
        self.set_initial_position()
        
        # 将主舵机居中
        self.move_servo(DEFAULT_SERVO_ID, CENTER_POSITION, 2000)
        
        self.get_logger().info('舵机初始化完成')
    
    def set_initial_position(self):
        """设置机械臂到初始位置"""
        command = "#000P1250T1500!#001P0900T1500!#002P2000T1500!#003P0800T1500!#004P1500T1500!#005P1200T1500!"
        return self.send_command(command)
    
    def servo_command_callback(self, msg):
        """舵机命令回调"""
        if msg.servo_id >= 0 and msg.position >= 0:
            self.move_servo(msg.servo_id, msg.position, msg.time_ms)
        
        if msg.stop:
            self.stop_servo(msg.servo_id)
        
        if msg.set_mode and msg.mode >= 0:
            self.set_mode(msg.servo_id, msg.mode)
    
    def harvest_command_callback(self, msg):
        """采摘命令回调"""
        if msg.start_harvest and self.harvest_state == HARVEST_IDLE:
            self.start_harvest()
        
        if msg.stop_harvest:
            self.stop_harvest()
    
    def tracking_target_callback(self, msg):
        """跟踪目标回调"""
        if not self.enable_tracking or not self.tracking_active:
            return
        
        # 执行舵机跟踪
        self.track_object(
            int(msg.z),  # frame_width
            int(msg.z),  # frame_height (假设正方形图像)
            int(msg.x),  # object_cx
            int(msg.y)   # object_cy
        )
    
    def track_object(self, frame_width, frame_height, object_cx, object_cy):
        """跟踪物体 - 与原始代码相同的逻辑"""
        # X轴跟踪
        frame_center_x = frame_width // 2 + 80
        offset_x = frame_center_x - object_cx
        
        if abs(object_cx - frame_center_x) <= self.tracking_deadzone:
            if self.stop_flag_x == 1:
                command = "#{:03d}PDST!".format(0)
                self.send_command(command)
                self.stop_flag_x = 0
                self.read_flag_x = 1
                self.send_left = 1
                self.send_right = 1
        else:
            self.stop_flag_x = 1
            if self.read_flag_x == 1:
                command = "#{:03d}PRAD!".format(0)
                self.send_command(command)
                self.pid_start_x = self.receive_catch()
                self.read_flag_x = 0
            else:
                if frame_center_x - object_cx > self.tracking_deadzone:
                    if self.pid_start_x > 2100:
                        command = "#{:03d}PDST!".format(0)
                    else:
                        temp = int((2167 - self.pid_start_x) * self.tracking_speed)
                        if temp < 4000:
                            temp = 4000
                        command = "#{:03d}P{:04d}T{:04d}!".format(0, 2167, temp)
                    if self.send_left == 1:
                        self.send_command(command)
                        self.send_left = 0
                        self.send_right = 1
                elif object_cx - frame_center_x > self.tracking_deadzone:
                    if self.pid_start_x < 833:
                        command = "#{:03d}PDST!".format(0)
                    else:
                        temp = int((self.pid_start_x - 833) * self.tracking_speed)
                        if temp < 3000:
                            temp = 3000
                        command = "#{:03d}P{:04d}T{:04d}!".format(0, 833, temp)
                    if self.send_right == 1:
                        self.send_command(command)
                        self.send_right = 0
                        self.send_left = 1
        
        # Y轴跟踪
        frame_center_y = frame_height // 2
        offset_y = frame_center_y - object_cy
        
        if abs(object_cy - frame_center_y) <= self.tracking_deadzone:
            if self.stop_flag_y == 1:
                command = "#{:03d}PDST!".format(1)
                self.send_command(command)
                self.stop_flag_y = 0
                self.read_flag_y = 1
                self.send_up = 1
                self.send_down = 1
        else:
            self.stop_flag_y = 1
            if self.read_flag_y == 1:
                command = "#{:03d}PRAD!".format(1)
                self.send_command(command)
                self.pid_start_y = self.receive_catch()
                self.read_flag_y = 0
            else:
                if frame_center_y - object_cy > self.tracking_deadzone:
                    if self.pid_start_y > 1480:
                        command = "#{:03d}PDST!".format(1)
                    else:
                        temp = int((1500 - self.pid_start_y) * self.tracking_speed)
                        if temp < 4000:
                            temp = 4000
                        command = "#{:03d}P{:04d}T{:04d}!".format(1, 1500, temp)
                    if self.send_up == 1:
                        self.send_command(command)
                        self.send_up = 0
                        self.send_down = 1
                elif object_cy - frame_center_y > self.tracking_deadzone:
                    if self.pid_start_y < 882:
                        command = "#{:03d}PDST!".format(1)
                    else:
                        temp = int((self.pid_start_y - 882) * self.tracking_speed)
                        if temp < 3000:
                            temp = 3000
                        command = "#{:03d}P{:04d}T{:04d}!".format(1, 882, temp)
                    if self.send_down == 1:
                        self.send_command(command)
                        self.send_down = 0
                        self.send_up = 1
    
    def receive_catch(self, timeout=0.1):
        """非阻塞方式接收舵机数据"""
        try:
            with self.serial_lock:
                original_timeout = self.serial.timeout
                self.serial.timeout = timeout
                
                data = self.serial.read(256)
                self.serial.timeout = original_timeout
                
                if data:
                    data_str = data.decode('utf-8').strip()
                    if data_str.startswith("#") and data_str.endswith("!"):
                        data_content = data_str.strip("#!").strip().strip("\r\n")
                        parts = data_content.split('P')
                        if len(parts) >= 2:
                            angle = int(parts[1].split('!')[0])
                            return angle
            return None
        except Exception as e:
            self.get_logger().error(f'接收数据失败: {e}')
            return None
    
    def start_harvest(self):
        """开始采摘"""
        if self.harvest_state != HARVEST_IDLE:
            self.get_logger().warn('采摘任务已在进行中')
            return
        
        self.get_logger().info('开始采摘任务')
        self.harvest_state = HARVEST_STARTED
        self.harvest_step_time = time.time()
        
        # 发布采摘开始状态
        status_msg = String()
        status_msg.data = json.dumps({
            "state": "started",
            "timestamp": time.time()
        })
        self.harvest_status_pub.publish(status_msg)
    
    def stop_harvest(self):
        """停止采摘"""
        self.harvest_state = HARVEST_IDLE
        self.set_initial_position()
        self.get_logger().info('采摘任务已停止')
    
    def harvest_state_machine(self):
        """采摘状态机"""
        if self.harvest_state == HARVEST_IDLE:
            return
        
        current_time = time.time()
        
        if self.harvest_state == HARVEST_STARTED:
            # 发送初始指令
            self.send_command(ARM_COMMANDS["rt_start"])
            self.get_logger().info('采摘步骤1: 机械臂就位')
            self.harvest_state = HARVEST_STEP1
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP1 and current_time - self.harvest_step_time > 2.0:
            # 第一步完成
            self.send_command(ARM_COMMANDS["rt_catch1"])
            self.get_logger().info('采摘步骤2: 准备抓取')
            self.harvest_state = HARVEST_STEP2
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP2 and current_time - self.harvest_step_time > 2.0:
            # 第二步完成
            self.send_command(ARM_COMMANDS["rt_catch2"])
            self.get_logger().info('采摘步骤3: 抓取目标')
            self.harvest_state = HARVEST_STEP3
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP3 and current_time - self.harvest_step_time > 2.0:
            # 第三步完成
            self.send_command(ARM_COMMANDS["rt_catch3"])
            self.get_logger().info('采摘步骤4: 抬升目标')
            self.harvest_state = HARVEST_STEP4
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP4 and current_time - self.harvest_step_time > 2.0:
            # 第四步完成
            self.send_command(ARM_COMMANDS["rt_catch4"])
            self.get_logger().info('采摘步骤5: 返回初始位置')
            self.harvest_state = HARVEST_COMPLETE
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_COMPLETE and current_time - self.harvest_step_time > 2.0:
            # 采摘完成
            self.harvested_count += 1
            self.get_logger().info(f'采摘完成，总计: {self.harvested_count}')
            
            # 发布采摘完成状态
            status_msg = String()
            status_msg.data = json.dumps({
                "state": "completed",
                "harvested_count": self.harvested_count,
                "timestamp": time.time()
            })
            self.harvest_status_pub.publish(status_msg)
            
            # 重置状态
            self.harvest_state = HARVEST_IDLE
    
    def publish_status(self):
        """发布舵机状态"""
        status_msg = ServoStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 舵机位置
        status_msg.servo_positions = self.current_positions
        
        # 采摘状态
        status_msg.harvest_state = self.harvest_state
        status_msg.harvested_count = self.harvested_count
        
        # 跟踪状态
        status_msg.tracking_active = self.tracking_active
        
        self.servo_status_pub.publish(status_msg)
    
    def destroy_node(self):
        """清理资源"""
        # 停止所有舵机
        for i in range(6):
            self.stop_servo(i)
        
        # 设置到初始位置
        self.set_initial_position()
        
        # 关闭串口
        if self.serial and self.serial.is_open:
            self.serial.close()
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ServoControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()