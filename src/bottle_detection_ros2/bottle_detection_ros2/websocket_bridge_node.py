#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket桥接节点
负责连接WebSocket服务器，接收控制命令并转发到ROS2系统
同时将视频流和状态信息发送到服务器
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32, Int32, Bool, Header
from bottle_detection_msgs.msg import RobotCommand, HarvestCommand, RobotStatus
import websocket
import json
import threading
import time
import base64
import queue

class WebSocketBridgeNode(Node):
    """WebSocket桥接节点类"""
    
    def __init__(self):
        super().__init__('websocket_bridge_node')
        
        # 声明参数
        self.declare_parameter('server_url', 'ws://101.201.150.96:1234/ws/robot/robot_123')
        self.declare_parameter('reconnect_attempts', 5)
        self.declare_parameter('reconnect_interval', 3.0)
        self.declare_parameter('robot_id', 'robot_123')
        
        # 获取参数
        self.server_url = self.get_parameter('server_url').value
        self.max_reconnect_attempts = self.get_parameter('reconnect_attempts').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.robot_id = self.get_parameter('robot_id').value
        
        # WebSocket相关
        self.ws = None
        self.connected = False
        self.reconnect_count = 0
        
        # 创建订阅者 - 接收来自其他节点的数据
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅压缩图像用于发送
        self.image_sub = self.create_subscription(
            CompressedImage,
            'bottle_detection/compressed_image',
            self.image_callback,
            qos
        )
        
        # 订阅机器人状态
        self.status_sub = self.create_subscription(
            RobotStatus,
            'robot/status',
            self.status_callback,
            10
        )
        
        # 订阅瓶子检测信息
        self.detection_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.detection_callback,
            10
        )
        
        # 创建发布者 - 发布控制命令
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_cmd_pub = self.create_publisher(RobotCommand, 'robot/command', 10)
        self.harvest_cmd_pub = self.create_publisher(HarvestCommand, 'robot/harvest_command', 10)
        self.mode_pub = self.create_publisher(String, 'robot/mode', 10)
        
        # 质量控制
        self.quality_pub = self.create_publisher(String, 'video/quality_preset', 10)
        
        # 位置更新
        self.position_pub = self.create_publisher(PoseStamped, 'robot/set_position', 10)
        
        # 消息队列
        self.image_queue = queue.Queue(maxsize=10)
        self.status_queue = queue.Queue(maxsize=10)
        
        # 启动WebSocket连接
        self.connect_to_server()
        
        # 状态变量
        self.current_mode = "manual"
        self.auto_harvest_active = False
        
        self.get_logger().info(f'WebSocket桥接节点已启动，连接到: {self.server_url}')
    
    def connect_to_server(self):
        """连接到WebSocket服务器"""
        self.ws = websocket.WebSocketApp(
            self.server_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        # 在新线程中运行WebSocket
        ws_thread = threading.Thread(target=self.ws.run_forever)
        ws_thread.daemon = True
        ws_thread.start()
    
    def on_open(self, ws):
        """WebSocket连接打开回调"""
        self.connected = True
        self.reconnect_count = 0
        self.get_logger().info('WebSocket连接已建立')
        
        # 发送初始连接消息
        init_msg = {
            "type": "robot_connect",
            "robot_id": self.robot_id,
            "timestamp": int(time.time() * 1000)
        }
        ws.send(json.dumps(init_msg))
    
    def on_message(self, ws, message):
        """WebSocket消息接收回调"""
        try:
            data = json.loads(message)
            message_type = data.get("type")
            
            if message_type == "command":
                self.handle_command(data)
            elif message_type == "quality_adjustment":
                self.handle_quality_adjustment(data)
            elif message_type == "set_position":
                self.handle_position_update(data)
            elif message_type == "mode_control":
                self.handle_mode_control(data)
            else:
                self.get_logger().warn(f'未知消息类型: {message_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理消息错误: {e}')
    
    def on_error(self, ws, error):
        """WebSocket错误回调"""
        self.get_logger().error(f'WebSocket错误: {error}')
    
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket关闭回调"""
        self.connected = False
        self.get_logger().warn(f'WebSocket连接关闭: {close_status_code} {close_msg}')
        self.schedule_reconnect()
    
    def schedule_reconnect(self):
        """计划重连"""
        if self.reconnect_count < self.max_reconnect_attempts:
            self.reconnect_count += 1
            self.get_logger().info(
                f'计划第 {self.reconnect_count} 次重连，'
                f'{self.reconnect_interval}秒后尝试...'
            )
            
            timer = threading.Timer(self.reconnect_interval, self.connect_to_server)
            timer.daemon = True
            timer.start()
        else:
            self.get_logger().error('达到最大重连次数，停止重连')
    
    def handle_command(self, command_data):
        """处理运动控制命令"""
        cmd = command_data.get("command")
        params = command_data.get("params", {})
        speed = params.get("speed", 50) / 100.0  # 转换为0-1范围
        
        self.get_logger().info(f'收到命令: {cmd}, 速度: {speed*100}%')
        
        # 发布到ROS2命令话题
        robot_cmd = RobotCommand()
        robot_cmd.header.stamp = self.get_clock().now().to_msg()
        robot_cmd.command = cmd
        robot_cmd.speed = speed
        
        # 转换为Twist消息
        twist = Twist()
        
        if cmd == "forward":
            twist.linear.x = speed * 0.5  # 最大速度0.5m/s
        elif cmd == "backward":
            twist.linear.x = -speed * 0.5
        elif cmd == "left":
            twist.angular.z = speed * 1.0  # 最大角速度1.0rad/s
        elif cmd == "right":
            twist.angular.z = -speed * 1.0
        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif cmd == "emergencyStop":
            # 紧急停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            robot_cmd.emergency_stop = True
        elif cmd == "startHarvest":
            # 触发采摘
            harvest_cmd = HarvestCommand()
            harvest_cmd.header.stamp = self.get_clock().now().to_msg()
            harvest_cmd.start_harvest = True
            self.harvest_cmd_pub.publish(harvest_cmd)
            return
        
        # 发布命令
        self.cmd_vel_pub.publish(twist)
        self.robot_cmd_pub.publish(robot_cmd)
    
    def handle_quality_adjustment(self, data):
        """处理视频质量调整"""
        preset = data.get("preset", "medium")
        
        quality_msg = String()
        quality_msg.data = preset
        self.quality_pub.publish(quality_msg)
        
        self.get_logger().info(f'调整视频质量为: {preset}')
        
        # 发送确认
        if self.ws and self.connected:
            response = {
                "type": "quality_adjustment_result",
                "success": True,
                "preset": preset,
                "timestamp": int(time.time() * 1000)
            }
            self.ws.send(json.dumps(response))
    
    def handle_position_update(self, data):
        """处理位置更新"""
        try:
            position_data = data.get("data", [])
            if len(position_data) >= 8:
                # 解析经纬度
                lat_int = (position_data[0] << 24) | (position_data[1] << 16) | \
                         (position_data[2] << 8) | position_data[3]
                lon_int = (position_data[4] << 24) | (position_data[5] << 16) | \
                         (position_data[6] << 8) | position_data[7]
                
                # 处理有符号整数
                if lat_int & 0x80000000:
                    lat_int = lat_int - 0x100000000
                if lon_int & 0x80000000:
                    lon_int = lon_int - 0x100000000
                
                # 转换为浮点数
                latitude = lat_int / 1000000.0
                longitude = lon_int / 1000000.0
                
                # 发布位置更新
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = longitude  # 经度作为x
                pose_msg.pose.position.y = latitude   # 纬度作为y
                pose_msg.pose.position.z = 0.0
                
                self.position_pub.publish(pose_msg)
                
                self.get_logger().info(f'更新位置: 纬度={latitude}, 经度={longitude}')
                
        except Exception as e:
            self.get_logger().error(f'处理位置更新错误: {e}')
    
    def handle_mode_control(self, data):
        """处理模式控制"""
        new_mode = data.get("mode", "manual")
        auto_harvest = data.get("harvest", False)
        
        self.current_mode = new_mode
        self.auto_harvest_active = auto_harvest
        
        # 发布模式更新
        mode_msg = String()
        mode_data = {
            "mode": new_mode,
            "auto_harvest": auto_harvest
        }
        mode_msg.data = json.dumps(mode_data)
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info(f'切换到{new_mode}模式，自动采摘: {auto_harvest}')
    
    def image_callback(self, msg):
        """图像回调 - 接收压缩图像并通过WebSocket发送"""
        if not self.connected or not self.ws:
            return
        
        try:
            # 将图像数据转为base64
            image_base64 = base64.b64encode(msg.data).decode('utf-8')
            
            # 构建消息
            ws_msg = {
                "type": "video_frame",
                "format": msg.format,
                "timestamp": int(time.time() * 1000),
                "data": image_base64
            }
            
            # 发送到WebSocket
            self.ws.send(json.dumps(ws_msg))
            
        except Exception as e:
            self.get_logger().error(f'发送图像失败: {e}')
    
    def status_callback(self, msg):
        """机器人状态回调"""
        if not self.connected or not self.ws:
            return
        
        try:
            # 构建状态消息
            status_data = {
                "type": "status_update",
                "data": {
                    "battery_level": msg.battery_level,
                    "cpu_usage": msg.cpu_usage,
                    "current_speed": msg.current_speed,
                    "current_direction": msg.current_direction,
                    "position": {
                        "x": msg.position_x,
                        "y": msg.position_y,
                        "latitude": msg.latitude,
                        "longitude": msg.longitude
                    },
                    "operation_mode": self.current_mode,
                    "auto_harvest_active": self.auto_harvest_active,
                    "harvested_count": msg.harvested_count,
                    "timestamp": int(time.time() * 1000)
                }
            }
            
            # 发送状态
            self.ws.send(json.dumps(status_data))
            
        except Exception as e:
            self.get_logger().error(f'发送状态失败: {e}')
    
    def detection_callback(self, msg):
        """瓶子检测信息回调"""
        if not self.connected or not self.ws:
            return
        
        try:
            # 直接转发检测信息
            detection_data = json.loads(msg.data)
            detection_data["type"] = "detection_update"
            
            self.ws.send(json.dumps(detection_data))
            
        except Exception as e:
            self.get_logger().error(f'发送检测信息失败: {e}')
    
    def destroy_node(self):
        """清理资源"""
        if self.ws:
            self.ws.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WebSocketBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()