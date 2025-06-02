#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2瓶子检测系统调试可视化程序
实时显示各个模块的订阅信息
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import json
import numpy as np
from datetime import datetime
from collections import deque
import cv2
from PIL import Image, ImageTk

# ROS2消息类型
from sensor_msgs.msg import CompressedImage, Image as RosImage
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import String, Float32, Int32, Bool
from bottle_detection_msgs.msg import (
    BottleDetection, RobotStatus, RobotCommand, 
    HarvestCommand, ServoCommand, ServoStatus
)


class DebugVisualizerNode(Node):
    """调试可视化节点"""
    
    def __init__(self, gui):
        super().__init__('debug_visualizer_node')
        self.gui = gui
        
        # QoS配置
        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 数据存储
        self.data = {
            'fps': 0.0,
            'detection_count': 0,
            'nearest_distance': -1.0,
            'robot_mode': 'unknown',
            'robot_speed': 0.0,
            'battery_level': 0.0,
            'cpu_usage': 0.0,
            'servo_positions': [],
            'harvest_state': 0,
            'last_update': {}
        }
        
        # 历史数据（用于绘图）
        self.history_length = 100
        self.distance_history = deque(maxlen=self.history_length)
        self.fps_history = deque(maxlen=self.history_length)
        self.detection_history = deque(maxlen=self.history_length)
        
        # 创建订阅者
        self._create_subscribers()
        
        # 更新定时器
        self.create_timer(0.1, self.update_gui)
        
        self.get_logger().info('调试可视化节点已启动')
    
    def _create_subscribers(self):
        """创建所有订阅者"""
        # 图像订阅
        self.compressed_image_sub = self.create_subscription(
            CompressedImage,
            'bottle_detection/compressed_image',
            self.compressed_image_callback,
            self.qos_best_effort
        )
        
        # 检测结果订阅
        self.detection_sub = self.create_subscription(
            BottleDetection,
            'bottle_detection/detection',
            self.detection_callback,
            10
        )
        
        self.detection_info_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.detection_info_callback,
            10
        )
        
        self.distance_sub = self.create_subscription(
            Float32,
            'bottle_detection/nearest_distance',
            self.distance_callback,
            10
        )
        
        self.count_sub = self.create_subscription(
            Int32,
            'bottle_detection/count',
            self.count_callback,
            10
        )
        
        # 机器人状态订阅
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            'robot/status',
            self.robot_status_callback,
            10
        )
        
        self.robot_mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.robot_mode_callback,
            10
        )
        
        # 控制命令订阅
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
        
        # 舵机状态订阅
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            'servo/status',
            self.servo_status_callback,
            10
        )
        
        # 采摘状态订阅
        self.harvest_status_sub = self.create_subscription(
            String,
            'harvest/status',
            self.harvest_status_callback,
            10
        )
    
    def compressed_image_callback(self, msg):
        """压缩图像回调"""
        try:
            # 解码JPEG图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                # 转换为RGB
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                # 调整大小以适应GUI
                height, width = cv_image.shape[:2]
                if width > 640:
                    scale = 640 / width
                    new_width = int(width * scale)
                    new_height = int(height * scale)
                    cv_image = cv2.resize(cv_image, (new_width, new_height))
                
                # 更新GUI
                self.gui.update_image(cv_image)
                
            self.data['last_update']['image'] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {e}')
    
    def detection_callback(self, msg):
        """检测结果回调"""
        self.data['bottle_detected'] = msg.bottle_detected
        self.data['bottle_count'] = msg.bottle_count
        
        if msg.bottle_detected:
            self.data['nearest_bottle'] = {
                'x': msg.nearest_bottle_x,
                'y': msg.nearest_bottle_y,
                'confidence': msg.confidence,
                'distance': msg.distance,
                'bbox': [msg.bbox_left, msg.bbox_top, msg.bbox_right, msg.bbox_bottom],
                'position_3d': [msg.position_x, msg.position_y, msg.position_z],
                'status': msg.status
            }
        else:
            self.data['nearest_bottle'] = None
        
        # 更新历史
        self.detection_history.append(msg.bottle_count)
        if msg.distance > 0:
            self.distance_history.append(msg.distance)
        
        self.data['last_update']['detection'] = time.time()
    
    def detection_info_callback(self, msg):
        """检测信息回调"""
        try:
            info = json.loads(msg.data)
            self.data['fps'] = info.get('fps', 0.0)
            self.fps_history.append(self.data['fps'])
            self.data['last_update']['info'] = time.time()
        except:
            pass
    
    def distance_callback(self, msg):
        """距离回调"""
        self.data['nearest_distance'] = msg.data
        self.data['last_update']['distance'] = time.time()
    
    def count_callback(self, msg):
        """计数回调"""
        self.data['detection_count'] = msg.data
        self.data['last_update']['count'] = time.time()
    
    def robot_status_callback(self, msg):
        """机器人状态回调"""
        self.data['battery_level'] = msg.battery_level
        self.data['cpu_usage'] = msg.cpu_usage
        self.data['current_speed'] = msg.current_speed
        self.data['current_direction'] = msg.current_direction
        self.data['position'] = {
            'x': msg.position_x,
            'y': msg.position_y,
            'lat': msg.latitude,
            'lon': msg.longitude
        }
        self.data['harvested_count'] = msg.harvested_count
        self.data['is_moving'] = msg.is_moving
        self.data['emergency_stop'] = msg.emergency_stop
        self.data['last_update']['robot_status'] = time.time()
    
    def robot_mode_callback(self, msg):
        """机器人模式回调"""
        try:
            mode_data = json.loads(msg.data)
            self.data['robot_mode'] = mode_data.get('mode', 'unknown')
            self.data['auto_harvest'] = mode_data.get('auto_harvest', False)
            self.data['last_update']['mode'] = time.time()
        except:
            pass
    
    def cmd_vel_callback(self, msg):
        """速度命令回调"""
        self.data['cmd_vel'] = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z
        }
        self.data['last_update']['cmd_vel'] = time.time()
    
    def robot_command_callback(self, msg):
        """机器人命令回调"""
        self.data['robot_command'] = {
            'command': msg.command,
            'speed': msg.speed,
            'emergency_stop': msg.emergency_stop
        }
        self.data['last_update']['robot_command'] = time.time()
    
    def servo_status_callback(self, msg):
        """舵机状态回调"""
        self.data['servo_positions'] = list(msg.servo_positions)
        self.data['harvest_state'] = msg.harvest_state
        self.data['tracking_active'] = msg.tracking_active
        self.data['last_update']['servo'] = time.time()
    
    def harvest_status_callback(self, msg):
        """采摘状态回调"""
        try:
            status = json.loads(msg.data)
            self.data['harvest_status'] = status
            self.data['last_update']['harvest'] = time.time()
        except:
            pass
    
    def update_gui(self):
        """更新GUI显示"""
        self.gui.update_data(self.data, self.distance_history, 
                           self.fps_history, self.detection_history)


class DebugVisualizerGUI:
    """调试可视化GUI"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS2 瓶子检测系统调试器")
        self.root.geometry("1400x900")
        
        # 设置样式
        style = ttk.Style()
        style.theme_use('clam')
        
        # 创建主框架
        self.main_frame = ttk.Frame(self.root, padding="5")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=2)
        self.main_frame.rowconfigure(0, weight=1)
        
        # 创建各个面板
        self._create_left_panel()
        self._create_right_panel()
        self._create_bottom_panel()
        
        # 数据显示变量
        self.image_label = None
        self.last_image_update = 0
        
    def _create_left_panel(self):
        """创建左侧面板 - 图像和检测信息"""
        left_frame = ttk.LabelFrame(self.main_frame, text="视觉检测", padding="5")
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # 图像显示
        self.image_canvas = tk.Canvas(left_frame, width=640, height=480, bg='black')
        self.image_canvas.grid(row=0, column=0, padx=5, pady=5)
        
        # 检测信息
        info_frame = ttk.LabelFrame(left_frame, text="检测信息", padding="5")
        info_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), padx=5, pady=5)
        
        # 创建信息标签
        self.labels = {}
        info_items = [
            ('fps', 'FPS'),
            ('detection_count', '检测数量'),
            ('nearest_distance', '最近距离(m)'),
            ('confidence', '置信度'),
            ('status', '状态'),
            ('position', '3D位置')
        ]
        
        for i, (key, text) in enumerate(info_items):
            ttk.Label(info_frame, text=f"{text}:").grid(row=i, column=0, sticky=tk.W, padx=5)
            self.labels[key] = ttk.Label(info_frame, text="--")
            self.labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
    
    def _create_right_panel(self):
        """创建右侧面板 - 系统状态"""
        right_frame = ttk.Frame(self.main_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # 机器人状态
        robot_frame = ttk.LabelFrame(right_frame, text="机器人状态", padding="5")
        robot_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        self.robot_labels = {}
        robot_items = [
            ('robot_mode', '运行模式'),
            ('battery_level', '电池电量(%)'),
            ('cpu_usage', 'CPU使用率(%)'),
            ('current_speed', '当前速度'),
            ('harvested_count', '采摘数量'),
            ('position_xy', '位置(X,Y)'),
            ('position_gps', 'GPS(经纬度)')
        ]
        
        for i, (key, text) in enumerate(robot_items):
            ttk.Label(robot_frame, text=f"{text}:").grid(row=i, column=0, sticky=tk.W, padx=5)
            self.robot_labels[key] = ttk.Label(robot_frame, text="--")
            self.robot_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
        
        # 控制命令
        cmd_frame = ttk.LabelFrame(right_frame, text="控制命令", padding="5")
        cmd_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        self.cmd_text = scrolledtext.ScrolledText(cmd_frame, height=8, width=40)
        self.cmd_text.grid(row=0, column=0, padx=5, pady=5)
        
        # 系统日志
        log_frame = ttk.LabelFrame(right_frame, text="系统日志", padding="5")
        log_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, width=40)
        self.log_text.grid(row=0, column=0, padx=5, pady=5)
    
    def _create_bottom_panel(self):
        """创建底部面板 - 实时图表"""
        bottom_frame = ttk.LabelFrame(self.main_frame, text="实时数据", padding="5")
        bottom_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=5, pady=5)
        
        # 创建画布用于绘制图表
        self.chart_canvas = tk.Canvas(bottom_frame, height=200, bg='white')
        self.chart_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
    
    def update_image(self, cv_image):
        """更新图像显示"""
        try:
            # 转换为PIL图像
            image = Image.fromarray(cv_image)
            photo = ImageTk.PhotoImage(image=image)
            
            # 更新画布
            self.image_canvas.delete("all")
            self.image_canvas.create_image(320, 240, image=photo, anchor=tk.CENTER)
            self.image_canvas.image = photo  # 保持引用
            
            self.last_image_update = time.time()
        except Exception as e:
            print(f"更新图像失败: {e}")
    
    def update_data(self, data, distance_history, fps_history, detection_history):
        """更新所有数据显示"""
        # 更新检测信息
        self.labels['fps'].config(text=f"{data.get('fps', 0):.1f}")
        self.labels['detection_count'].config(text=str(data.get('detection_count', 0)))
        
        distance = data.get('nearest_distance', -1)
        if distance > 0:
            self.labels['nearest_distance'].config(text=f"{distance:.2f}")
        else:
            self.labels['nearest_distance'].config(text="--")
        
        # 更新最近瓶子信息
        if data.get('nearest_bottle'):
            bottle = data['nearest_bottle']
            self.labels['confidence'].config(text=f"{bottle.get('confidence', 0):.2f}")
            self.labels['status'].config(text=bottle.get('status', '--'))
            pos = bottle.get('position_3d', [0, 0, 0])
            self.labels['position'].config(text=f"({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        else:
            self.labels['confidence'].config(text="--")
            self.labels['status'].config(text="未检测到")
            self.labels['position'].config(text="--")
        
        # 更新机器人状态
        mode = data.get('robot_mode', 'unknown')
        if data.get('auto_harvest'):
            mode += " (采摘中)"
        self.robot_labels['robot_mode'].config(text=mode)
        self.robot_labels['battery_level'].config(text=f"{data.get('battery_level', 0):.1f}")
        self.robot_labels['cpu_usage'].config(text=f"{data.get('cpu_usage', 0):.1f}")
        self.robot_labels['current_speed'].config(text=f"{data.get('current_speed', 0):.2f}")
        self.robot_labels['harvested_count'].config(text=str(data.get('harvested_count', 0)))
        
        # 更新位置
        pos = data.get('position', {})
        self.robot_labels['position_xy'].config(text=f"({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})")
        self.robot_labels['position_gps'].config(text=f"({pos.get('lat', 0):.6f}, {pos.get('lon', 0):.6f})")
        
        # 更新控制命令
        if 'cmd_vel' in data:
            cmd = data['cmd_vel']
            cmd_text = f"线速度: ({cmd['linear_x']:.2f}, {cmd['linear_y']:.2f}, {cmd['linear_z']:.2f})\n"
            cmd_text += f"角速度: ({cmd['angular_x']:.2f}, {cmd['angular_y']:.2f}, {cmd['angular_z']:.2f})"
            self.cmd_text.delete(1.0, tk.END)
            self.cmd_text.insert(tk.END, cmd_text)
        
        # 更新系统日志
        current_time = time.time()
        for topic, last_time in data.get('last_update', {}).items():
            if current_time - last_time > 2.0:
                self.add_log(f"警告: {topic} 超过2秒未更新", 'warning')
        
        # 绘制实时图表
        self.draw_charts(distance_history, fps_history, detection_history)
    
    def draw_charts(self, distance_history, fps_history, detection_history):
        """绘制实时图表"""
        self.chart_canvas.delete("all")
        
        width = self.chart_canvas.winfo_width()
        height = self.chart_canvas.winfo_height()
        
        if width <= 1 or height <= 1:
            return
        
        # 绘制三个子图
        chart_width = width // 3
        
        # 距离历史
        self._draw_line_chart(distance_history, 0, 0, chart_width, height, 
                            "距离(m)", color='blue')
        
        # FPS历史
        self._draw_line_chart(fps_history, chart_width, 0, chart_width, height,
                            "FPS", color='green')
        
        # 检测数量历史
        self._draw_line_chart(detection_history, chart_width*2, 0, chart_width, height,
                            "检测数", color='red')
    
    def _draw_line_chart(self, data, x, y, width, height, title, color='blue'):
        """绘制折线图"""
        if not data or len(data) < 2:
            return
        
        # 绘制标题
        self.chart_canvas.create_text(x + width//2, y + 10, text=title, 
                                     font=('Arial', 10, 'bold'))
        
        # 计算数据范围
        min_val = min(data)
        max_val = max(data)
        if max_val == min_val:
            max_val = min_val + 1
        
        # 绘制坐标轴
        margin = 20
        chart_x = x + margin
        chart_y = y + margin
        chart_width = width - 2*margin
        chart_height = height - 2*margin
        
        # Y轴
        self.chart_canvas.create_line(chart_x, chart_y, chart_x, chart_y + chart_height)
        # X轴
        self.chart_canvas.create_line(chart_x, chart_y + chart_height, 
                                     chart_x + chart_width, chart_y + chart_height)
        
        # 绘制数据点
        points = []
        for i, value in enumerate(data):
            px = chart_x + (i / (len(data)-1)) * chart_width
            py = chart_y + chart_height - ((value - min_val) / (max_val - min_val)) * chart_height
            points.extend([px, py])
        
        if len(points) >= 4:
            self.chart_canvas.create_line(points, fill=color, width=2)
        
        # 显示最新值
        latest_val = data[-1]
        self.chart_canvas.create_text(x + width - margin, y + margin, 
                                     text=f"{latest_val:.2f}", 
                                     anchor='ne', fill=color)
    
    def add_log(self, message, level='info'):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # 限制日志长度
        if int(self.log_text.index('end-1c').split('.')[0]) > 100:
            self.log_text.delete('1.0', '2.0')
    
    def run(self):
        """运行GUI"""
        self.root.mainloop()


def main(args=None):
    """主函数"""
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建GUI
    gui = DebugVisualizerGUI()
    
    # 创建ROS2节点
    node = DebugVisualizerNode(gui)
    
    # 在单独线程运行ROS2
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # 运行GUI主循环
        gui.run()
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()