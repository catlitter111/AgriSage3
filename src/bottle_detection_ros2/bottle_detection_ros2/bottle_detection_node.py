#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测ROS2主节点 - 修复版
修复相机打开和shutdown问题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32, Int32, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import json
import sys
import signal

# 全局变量防止重复shutdown
shutdown_called = False

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global shutdown_called
    if not shutdown_called:
        print("\n接收到中断信号，正在关闭...")
        shutdown_called = True
    sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)

class BottleDetectionNode(Node):
    """瓶子检测主节点类"""
    
    def __init__(self):
        super().__init__('bottle_detection_node')
        
        self.get_logger().info('=== 瓶子检测节点初始化开始 ===')
        
        # 声明ROS2参数
        self._declare_parameters()
        
        # 获取参数值
        self._get_parameters()
        
        # 初始化CV Bridge
        self.cv_bridge = CvBridge()
        
        # 标志位
        self.camera_opened = False
        self.model_loaded = False
        
        # 先创建发布者（不需要相机和模型）
        self._create_publishers()
        
        # 尝试初始化相机（如果失败不影响节点启动）
        self._init_camera()
        
        # 尝试初始化模型（如果失败不影响节点启动）
        self._init_model()
        
        # 创建处理定时器
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.timer_callback)
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 状态变量
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        
        self.get_logger().info('=== 瓶子检测节点初始化完成 ===')
        self.get_logger().info(f'相机状态: {"已打开" if self.camera_opened else "未打开"}')
        self.get_logger().info(f'模型状态: {"已加载" if self.model_loaded else "未加载"}')
    
    def _declare_parameters(self):
        """声明ROS2参数"""
        # 相机参数
        self.declare_parameter('camera_id', 1)  # 默认使用相机1
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('calibration_file', '')
        
        # 模型参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_size', [640, 640])
        
        # 检测参数
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('max_distance', 5.0)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 发布参数
        self.declare_parameter('publish_rate', 10.0)  # 降低默认频率
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        # 显示参数
        self.declare_parameter('show_display', False)
    
    def _get_parameters(self):
        """获取参数值"""
        # 相机参数
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # 模型参数
        self.model_path = self.get_parameter('model_path').value
        self.model_size = self.get_parameter('model_size').value
        
        # 检测参数
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # 发布参数
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # 显示参数
        self.show_display = self.get_parameter('show_display').value
    
    def _create_publishers(self):
        """创建发布者"""
        # 设置QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建发布者
        self.left_image_pub = self.create_publisher(
            Image, 'camera/left/image_raw', qos_profile)
        self.right_image_pub = self.create_publisher(
            Image, 'camera/right/image_raw', qos_profile)
        self.annotated_image_pub = self.create_publisher(
            Image, 'bottle_detection/annotated_image', qos_profile)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, 'bottle_detection/compressed_image', qos_profile)
        
        # 使用标准消息类型发布检测结果
        self.bottle_position_pub = self.create_publisher(
            PointStamped, 'bottle_detection/nearest_position', 10)
        self.bottle_distance_pub = self.create_publisher(
            Float32, 'bottle_detection/nearest_distance', 10)
        self.bottle_count_pub = self.create_publisher(
            Int32, 'bottle_detection/count', 10)
        self.detection_info_pub = self.create_publisher(
            String, 'bottle_detection/info', 10)
    
    def _init_camera(self):
        """初始化相机（简化版）"""
        try:
            self.get_logger().info(f'尝试打开相机 ID={self.camera_id}...')
            
            # 使用OpenCV直接打开相机，避免GStreamer
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                # 尝试默认后端
                self.cap = cv2.VideoCapture(self.camera_id)
            
            if self.cap.isOpened():
                # 设置缓冲区大小
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # 读取实际参数
                width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = self.cap.get(cv2.CAP_PROP_FPS)
                
                self.get_logger().info(f'相机打开成功: {width}x{height} @ {fps}fps')
                
                # 测试读取
                ret, frame = self.cap.read()
                if ret:
                    self.get_logger().info('相机测试读取成功')
                    self.camera_opened = True
                else:
                    self.get_logger().warn('相机打开了但无法读取帧')
                    self.cap.release()
                    self.cap = None
            else:
                self.get_logger().error('无法打开相机')
                
        except Exception as e:
            self.get_logger().error(f'相机初始化失败: {str(e)}')
            self.camera_opened = False
    
    def _init_model(self):
        """初始化模型（可选）"""
        if not self.model_path:
            self.get_logger().info('未指定模型路径，跳过模型加载')
            return
        
        try:
            self.get_logger().info(f'加载模型: {self.model_path}')
            # 这里暂时跳过实际的模型加载
            # from .bottle_detector import BottleDetector
            # self.bottle_detector = BottleDetector(self.model_path, tuple(self.model_size))
            # self.model_loaded = self.bottle_detector.load_model()
            self.model_loaded = False  # 暂时设为False
        except Exception as e:
            self.get_logger().error(f'模型加载失败: {str(e)}')
            self.model_loaded = False
    
    def timer_callback(self):
        """定时器回调函数 - 主处理循环"""
        if not self.camera_opened:
            return
        
        with self.lock:
            try:
                # 读取图像
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn('无法读取相机图像', throttle_duration_sec=1.0)
                    return
                
                # 获取时间戳
                timestamp = self.get_clock().now().to_msg()
                
                # 处理图像
                self._process_frame(frame, timestamp)
                
                # 更新FPS
                self._update_fps()
                
                # 显示图像（如果启用）
                if self.show_display:
                    self._display_frame(frame)
                    
            except Exception as e:
                self.get_logger().error(f'处理帧时出错: {str(e)}')
    
    def _process_frame(self, frame, timestamp):
        """处理单帧图像"""
        # 获取图像尺寸
        h, w = frame.shape[:2]
        
        # 判断是否为双目相机
        if w > h * 1.5:  # 宽度是高度的1.5倍以上
            # 分割左右图像
            mid = w // 2
            frame_left = frame[:, :mid]
            frame_right = frame[:, mid:]
        else:
            # 单目相机，复制为左右图像
            frame_left = frame
            frame_right = frame.copy()
        
        # 发布原始图像
        self._publish_raw_images(frame_left, frame_right, timestamp)
        
        # 创建标注图像（暂时只添加FPS）
        annotated_image = frame_left.copy()
        cv2.putText(annotated_image, f'FPS: {self.current_fps:.1f}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 0), 2)
        
        # 添加状态信息
        status_text = f'Camera: {self.camera_id} | Model: {"Loaded" if self.model_loaded else "Not Loaded"}'
        cv2.putText(annotated_image, status_text, 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (255, 255, 0), 2)
        
        # 发布标注图像
        self._publish_annotated_image(annotated_image, timestamp)
        
        # 发布空的检测结果（因为没有加载模型）
        self._publish_empty_detection(timestamp)
    
    def _publish_raw_images(self, left_image, right_image, timestamp):
        """发布原始左右相机图像"""
        try:
            # 发布左相机图像
            left_msg = self.cv_bridge.cv2_to_imgmsg(left_image, 'bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'left_camera'
            self.left_image_pub.publish(left_msg)
            
            # 发布右相机图像
            right_msg = self.cv_bridge.cv2_to_imgmsg(right_image, 'bgr8')
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'right_camera'
            self.right_image_pub.publish(right_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布原始图像失败: {str(e)}')
    
    def _publish_annotated_image(self, image, timestamp):
        """发布标注后的图像"""
        try:
            # 发布原始格式
            msg = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'left_camera'
            self.annotated_image_pub.publish(msg)
            
            # 发布压缩格式（如果启用）
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = cv2.imencode(
                    '.jpg', image, 
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])[1].tostring()
                self.compressed_image_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布标注图像失败: {str(e)}')
    
    def _publish_empty_detection(self, timestamp):
        """发布空的检测结果"""
        # 发布瓶子数量（0）
        count_msg = Int32()
        count_msg.data = 0
        self.bottle_count_pub.publish(count_msg)
        
        # 发布距离（-1表示无检测）
        distance_msg = Float32()
        distance_msg.data = -1.0
        self.bottle_distance_pub.publish(distance_msg)
        
        # 发布检测信息
        info = {
            'bottle_detected': False,
            'total_count': 0,
            'timestamp': time.time(),
            'status': 'No model loaded' if not self.model_loaded else 'No detection'
        }
        info_msg = String()
        info_msg.data = json.dumps(info, ensure_ascii=False)
        self.detection_info_pub.publish(info_msg)
    
    def _update_fps(self):
        """更新FPS计算"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def _display_frame(self, frame):
        """显示图像"""
        cv2.imshow('Bottle Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('用户请求退出')
            global shutdown_called
            shutdown_called = True
            raise KeyboardInterrupt()
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在清理资源...')
        
        # 关闭相机
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        
        # 关闭显示窗口
        if self.show_display:
            cv2.destroyAllWindows()
        
        super().destroy_node()

def main(args=None):
    """主函数"""
    global shutdown_called
    
    try:
        rclpy.init(args=args)
        node = BottleDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        
        # 只在未调用过的情况下调用shutdown
        if not shutdown_called:
            shutdown_called = True
            rclpy.shutdown()

if __name__ == '__main__':
    main()