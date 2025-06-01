#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
集成瓶子检测节点
整合了异步瓶子检测、双目深度估计、视频质量控制等功能
支持手动和自动两种工作模式
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String, Float32, Int32, Header
from bottle_detection_msgs.msg import BottleDetection, ServoCommand
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import json
from queue import Queue, Empty
from .stereo_camera import StereoCamera
from .bottle_rknn_pool import BottleRKNNPoolExecutor
from .bottle_detector_async import detect_bottle_async, draw_detections
from .utils import MedianFilter

class IntegratedBottleDetectionNode(Node):
    """集成瓶子检测节点类"""
    
    def __init__(self):
        super().__init__('integrated_bottle_detection_node')
        
        # 声明参数
        self._declare_parameters()
        self._get_parameters()
        
        # 初始化CV Bridge
        self.cv_bridge = CvBridge()
        
        # 初始化双目相机
        self.stereo_camera = StereoCamera(
            self.camera_id,
            self.camera_width,
            self.camera_height
        )
        
        # 初始化异步瓶子检测器
        try:
            self.bottle_detector_pool = BottleRKNNPoolExecutor(
                model_path=self.model_path,
                detector_func=detect_bottle_async,
                thread_num=self.thread_num,
                queue_size=self.queue_size
            )
            self.get_logger().info(f'异步检测器初始化成功，线程数: {self.thread_num}')
        except Exception as e:
            self.get_logger().error(f'异步检测器初始化失败: {e}')
            raise RuntimeError('异步检测器初始化失败')
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建发布者
        self._create_publishers(qos_profile)
        
        # 创建订阅者
        self._create_subscribers()
        
        # 初始化系统
        self._initialize_system()
        
        # 创建处理定时器
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.timer_callback)
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 状态变量
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        self.current_mode = "manual"
        self.auto_harvest_active = False
        
        # 帧数据缓存
        self.frame_data_cache = {}
        self.max_cache_size = self.thread_num * 3
        
        # 瓶子检测结果
        self.bottle_detections_with_distance = []
        self.nearest_bottle_distance = None
        
        # 距离滤波器
        self.distance_filter = MedianFilter(window_size=5)
        
        # 视频质量控制
        self.quality_presets = {
            "high": {"resolution": (640, 480), "quality": 80},
            "medium": {"resolution": (480, 360), "quality": 70},
            "low": {"resolution": (320, 240), "quality": 60},
            "very_low": {"resolution": (240, 180), "quality": 50},
            "minimum": {"resolution": (160, 120), "quality": 40}
        }
        self.current_quality = "medium"
        
        # 舵机跟踪控制
        self.enable_servo_tracking = True
        self.servo_tracking_active = False
        
        # 预填充处理管道
        self._prefill_pipeline()
        
        self.get_logger().info(
            f'集成瓶子检测节点已启动\n'
            f'相机ID: {self.camera_id}\n'
            f'模型路径: {self.model_path}\n'
            f'线程数: {self.thread_num}\n'
            f'发布频率: {self.publish_rate} Hz'
        )
    
    def _declare_parameters(self):
        """声明ROS2参数"""
        # 相机参数
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('calibration_file', '')
        
        # 模型参数
        self.declare_parameter('model_path', 'yolo11n.rknn')
        self.declare_parameter('model_size', [640, 640])
        
        # 检测参数
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('max_distance', 5.0)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 发布参数
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        # 显示参数
        self.declare_parameter('show_display', True)
        
        # 异步处理参数
        self.declare_parameter('thread_num', 3)
        self.declare_parameter('queue_size', 10)
        
        # 舵机跟踪参数
        self.declare_parameter('enable_servo_tracking', True)
    
    def _get_parameters(self):
        """获取参数值"""
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        self.model_path = self.get_parameter('model_path').value
        self.model_size = self.get_parameter('model_size').value
        
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.show_display = self.get_parameter('show_display').value
        
        self.thread_num = self.get_parameter('thread_num').value
        self.queue_size = self.get_parameter('queue_size').value
        
        self.enable_servo_tracking = self.get_parameter('enable_servo_tracking').value
    
    def _create_publishers(self, qos_profile):
        """创建发布者"""
        # 图像发布者
        self.left_image_pub = self.create_publisher(
            Image, 'camera/left/image_raw', qos_profile)
        self.right_image_pub = self.create_publisher(
            Image, 'camera/right/image_raw', qos_profile)
        self.annotated_image_pub = self.create_publisher(
            Image, 'bottle_detection/annotated_image', qos_profile)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, 'bottle_detection/compressed_image', qos_profile)
        
        # 检测结果发布者
        self.bottle_position_pub = self.create_publisher(
            PointStamped, 'bottle_detection/nearest_position', 10)
        self.bottle_distance_pub = self.create_publisher(
            Float32, 'bottle_detection/nearest_distance', 10)
        self.bottle_count_pub = self.create_publisher(
            Int32, 'bottle_detection/count', 10)
        self.detection_info_pub = self.create_publisher(
            String, 'bottle_detection/info', 10)
        
        # 自定义消息发布者
        self.bottle_detection_pub = self.create_publisher(
            BottleDetection, 'bottle_detection/detection', 10)
        
        # 舵机跟踪目标发布者
        self.tracking_target_pub = self.create_publisher(
            Point, 'servo/tracking_target', 10)
    
    def _create_subscribers(self):
        """创建订阅者"""
        # 模式控制订阅
        self.mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.mode_callback,
            10
        )
        
        # 视频质量控制订阅
        self.quality_sub = self.create_subscription(
            String,
            'video/quality_preset',
            self.quality_callback,
            10
        )
    
    def _initialize_system(self):
        """初始化系统组件"""
        # 打开相机
        if not self.stereo_camera.open_camera():
            self.get_logger().error('无法打开相机，节点将退出')
            raise RuntimeError('相机初始化失败')
        
        # 加载相机参数
        if self.calibration_file:
            self.stereo_camera.load_camera_params(self.calibration_file)
        else:
            self.stereo_camera.load_camera_params()
        
        # 设置双目校正
        self.stereo_camera.setup_stereo_rectification()
        
        # 设置距离范围
        self.stereo_camera.set_distance_range(self.min_distance, self.max_distance)
        
        self.get_logger().info('系统初始化完成')
    
    def _prefill_pipeline(self):
        """预填充处理管道"""
        self.get_logger().info('预填充处理管道...')
        for i in range(self.thread_num):
            frame_left, frame_right = self.stereo_camera.capture_frame()
            if frame_left is None or frame_right is None:
                continue
            
            frame_id = self.bottle_detector_pool.put(frame_left)
            
            self.frame_data_cache[frame_id] = {
                'left': frame_left.copy(),
                'right': frame_right.copy(),
                'timestamp': self.get_clock().now().to_msg()
            }
    
    def timer_callback(self):
        """定时器回调函数"""
        with self.lock:
            # 读取双目图像
            frame_left, frame_right = self.stereo_camera.capture_frame()
            if frame_left is None or frame_right is None:
                self.get_logger().warn('无法读取相机图像', throttle_duration_sec=1.0)
                return
            
            timestamp = self.get_clock().now().to_msg()
            
            # 提交新帧到异步处理队列
            if not self.bottle_detector_pool.is_full():
                frame_id = self.bottle_detector_pool.put(frame_left)
                
                self.frame_data_cache[frame_id] = {
                    'left': frame_left.copy(),
                    'right': frame_right.copy(),
                    'timestamp': timestamp
                }
                
                # 清理过期缓存
                if len(self.frame_data_cache) > self.max_cache_size:
                    oldest_id = min(self.frame_data_cache.keys())
                    del self.frame_data_cache[oldest_id]
            
            # 尝试获取处理结果
            result_frame_id, result, success = self.bottle_detector_pool.get(timeout=0.001)
            
            if success and result is not None:
                _, _, bottle_detections = result
                
                if result_frame_id in self.frame_data_cache:
                    frame_data = self.frame_data_cache[result_frame_id]
                    cached_left = frame_data['left']
                    cached_right = frame_data['right']
                    cached_timestamp = frame_data['timestamp']
                    
                    del self.frame_data_cache[result_frame_id]
                    
                    # 处理检测结果
                    self._process_detection_result(
                        cached_left, cached_right, bottle_detections, cached_timestamp)
            
            # 更新FPS
            self._update_fps()
    
    def _process_detection_result(self, frame_left, frame_right, bottle_detections, timestamp):
        """处理检测结果并发布"""
        # 校正图像
        frame_left_rectified, img_left_gray, img_right_gray = \
            self.stereo_camera.rectify_stereo_images(frame_left, frame_right)
        
        # 计算视差和3D点云
        disparity, disp_normalized = self.stereo_camera.compute_disparity(
            img_left_gray, img_right_gray)
        threeD = self.stereo_camera.compute_3d_points(disparity)
        
        # 处理检测结果
        annotated_image = frame_left.copy()
        nearest_bottle = None
        min_distance = float('inf')
        
        # 计算每个瓶子的距离
        valid_detections = []
        for detection in bottle_detections:
            left, top, right, bottom, score, cx, cy = detection
            
            if score < self.confidence_threshold:
                continue
            
            # 计算距离
            distance = self.stereo_camera.get_bottle_distance(threeD, cx, cy)
            
            if distance and self.min_distance <= distance <= self.max_distance:
                # 应用距离滤波
                filtered_distance = self.distance_filter.update(distance)
                
                valid_detections.append({
                    'detection': detection,
                    'distance': filtered_distance,
                    '3d_position': threeD[cy][cx] if threeD is not None else None
                })
                
                if filtered_distance < min_distance:
                    min_distance = filtered_distance
                    nearest_bottle = valid_detections[-1]
        
        # 更新全局状态
        self.bottle_detections_with_distance = valid_detections
        self.nearest_bottle_distance = min_distance if nearest_bottle else None
        
        # 在图像上绘制检测结果
        self._draw_detections_on_image(annotated_image, valid_detections)
        
        # 添加状态信息
        self._draw_status_info(annotated_image)
        
        # 发布结果
        self._publish_raw_images(frame_left, frame_right, timestamp)
        self._publish_annotated_image(annotated_image, timestamp)
        self._publish_detection_results(nearest_bottle, len(valid_detections), timestamp)
        
        # 手动模式下的舵机跟踪
        if self.current_mode == "manual" and self.enable_servo_tracking and nearest_bottle:
            self._publish_tracking_target(nearest_bottle)
        
        # 显示图像
        if self.show_display:
            self._display_images(annotated_image, disp_normalized)
    
    def _draw_detections_on_image(self, image, valid_detections):
        """在图像上绘制检测结果"""
        for valid_det in valid_detections:
            detection = valid_det['detection']
            distance = valid_det['distance']
            left, top, right, bottom, score, cx, cy = detection
            
            # 绘制边界框
            color = (0, 255, 0)  # 绿色
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, 2)
            
            # 绘制标签
            label = f"bottle: {score:.2f}"
            cv2.putText(image, label, (int(left), int(top-10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 绘制距离
            distance_text = f"{distance:.2f}m"
            cv2.putText(image, distance_text, (int(cx-20), int(cy)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # 绘制中心点
            cv2.circle(image, (int(cx), int(cy)), 5, (255, 0, 0), -1)
    
    def _draw_status_info(self, image):
        """在图像上绘制状态信息"""
        # FPS
        cv2.putText(image, f"FPS: {self.current_fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 模式
        mode_text = f"Mode: {self.current_mode.upper()}"
        if self.current_mode == "auto" and self.auto_harvest_active:
            mode_text += " (HARVEST)"
        cv2.putText(image, mode_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 队列大小
        queue_text = f"Queue: {self.bottle_detector_pool.get_queue_size()}"
        cv2.putText(image, queue_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    def _publish_raw_images(self, left_image, right_image, timestamp):
        """发布原始图像"""
        try:
            # 左相机图像
            left_msg = self.cv_bridge.cv2_to_imgmsg(left_image, 'bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'left_camera'
            self.left_image_pub.publish(left_msg)
            
            # 右相机图像
            right_msg = self.cv_bridge.cv2_to_imgmsg(right_image, 'bgr8')
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'right_camera'
            self.right_image_pub.publish(right_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布原始图像失败: {e}')
    
    def _publish_annotated_image(self, image, timestamp):
        """发布标注图像"""
        try:
            # 根据质量设置调整大小
            quality_config = self.quality_presets[self.current_quality]
            resized = cv2.resize(image, quality_config["resolution"])
            
            # 发布原始格式
            msg = self.cv_bridge.cv2_to_imgmsg(resized, 'bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'left_camera'
            self.annotated_image_pub.publish(msg)
            
            # 发布压缩格式
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = cv2.imencode(
                    '.jpg', resized,
                    [cv2.IMWRITE_JPEG_QUALITY, quality_config["quality"]])[1].tostring()
                self.compressed_image_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布标注图像失败: {e}')
    
    def _publish_detection_results(self, nearest_bottle, bottle_count, timestamp):
        """发布检测结果"""
        # 发布瓶子数量
        count_msg = Int32()
        count_msg.data = bottle_count
        self.bottle_count_pub.publish(count_msg)
        
        if nearest_bottle:
            detection = nearest_bottle['detection']
            distance = nearest_bottle['distance']
            position_3d = nearest_bottle['3d_position']
            left, top, right, bottom, score, cx, cy = detection
            
            # 发布距离
            distance_msg = Float32()
            distance_msg.data = distance
            self.bottle_distance_pub.publish(distance_msg)
            
            # 发布3D位置
            if position_3d is not None:
                position_msg = PointStamped()
                position_msg.header.stamp = timestamp
                position_msg.header.frame_id = 'left_camera'
                position_msg.point.x = float(position_3d[0] / 1000.0)
                position_msg.point.y = float(position_3d[1] / 1000.0)
                position_msg.point.z = float(position_3d[2] / 1000.0)
                self.bottle_position_pub.publish(position_msg)
            
            # 发布详细信息
            info = {
                'bottle_detected': True,
                'nearest_bottle': {
                    'pixel_x': int(cx),
                    'pixel_y': int(cy),
                    'bbox': [int(left), int(top), int(right), int(bottom)],
                    'confidence': float(score),
                    'distance': float(distance),
                    'status': self._get_distance_status(distance)
                },
                'total_count': bottle_count,
                'fps': float(self.current_fps),
                'timestamp': time.time()
            }
            
            # 发布自定义消息
            detection_msg = BottleDetection()
            detection_msg.header.stamp = timestamp
            detection_msg.header.frame_id = 'left_camera'
            detection_msg.bottle_detected = True
            detection_msg.bottle_count = bottle_count
            detection_msg.nearest_bottle_x = int(cx)
            detection_msg.nearest_bottle_y = int(cy)
            detection_msg.bbox_left = int(left)
            detection_msg.bbox_top = int(top)
            detection_msg.bbox_right = int(right)
            detection_msg.bbox_bottom = int(bottom)
            detection_msg.confidence = float(score)
            detection_msg.distance = float(distance)
            detection_msg.position_x = float(position_3d[0] / 1000.0) if position_3d else 0.0
            detection_msg.position_y = float(position_3d[1] / 1000.0) if position_3d else 0.0
            detection_msg.position_z = float(position_3d[2] / 1000.0) if position_3d else 0.0
            detection_msg.image_width = self.camera_width // 2
            detection_msg.image_height = self.camera_height
            detection_msg.status = self._get_distance_status(distance)
            self.bottle_detection_pub.publish(detection_msg)
        else:
            # 没有检测到瓶子
            distance_msg = Float32()
            distance_msg.data = -1.0
            self.bottle_distance_pub.publish(distance_msg)
            
            info = {
                'bottle_detected': False,
                'total_count': 0,
                'fps': float(self.current_fps),
                'timestamp': time.time()
            }
            
            # 发布空的检测消息
            detection_msg = BottleDetection()
            detection_msg.header.stamp = timestamp
            detection_msg.header.frame_id = 'left_camera'
            detection_msg.bottle_detected = False
            detection_msg.bottle_count = 0
            detection_msg.distance = -1.0
            detection_msg.status = "未检测到目标"
            self.bottle_detection_pub.publish(detection_msg)
        
        # 发布JSON信息
        info_msg = String()
        info_msg.data = json.dumps(info, ensure_ascii=False)
        self.detection_info_pub.publish(info_msg)
    
    def _publish_tracking_target(self, nearest_bottle):
        """发布舵机跟踪目标"""
        detection = nearest_bottle['detection']
        _, _, _, _, _, cx, cy = detection
        
        tracking_msg = Point()
        tracking_msg.x = float(cx)
        tracking_msg.y = float(cy)
        tracking_msg.z = float(self.camera_width // 2)  # 传递图像宽度
        
        self.tracking_target_pub.publish(tracking_msg)
    
    def _get_distance_status(self, distance):
        """根据距离返回状态描述"""
        if distance < 0.5:
            return "距离过近"
        elif distance < 0.8:
            return "采摘距离"
        elif distance < 1.5:
            return "正常"
        elif distance < 3.0:
            return "距离较远"
        else:
            return "距离过远"
    
    def _update_fps(self):
        """更新FPS计算"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def _display_images(self, annotated_image, disparity):
        """显示图像窗口"""
        cv2.imshow('Bottle Detection', annotated_image)
        cv2.imshow('Disparity', disparity)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('用户请求退出')
            rclpy.shutdown()
    
    def mode_callback(self, msg):
        """模式更新回调"""
        try:
            data = json.loads(msg.data)
            self.current_mode = data.get("mode", "manual")
            self.auto_harvest_active = data.get("auto_harvest", False)
            
            self.get_logger().info(
                f'模式更新: {self.current_mode}, '
                f'自动采摘: {self.auto_harvest_active}'
            )
        except Exception as e:
            self.get_logger().error(f'解析模式数据错误: {e}')
    
    def quality_callback(self, msg):
        """视频质量更新回调"""
        quality = msg.data
        if quality in self.quality_presets:
            self.current_quality = quality
            self.get_logger().info(f'视频质量设置为: {quality}')
        else:
            self.get_logger().warn(f'未知的质量预设: {quality}')
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在清理资源...')
        
        if hasattr(self, 'stereo_camera'):
            self.stereo_camera.close_camera()
        
        if hasattr(self, 'bottle_detector_pool'):
            self.bottle_detector_pool.release()
        
        if self.show_display:
            cv2.destroyAllWindows()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IntegratedBottleDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()