#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测节点 - 订阅图像，进行瓶子检测，并发布检测结果
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from bottle_detection_robot.bottle_detector import BottleDetector
from bottle_detection_robot.stereo_camera import StereoCamera
from bottle_detection_robot.msg import BottleDetection

class BottleDetectorNode(Node):
    """
    瓶子检测节点类，负责订阅图像，进行瓶子检测，并发布检测结果
    """
    
    def __init__(self):
        """初始化瓶子检测节点"""
        super().__init__('bottle_detector_node')
        
        # 声明参数
        self.declare_parameter('model_path', '')                # RKNN模型路径
        self.declare_parameter('model_size', [640, 640])        # 模型输入尺寸
        self.declare_parameter('detection_threshold', 0.2)      # 检测阈值
        self.declare_parameter('detection_interval', 0.1)       # 检测间隔（秒）
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        model_size = tuple(self.get_parameter('model_size').value)
        self.detection_threshold = self.get_parameter('detection_threshold').value
        detection_interval = self.get_parameter('detection_interval').value
        
        # 如果没有提供模型路径，使用默认路径
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('bottle_detection_robot'),
                'models',
                'yolo11n.rknn'
            )
            if not os.path.exists(model_path):
                self.get_logger().error(f'找不到RKNN模型: {model_path}')
                return
        
        # 创建发布者
        self.detection_pub = self.create_publisher(BottleDetection, 'bottle/detection', 10)
        self.detection_image_pub = self.create_publisher(Image, 'bottle/detection_image', 10)
        
        # 创建订阅者
        self.left_sub = self.create_subscription(
            Image,
            'camera/left/image_raw',
            self.left_image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            'camera/depth',
            self.depth_callback,
            10
        )
        
        # 创建CV bridge
        self.bridge = CvBridge()
        
        # 初始化瓶子检测器
        self.get_logger().info(f'初始化瓶子检测器，模型路径: {model_path}')
        self.bottle_detector = BottleDetector(model_path, model_size)
        
        # 加载模型
        if not self.bottle_detector.load_model():
            self.get_logger().error('加载RKNN模型失败，节点将停止')
            return
        
        # 初始化3D相机对象，用于距离计算
        self.stereo_camera = StereoCamera()
        
        # 存储最新的帧和深度图
        self.latest_frame = None
        self.latest_depth = None
        self.latest_3d_points = None
        
        # 创建定时器，定期进行检测
        self.timer = self.create_timer(detection_interval, self.detection_timer_callback)
        
        self.get_logger().info('瓶子检测节点已启动，等待图像数据...')
    
    def left_image_callback(self, msg):
        """左相机图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV图像
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().debug('接收到左相机图像')
        except Exception as e:
            self.get_logger().error(f'处理左相机图像时出错: {e}')
    
    def depth_callback(self, msg):
        """深度图回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV图像
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            self.get_logger().debug('接收到深度图')
            
            # 如果需要，可以在这里将深度图转换为3D点云
            # 但由于我们在相机节点已经计算了3D点云，这里主要使用深度图来计算距离
        except Exception as e:
            self.get_logger().error(f'处理深度图时出错: {e}')
    
    def get_bottle_distance(self, depth_img, cx, cy, radius=3):
        """
        计算瓶子的距离
        
        参数:
        depth_img -- 深度图
        cx, cy -- 瓶子中心坐标
        radius -- 计算区域半径
        
        返回:
        距离(米)
        """
        if depth_img is None:
            return None
            
        # 计算区域内的平均深度值
        roi = depth_img[
            max(0, cy-radius):min(depth_img.shape[0], cy+radius+1),
            max(0, cx-radius):min(depth_img.shape[1], cx+radius+1)
        ]
        
        if roi.size == 0:
            return None
            
        # 忽略零值（无效深度值）
        valid_depths = roi[roi > 0]
        if valid_depths.size == 0:
            return None
            
        # 计算平均深度，并转换为米
        # 注意：这里的深度值范围是0-255，需要映射到实际距离
        # 这个映射关系需要根据相机标定结果进行调整
        depth_value = np.median(valid_depths)
        # 假设深度值是线性映射到0-5米的范围
        distance = depth_value / 255.0 * 5.0
        
        return distance
    
    def detection_timer_callback(self):
        """检测定时器回调函数，进行瓶子检测并发布结果"""
        # 检查是否有可用的帧和深度图
        if self.latest_frame is None or self.latest_depth is None:
            return
        
        try:
            # 进行瓶子检测
            bottle_detections = self.bottle_detector.detect(self.latest_frame)
            
            # 如果检测到瓶子
            if bottle_detections:
                # 创建结果图像
                result_image = self.latest_frame.copy()
                
                # 计算每个瓶子的距离，并保存带距离信息的检测结果
                bottle_detections_with_distance = []
                for left, top, right, bottom, score, cx, cy in bottle_detections:
                    # 计算瓶子距离
                    distance = self.get_bottle_distance(self.latest_depth, cx, cy)
                    
                    if distance is not None:
                        bottle_detections_with_distance.append((left, top, right, bottom, score, distance, cx, cy))
                        # 在图像上绘制检测结果和距离
                        self.bottle_detector.draw_detection(result_image, (left, top, right, bottom, score), distance)
                    else:
                        # 如果无法计算距离，只绘制检测结果
                        self.bottle_detector.draw_detection(result_image, (left, top, right, bottom, score))
                
                # 如果有带距离信息的瓶子
                if bottle_detections_with_distance:
                    # 找出距离最近的瓶子
                    nearest_bottle = min(bottle_detections_with_distance, key=lambda x: x[5])
                    left, top, right, bottom, score, distance, cx, cy = nearest_bottle
                    
                    # 创建并发布瓶子检测消息
                    detection_msg = BottleDetection()
                    detection_msg.header.stamp = self.get_clock().now().to_msg()
                    detection_msg.header.frame_id = 'camera_left_frame'
                    detection_msg.distance = distance
                    detection_msg.x = cx
                    detection_msg.y = cy
                    detection_msg.score = score
                    
                    self.detection_pub.publish(detection_msg)
                    self.get_logger().info(f'检测到最近的瓶子: 距离={distance:.2f}m, 坐标=({cx},{cy}), 分数={score:.2f}')
                
                # 发布检测结果图像
                result_image_msg = self.bridge.cv2_to_imgmsg(result_image, 'bgr8')
                result_image_msg.header.stamp = self.get_clock().now().to_msg()
                result_image_msg.header.frame_id = 'camera_left_frame'
                self.detection_image_pub.publish(result_image_msg)
                
            else:
                self.get_logger().debug('未检测到瓶子')
        
        except Exception as e:
            self.get_logger().error(f'瓶子检测过程中出错: {e}')
    
    def destroy_node(self):
        """节点销毁时释放资源"""
        if hasattr(self, 'bottle_detector'):
            self.bottle_detector.release_model()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    bottle_detector_node = BottleDetectorNode()
    
    try:
        rclpy.spin(bottle_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        bottle_detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()