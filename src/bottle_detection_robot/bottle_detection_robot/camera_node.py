#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机节点 - 读取双目摄像头图像并发布
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from bottle_detection_robot.stereo_camera import StereoCamera

class CameraNode(Node):
    """
    相机节点类，负责读取双目摄像头图像并发布
    """
    
    def __init__(self):
        """初始化相机节点"""
        super().__init__('camera_node')
        
        # 声明参数
        self.declare_parameter('camera_id', 0)           # 相机ID，默认为0
        self.declare_parameter('frame_width', 1280)      # 画面宽度
        self.declare_parameter('frame_height', 480)      # 画面高度
        self.declare_parameter('fps', 10)                # 帧率
        self.declare_parameter('camera_params_file', '') # 相机参数文件路径
        
        # 获取参数
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        camera_params_file = self.get_parameter('camera_params_file').value
        
        # 创建图像发布者
        self.left_pub = self.create_publisher(Image, 'camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'camera/right/image_raw', 10)
        self.disparity_pub = self.create_publisher(Image, 'camera/disparity', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth', 10)
        
        # 创建相机信息发布者
        self.left_info_pub = self.create_publisher(CameraInfo, 'camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'camera/right/camera_info', 10)
        
        # 创建CV bridge
        self.bridge = CvBridge()
        
        # 初始化相机
        self.get_logger().info('初始化双目相机...')
        self.stereo_camera = StereoCamera(self.camera_id, self.frame_width, self.frame_height)
        
        # 加载相机参数
        if camera_params_file:
            # 使用提供的参数文件路径
            params_path = camera_params_file
        else:
            # 否则尝试在包路径下寻找默认参数文件
            params_path = os.path.join(
                get_package_share_directory('bottle_detection_robot'),
                'config',
                'out.xls'
            )
            if not os.path.exists(params_path):
                self.get_logger().warning(f"找不到相机参数文件: {params_path}，将使用硬编码参数")
                params_path = None
        
        # 加载相机参数
        self.get_logger().info(f'加载相机参数: {params_path if params_path else "使用硬编码参数"}')
        self.stereo_camera.load_camera_params(params_path)
        
        # 设置双目校正参数
        self.get_logger().info('设置双目校正参数')
        self.stereo_camera.setup_stereo_rectification((640, 480))
        
        # 打开相机
        if not self.stereo_camera.open_camera():
            self.get_logger().error('无法打开相机，节点将停止')
            return
        
        # 创建定时器，定期发布图像
        timer_period = 1.0 / self.fps  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('相机节点已启动，开始发布图像')
    
    def timer_callback(self):
        """定时器回调函数，读取并发布图像"""
        try:
            # 读取图像
            frame_left, frame_right = self.stereo_camera.capture_frame()
            if frame_left is None or frame_right is None:
                self.get_logger().warn('无法读取图像帧')
                return
            
            # 校正图像
            frame_left_rectified, img_left_rectified, img_right_rectified = self.stereo_camera.rectify_stereo_images(
                frame_left, frame_right)
            
            # 计算视差图
            disparity, disp_normalized = self.stereo_camera.compute_disparity(
                img_left_rectified, img_right_rectified)
            
            # 计算3D点云
            threeD = self.stereo_camera.compute_3d_points(disparity)
            
            # 创建深度图
            z_values = threeD[:, :, 2]
            # 将无效值设为最大值
            z_values[np.isinf(z_values)] = 0
            z_values[np.isnan(z_values)] = 0
            # 归一化深度图
            depth_normalized = np.zeros_like(z_values, dtype=np.uint8)
            valid_mask = (z_values > 0) & (z_values < 10000)  # 10米以内的有效值
            if np.any(valid_mask):
                min_val = np.min(z_values[valid_mask])
                max_val = np.max(z_values[valid_mask])
                # 归一化到0-255
                if max_val > min_val:
                    depth_normalized[valid_mask] = ((z_values[valid_mask] - min_val) / (max_val - min_val) * 255).astype(np.uint8)
            
            # 设置消息时间戳和帧ID
            now = self.get_clock().now().to_msg()
            frame_id_left = 'camera_left_frame'
            frame_id_right = 'camera_right_frame'
            
            # 转换为ROS消息并发布
            # 左相机图像
            left_msg = self.bridge.cv2_to_imgmsg(frame_left_rectified, 'bgr8')
            left_msg.header.stamp = now
            left_msg.header.frame_id = frame_id_left
            self.left_pub.publish(left_msg)
            
            # 右相机图像
            right_msg = self.bridge.cv2_to_imgmsg(img_right_rectified, 'mono8')
            right_msg.header.stamp = now
            right_msg.header.frame_id = frame_id_right
            self.right_pub.publish(right_msg)
            
            # 视差图
            disparity_msg = self.bridge.cv2_to_imgmsg(disp_normalized, 'mono8')
            disparity_msg.header.stamp = now
            disparity_msg.header.frame_id = frame_id_left
            self.disparity_pub.publish(disparity_msg)
            
            # 深度图
            depth_msg = self.bridge.cv2_to_imgmsg(depth_normalized, 'mono8')
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = frame_id_left
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            left_info_msg = CameraInfo()
            left_info_msg.header.stamp = now
            left_info_msg.header.frame_id = frame_id_left
            left_info_msg.height = frame_left_rectified.shape[0]
            left_info_msg.width = frame_left_rectified.shape[1]
            # 填充相机矩阵等参数
            # K = [fx 0 cx; 0 fy cy; 0 0 1]
            left_info_msg.k = list(self.stereo_camera.left_camera_matrix.flatten())
            # 发布相机信息
            self.left_info_pub.publish(left_info_msg)
            
            # 同样处理右相机信息
            right_info_msg = CameraInfo()
            right_info_msg.header.stamp = now
            right_info_msg.header.frame_id = frame_id_right
            right_info_msg.height = img_right_rectified.shape[0]
            right_info_msg.width = img_right_rectified.shape[1]
            right_info_msg.k = list(self.stereo_camera.right_camera_matrix.flatten())
            self.right_info_pub.publish(right_info_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')
    
    def destroy_node(self):
        """节点销毁时关闭相机"""
        if hasattr(self, 'stereo_camera'):
            self.stereo_camera.close_camera()
            self.get_logger().info('相机已关闭')
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()