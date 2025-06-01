#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测启动文件 - 一次性启动所有节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """生成启动描述"""
    # 获取包路径
    pkg_dir = get_package_share_directory('bottle_detection_robot')
    
    # 相机参数文件路径
    camera_params_file = os.path.join(pkg_dir, 'config', 'out.xls')
    
    # RKNN模型路径
    model_path = os.path.join(pkg_dir, 'models', 'yolo11n.rknn')
    
    # 创建相机节点
    camera_node = Node(
        package='bottle_detection_robot',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'camera_id': 1,               # 相机ID
            'frame_width': 1280,          # 画面宽度
            'frame_height': 480,          # 画面高度
            'fps': 30,                    # 帧率
            'camera_params_file': camera_params_file  # 相机参数文件路径
        }],
        output='screen'
    )
    
    # 创建瓶子检测节点
    bottle_detector_node = Node(
        package='bottle_detection_robot',
        executable='bottle_detector_node',
        name='bottle_detector_node',
        parameters=[{
            'model_path': model_path,     # RKNN模型路径
            'model_size': [640, 640],     # 模型输入尺寸
            'detection_threshold': 0.2,   # 检测阈值
            'detection_interval': 0.1     # 检测间隔（秒）
        }],
        output='screen'
    )
    
    # 创建RViz节点，用于可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'bottle_detection.rviz')],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        camera_node,
        bottle_detector_node,
        rviz_node
    ])