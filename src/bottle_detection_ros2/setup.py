#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup
import os
from glob import glob

package_name = 'bottle_detection_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # 安装package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        
        # 安装消息文件
        (os.path.join('share', package_name, 'msg'), 
            glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 nodes for bottle detection and harvesting system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bottle_detection_node = bottle_detection_ros2.bottle_detection_node:main',
            'bottle_detection_node_async = bottle_detection_ros2.bottle_detection_node_async:main',
            'integrated_bottle_detection_node = bottle_detection_ros2.integrated_bottle_detection_node:main',
            'websocket_bridge_node = bottle_detection_ros2.websocket_bridge_node:main',
            'robot_control_node = bottle_detection_ros2.robot_control_node:main',
            'servo_control_node = bottle_detection_ros2.servo_control_node:main',
            'auto_harvest_controller = bottle_detection_ros2.auto_harvest_controller:main',
        ],
    },
)