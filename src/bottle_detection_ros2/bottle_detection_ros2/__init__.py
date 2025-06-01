#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测ROS2功能包
"""

# 版本信息
__version__ = '1.0.0'
__author__ = 'Your Name'
__email__ = 'your_email@example.com'

# 导出主要类和函数
from .stereo_camera import StereoCamera
from .bottle_detector import BottleDetector
from .utils import (
    calculate_fps,
    draw_fps,
    draw_crosshair,
    draw_distance_bar,
    MovingAverage,
    MedianFilter
)

__all__ = [
    'StereoCamera',
    'BottleDetector',
    'calculate_fps',
    'draw_fps',
    'draw_crosshair', 
    'draw_distance_bar',
    'MovingAverage',
    'MedianFilter'
]