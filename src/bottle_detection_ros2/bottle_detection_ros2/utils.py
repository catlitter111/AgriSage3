#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工具函数模块
提供图像处理、数据转换等通用功能
"""

import cv2
import numpy as np
import time

def calculate_fps(frame_count, start_time):
    """
    计算帧率
    
    参数:
    frame_count: 帧数
    start_time: 开始时间
    
    返回:
    fps值
    """
    elapsed_time = time.time() - start_time
    if elapsed_time > 0:
        return frame_count / elapsed_time
    return 0.0

def draw_fps(image, fps, position=(10, 30)):
    """
    在图像上绘制FPS
    
    参数:
    image: 输入图像
    fps: 帧率值
    position: 文字位置
    """
    text = f"FPS: {fps:.1f}"
    cv2.putText(image, text, position, 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

def draw_crosshair(image, center_x, center_y, size=20, color=(0, 255, 0), thickness=2):
    """
    绘制十字准星
    
    参数:
    image: 输入图像
    center_x, center_y: 中心坐标
    size: 准星大小
    color: 颜色
    thickness: 线条粗细
    """
    # 水平线
    cv2.line(image, 
             (center_x - size, center_y), 
             (center_x + size, center_y), 
             color, thickness)
    
    # 垂直线
    cv2.line(image, 
             (center_x, center_y - size), 
             (center_x, center_y + size), 
             color, thickness)

def draw_distance_bar(image, distance, max_distance=5.0, 
                     position=(50, 100), width=200, height=20):
    """
    绘制距离条
    
    参数:
    image: 输入图像
    distance: 当前距离
    max_distance: 最大距离
    position: 条形图位置
    width: 条形图宽度
    height: 条形图高度
    """
    x, y = position
    
    # 绘制背景框
    cv2.rectangle(image, (x, y), (x + width, y + height), (255, 255, 255), 2)
    
    if distance is not None and distance > 0:
        # 计算填充长度
        fill_width = int((distance / max_distance) * width)
        fill_width = min(fill_width, width)
        
        # 选择颜色（近距离绿色，远距离红色）
        if distance < 1.0:
            color = (0, 255, 0)  # 绿色
        elif distance < 3.0:
            color = (0, 255, 255)  # 黄色
        else:
            color = (0, 0, 255)  # 红色
        
        # 绘制填充
        cv2.rectangle(image, (x, y), (x + fill_width, y + height), color, -1)
        
        # 绘制距离文本
        text = f"{distance:.2f}m"
        cv2.putText(image, text, (x + width + 10, y + height - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def apply_colormap(disparity_image):
    """
    为视差图应用颜色映射
    
    参数:
    disparity_image: 视差图（灰度）
    
    返回:
    彩色视差图
    """
    return cv2.applyColorMap(disparity_image, cv2.COLORMAP_JET)

def create_side_by_side_image(left_image, right_image):
    """
    创建左右并排图像
    
    参数:
    left_image: 左图
    right_image: 右图
    
    返回:
    并排图像
    """
    # 确保两张图像高度相同
    h1, w1 = left_image.shape[:2]
    h2, w2 = right_image.shape[:2]
    
    if h1 != h2:
        # 调整高度
        if h1 > h2:
            right_image = cv2.resize(right_image, (w2, h1))
        else:
            left_image = cv2.resize(left_image, (w1, h2))
    
    # 水平拼接
    return np.hstack([left_image, right_image])

def normalize_image(image, min_val=None, max_val=None):
    """
    归一化图像到0-255范围
    
    参数:
    image: 输入图像
    min_val: 最小值（可选）
    max_val: 最大值（可选）
    
    返回:
    归一化后的图像
    """
    if min_val is None:
        min_val = np.min(image)
    if max_val is None:
        max_val = np.max(image)
    
    if max_val > min_val:
        normalized = (image - min_val) / (max_val - min_val) * 255
        return normalized.astype(np.uint8)
    else:
        return np.zeros_like(image, dtype=np.uint8)

def calculate_center_offset(image_width, object_x):
    """
    计算物体相对于图像中心的偏移
    
    参数:
    image_width: 图像宽度
    object_x: 物体x坐标
    
    返回:
    偏移量（正值表示物体在右侧，负值表示在左侧）
    """
    center_x = image_width // 2
    return object_x - center_x

def is_in_center_region(image_width, image_height, object_x, object_y, 
                       tolerance_x=0.1, tolerance_y=0.1):
    """
    判断物体是否在图像中心区域
    
    参数:
    image_width, image_height: 图像尺寸
    object_x, object_y: 物体坐标
    tolerance_x, tolerance_y: 容差（相对于图像尺寸的比例）
    
    返回:
    True如果在中心区域，否则False
    """
    center_x = image_width // 2
    center_y = image_height // 2
    
    tol_x = image_width * tolerance_x
    tol_y = image_height * tolerance_y
    
    return (abs(object_x - center_x) < tol_x and 
            abs(object_y - center_y) < tol_y)

def smooth_value(current_value, previous_value, alpha=0.7):
    """
    使用指数移动平均平滑数值
    
    参数:
    current_value: 当前值
    previous_value: 之前的值
    alpha: 平滑系数（0-1，越大越倾向于当前值）
    
    返回:
    平滑后的值
    """
    if previous_value is None:
        return current_value
    return alpha * current_value + (1 - alpha) * previous_value

class MovingAverage:
    """移动平均滤波器类"""
    
    def __init__(self, window_size=5):
        """
        初始化移动平均滤波器
        
        参数:
        window_size: 窗口大小
        """
        self.window_size = window_size
        self.values = []
    
    def update(self, value):
        """
        更新并返回移动平均值
        
        参数:
        value: 新值
        
        返回:
        移动平均值
        """
        self.values.append(value)
        
        # 保持窗口大小
        if len(self.values) > self.window_size:
            self.values.pop(0)
        
        # 计算平均值
        if self.values:
            return sum(self.values) / len(self.values)
        return value
    
    def reset(self):
        """重置滤波器"""
        self.values = []

class MedianFilter:
    """中值滤波器类"""
    
    def __init__(self, window_size=5):
        """
        初始化中值滤波器
        
        参数:
        window_size: 窗口大小
        """
        self.window_size = window_size
        self.values = []
    
    def update(self, value):
        """
        更新并返回中值
        
        参数:
        value: 新值
        
        返回:
        中值
        """
        self.values.append(value)
        
        # 保持窗口大小
        if len(self.values) > self.window_size:
            self.values.pop(0)
        
        # 计算中值
        if self.values:
            sorted_values = sorted(self.values)
            return sorted_values[len(sorted_values) // 2]
        return value
    
    def reset(self):
        """重置滤波器"""
        self.values = []