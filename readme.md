# 瓶子检测与自动采摘机器人系统

## 项目概述

这是一个基于ROS2的智能农业机器人系统，专门用于瓶子检测和自动采摘任务。系统集成了计算机视觉、深度感知、机器人控制和远程通信等多项技术，能够自主识别并采摘目标物体。

### 主要特性

- 🎯 **高精度目标检测**：使用YOLO11n深度学习模型进行瓶子检测
- 👁️ **3D深度感知**：双目相机实时计算目标距离（0.2-5米范围）
- 🤖 **智能采摘**：6自由度机械臂精准抓取
- 📡 **远程控制**：WebSocket实时通信，支持远程监控和控制
- 🚀 **高性能处理**：多线程异步架构，30+ FPS实时检测
- 🔄 **双模式操作**：支持手动遥控和自动巡航采摘

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                     远程控制中心                          │
│                  (WebSocket Server)                      │
└────────────────────┬───────────────────────────────────┘
                     │ WebSocket
┌────────────────────┴───────────────────────────────────┐
│                  ROS2 系统架构                          │
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐ │
│  │瓶子检测节点  │  │ 机器人控制   │  │  舵机控制     │ │
│  │(双目视觉)   │  │ (底盘移动)   │  │ (机械臂)     │ │
│  └──────┬──────┘  └──────┬───────┘  └───────┬───────┘ │
│         │                │                   │         │
│  ┌──────┴────────────────┴───────────────────┴───────┐ │
│  │              自动采摘控制器                        │ │
│  │          (状态机 & 决策逻辑)                      │ │
│  └───────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

## 功能模块详解

### 1. 瓶子检测模块 (`integrated_bottle_detection_node`)

- **双目相机采集**：1280x480分辨率，左右相机同步采集
- **YOLO检测**：基于RKNN优化的YOLO11n模型，专门针对瓶子检测优化
- **深度计算**：立体匹配算法计算3D点云，精确测量目标距离
- **多线程处理**：3个NPU核心并行处理，大幅提升检测速度

### 2. 机器人控制模块 (`robot_control_node`)

- **串口通信**：通过串口与底层电机控制器通信
- **运动控制**：支持前进、后退、左转、右转等基本动作
- **速度调节**：线速度最大0.5m/s，角速度最大1.0rad/s
- **状态反馈**：实时上报位置、电量、CPU使用率等信息

### 3. 舵机控制模块 (`servo_control_node`)

- **6轴机械臂**：精确控制6个舵机实现复杂采摘动作
- **视觉跟踪**：根据检测结果自动调整舵机角度对准目标
- **采摘序列**：预定义5步采摘动作序列，确保稳定抓取
- **位置反馈**：实时读取舵机位置，闭环控制

### 4. WebSocket通信模块 (`websocket_bridge_node`)

- **实时视频流**：JPEG压缩传输，支持动态质量调整
- **双向控制**：接收远程控制命令，上报机器人状态
- **自动重连**：网络断开后自动尝试重连
- **数据压缩**：高效的数据编码，降低带宽占用

### 5. 自动采摘控制器 (`auto_harvest_controller`)

- **目标搜索**：自动旋转搜索视野内的瓶子
- **智能接近**：根据距离自动调整接近策略
  - 远距离(>1.2m)：快速接近
  - 中距离(0.8-1.2m)：精细调整
  - 近距离(0.5-0.8m)：舵机跟踪
  - 采摘距离(0.3-0.5m)：执行抓取
- **避障保护**：距离过近自动后退

## 安装与配置

### 系统要求

- **操作系统**：Ubuntu 20.04/22.04
- **ROS版本**：ROS2 Foxy/Humble
- **硬件要求**：
  - RK3588或类似的NPU支持
  - USB双目相机
  - 串口设备（机器人底盘、舵机控制器）

### 依赖安装

```bash
# 安装ROS2依赖
sudo apt update
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport

# 安装Python依赖
pip3 install opencv-python numpy pandas websocket-client pyserial

# 安装RKNN运行时（根据具体硬件平台）
# 参考: https://github.com/rockchip-linux/rknn-toolkit2
```

### 编译安装

```bash
# 创建工作空间
mkdir -p ~/bottle_ws/src
cd ~/bottle_ws/src

# 克隆代码
git clone <repository_url> bottle_detection_ros2

# 编译
cd ~/bottle_ws
colcon build --packages-select bottle_detection_ros2

# 设置环境
source install/setup.bash
```

### 配置文件

1. **相机标定文件**：将相机标定参数保存到 `data/out.xls`
2. **YOLO模型**：将训练好的RKNN模型放到 `data/yolo11n.rknn`

## 使用方法

### 1. 启动完整系统

```bash
ros2 launch bottle_detection_ros2 integrated_system.launch.py
```

### 2. 单独启动瓶子检测

```bash
ros2 launch bottle_detection_ros2 bottle_detection.launch.py show_display:=true
```

### 3. 启动参数说明

```bash
# 自定义参数启动
ros2 launch bottle_detection_ros2 integrated_system.launch.py \
  camera_id:=1 \
  robot_id:=robot_001 \
  ws_server_url:=ws://your-server:1234/ws/robot/robot_001 \
  show_display:=false
```

### 主要启动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| camera_id | 1 | 相机设备ID |
| model_path | yolo11n.rknn | RKNN模型路径 |
| ws_server_url | ws://101.201.150.96:1234/ws/robot/robot_123 | WebSocket服务器地址 |
| robot_id | robot_123 | 机器人唯一标识 |
| show_display | true | 是否显示图像窗口 |
| min_distance | 0.2 | 最小检测距离(米) |
| max_distance | 5.0 | 最大检测距离(米) |

## ROS2话题接口

### 发布话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| /camera/left/image_raw | sensor_msgs/Image | 左相机原始图像 |
| /camera/right/image_raw | sensor_msgs/Image | 右相机原始图像 |
| /bottle_detection/annotated_image | sensor_msgs/Image | 标注后的检测图像 |
| /bottle_detection/compressed_image | sensor_msgs/CompressedImage | 压缩的检测图像 |
| /bottle_detection/nearest_position | geometry_msgs/PointStamped | 最近瓶子的3D位置 |
| /bottle_detection/nearest_distance | std_msgs/Float32 | 最近瓶子的距离 |
| /bottle_detection/count | std_msgs/Int32 | 检测到的瓶子数量 |
| /bottle_detection/info | std_msgs/String | 详细检测信息(JSON) |
| /robot/status | bottle_detection_msgs/RobotStatus | 机器人状态 |
| /servo/status | bottle_detection_msgs/ServoStatus | 舵机状态 |

### 订阅话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| /cmd_vel | geometry_msgs/Twist | 速度控制命令 |
| /robot/mode | std_msgs/String | 工作模式切换 |
| /robot/harvest_command | bottle_detection_msgs/HarvestCommand | 采摘控制命令 |
| /servo/command | bottle_detection_msgs/ServoCommand | 舵机控制命令 |
| /video/quality_preset | std_msgs/String | 视频质量设置 |

## 工作模式

### 手动模式
- 通过WebSocket接收远程控制命令
- 实时视频传输，操作员远程观察
- 手动控制机器人移动和采摘

### 自动模式
- 自主搜索目标
- 自动接近并对准
- 自动执行采摘序列
- 完成后继续搜索下一个目标

## 故障排除

### 相机无法打开
```bash
# 检查相机设备
v4l2-ctl --list-devices

# 测试相机
v4l2-ctl --device=/dev/video1 --stream-mmap --stream-to=test.raw --stream-count=1
```

### 串口权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 临时授权
sudo chmod 666 /dev/ttyS3 /dev/ttyS9
```

### RKNN模型加载失败
- 确认模型文件路径正确
- 检查RKNN运行时版本兼容性
- 验证NPU驱动正常工作

## 性能优化

### 检测性能
- **单线程**：约10-15 FPS
- **3线程**：约25-30 FPS
- **6线程**：约35-40 FPS（硬件限制）

### 优化建议
1. 调整`thread_num`参数匹配硬件能力
2. 降低`publish_rate`减少CPU占用
3. 使用`low`或`very_low`视频质量降低带宽
4. 关闭`show_display`提升性能

## 开发指南

### 添加新的检测目标
1. 修改`CLASSES`列表添加新类别
2. 重新训练YOLO模型包含新目标
3. 更新检测过滤逻辑

### 自定义采摘动作
1. 修改`ARM_COMMANDS`定义新的动作序列
2. 调整`harvest_state_machine`状态转换
3. 测试新动作的稳定性和成功率

### 扩展通信协议
1. 在`websocket_bridge_node`添加新消息类型
2. 实现对应的处理函数
3. 更新服务器端协议

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

## 贡献指南

欢迎提交Issue和Pull Request。提交代码前请确保：
- 代码符合ROS2编码规范
- 添加必要的注释和文档
- 通过所有测试用例

## 联系方式

- 项目维护者：[维护者名称]
- 邮箱：[email]
- 项目主页：[project-url]