cmake_minimum_required(VERSION 3.8)
project(bottle_detection_ros2)

# 默认为C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# 默认为C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BottleDetection.msg"
  "msg/RobotCommand.msg"
  "msg/RobotStatus.msg"
  "msg/HarvestCommand.msg"
  "msg/ServoCommand.msg"
  "msg/ServoStatus.msg"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

# Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  bottle_detection_ros2/bottle_detection_node.py
  bottle_detection_ros2/bottle_detection_node_async.py
  bottle_detection_ros2/integrated_bottle_detection_node.py
  bottle_detection_ros2/websocket_bridge_node.py
  bottle_detection_ros2/robot_control_node.py
  bottle_detection_ros2/servo_control_node.py
  bottle_detection_ros2/auto_harvest_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装模型文件（如果有）
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# 安装数据文件
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()