cmake_minimum_required(VERSION 3.5)
project(y1_mujoco)

# 设置 CMake 的编译选项
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 创建 Python 脚本安装规则
install(PROGRAMS
  scripts/y1_with_rubber.py
  DESTINATION lib/${PROJECT_NAME}
)

# 包的描述
ament_package()