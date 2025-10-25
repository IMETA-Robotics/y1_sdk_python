cmake_minimum_required(VERSION 3.0.2)
project(y1_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

# 包含头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/joint_states_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)