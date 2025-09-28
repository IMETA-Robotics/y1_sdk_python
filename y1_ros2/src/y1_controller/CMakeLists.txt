cmake_minimum_required(VERSION 3.0.2)
project(y1_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  y1_msg
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)