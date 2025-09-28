cmake_minimum_required(VERSION 3.8)
project(y1_controller)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(y1_msg REQUIRED)

# 安装 Python 包
ament_python_install_package(${PROJECT_NAME})

# 安装 Python 可执行脚本
install(PROGRAMS
  scripts/y1_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件（如果有）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安装配置文件（如果有）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安装 URDF 文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY
    urdf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安装资源文件
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclpy
  std_msgs
  y1_msg
)

# 最后调用 ament_package()
ament_package()