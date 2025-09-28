# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "y1_msg: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iy1_msg:/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(y1_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" NAME_WE)
add_custom_target(_y1_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "y1_msg" "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" NAME_WE)
add_custom_target(_y1_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "y1_msg" "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" NAME_WE)
add_custom_target(_y1_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "y1_msg" "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" NAME_WE)
add_custom_target(_y1_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "y1_msg" "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" "std_msgs/Header:std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg
)
_generate_msg_cpp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg
)
_generate_msg_cpp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg
)
_generate_msg_cpp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(y1_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(y1_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(y1_msg_generate_messages y1_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_cpp _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_cpp _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_cpp _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_cpp _y1_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(y1_msg_gencpp)
add_dependencies(y1_msg_gencpp y1_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS y1_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg
)
_generate_msg_eus(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg
)
_generate_msg_eus(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg
)
_generate_msg_eus(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(y1_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(y1_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(y1_msg_generate_messages y1_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_eus _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_eus _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_eus _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_eus _y1_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(y1_msg_geneus)
add_dependencies(y1_msg_geneus y1_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS y1_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg
)
_generate_msg_lisp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg
)
_generate_msg_lisp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg
)
_generate_msg_lisp(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(y1_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(y1_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(y1_msg_generate_messages y1_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_lisp _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_lisp _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_lisp _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_lisp _y1_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(y1_msg_genlisp)
add_dependencies(y1_msg_genlisp y1_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS y1_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg
)
_generate_msg_nodejs(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg
)
_generate_msg_nodejs(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg
)
_generate_msg_nodejs(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(y1_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(y1_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(y1_msg_generate_messages y1_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_nodejs _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_nodejs _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_nodejs _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_nodejs _y1_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(y1_msg_gennodejs)
add_dependencies(y1_msg_gennodejs y1_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS y1_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg
)
_generate_msg_py(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg
)
_generate_msg_py(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg
)
_generate_msg_py(y1_msg
  "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg
)

### Generating Services

### Generating Module File
_generate_module_py(y1_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(y1_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(y1_msg_generate_messages y1_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmEndPoseControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_py _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointPositionControl.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_py _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmJointState.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_py _y1_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/IMETA_LAB/y1_sdk_python/y1_ros/src/y1_msg/msg/ArmStatus.msg" NAME_WE)
add_dependencies(y1_msg_generate_messages_py _y1_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(y1_msg_genpy)
add_dependencies(y1_msg_genpy y1_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS y1_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/y1_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(y1_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/y1_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(y1_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/y1_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(y1_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/y1_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(y1_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/y1_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(y1_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
