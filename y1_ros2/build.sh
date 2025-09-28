#!/bin/bash

thread_num=$(($(nproc) - 1))

# y1 arm gazebo
# catkin_make install --pkg y1_gazebo -j${thread_num}

# y1 arm ros msgs
catkin_make install --only-pkg-with-deps y1_msg -j${thread_num}

# y1 description
catkin_make install --only-pkg-with-deps y1_description -j${thread_num}

# y1 arm ros1 driver
catkin_make install --only-pkg-with-deps y1_controller -j${thread_num}