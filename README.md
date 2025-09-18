# Software Dependency

  - Ubuntu 20.04 LTS
  - ROS Noetic

  ```sh
  sudo apt update
  sudo apt install can-utils net-tools iproute2 
  sudo apt install ros-noetic-kdl-parser liborocos-kdl-dev ros-noetic-urdf ros-noetic-trac-ik
  ```

## Install y1_sdk

  ```sh
  cd y1_sdk/
  pip install .
  ```

## Use y1_ros

### config can device(同一台电脑环境没变化，只需配置一次)
  如果只有一个臂:
  ```sh
  cd y1_ros/can_scripts/
  bash set_only_one_can.sh
  ```
### 启动can（每次电脑重启 或 拔插can线时）
  ```sh
  bash can_scripts/start_can0.sh
  ```

### build
  ```sh
  cd y1_ros/
  bash build.sh
  ```

### launch
  ```sh
  source devel/setup.bash
  ```

  单臂重力补偿模式(采集数据时可用):
  ```sh
  roslaunch y1_controller one_master.launch
  ```

  单臂控制模式(采集数据时可用):
  ```sh
  roslaunch y1_controller single_arm_control.launch
  ```

### joint position control

  ```sh
  source devel/setup.bash
  bash control_scripts/joint_position_control.sh
  ```

### go zero position

  ```sh
  source devel/setup.bash
  bash control_scripts/go_zero_position.sh 
  ```