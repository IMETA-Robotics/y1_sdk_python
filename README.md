# Software Dependency

  - Ubuntu 20.04 LTS
  - ROS Noetic

  ```sh
  sudo apt update
  sudo apt install can-utils net-tools iproute2 
  sudo apt install ros-noetic-kdl-parser liborocos-kdl-dev ros-noetic-urdf ros-noetic-trac-ik
  ```

## Set single arm Can
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

## 多臂设置参考一下飞书文档
Refer [https://nxjux7a2aq.feishu.cn/wiki/MPQ4wPDiXi7fiYkM1NmcJssYnRc]

## Install y1_sdk

  ```sh
  cd y1_sdk/
  pip install .
  ```

## Use y1_ros

### build
  ```sh
  cd y1_ros/
  bash build.sh
  ```

### 单臂使用（末端：夹爪示教器2合1）
  Note: launch中默认参数can接口为can0， arm_end_type 为3（可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```

  单臂重力补偿模式(采集数据时可用):
  ```sh
  roslaunch y1_controller one_master.launch
  ```

  单臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller single_arm_control.launch
  ```

### 双臂使用 （末端：夹爪示教器2合1）
  Note: launch中默认参数can接口为can0、can2， arm_end_type 为3（可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```
  双臂重力补偿模式(采集数据时可用):
  ```sh
  roslaunch y1_controller two_master.launch
  ```
  双臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller two_arm_control.launch
  ```

### 一主一从使用（两条臂，一个末端是示教器， 一个末端是夹爪）
  Note: launch中默认参数can接口为can0、can1， 
  主臂arm_end_type为2, 从臂arm_end_type为1 （可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```
  主从摇操模式(采集数据时可用):
  ```sh
  roslaunch y1_controller one_master_slave.launch
  ```
  单个从臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller single_arm_control.launch
  ```

### 两主两从使用 （四条臂，两个末端是示教器， 两个末端是夹爪）
  Note: launch中默认参数, 右主can0, 右从can1, 左主can2, 左从can3， 
  两个主臂arm_end_type为2, 两个从臂arm_end_type为1 （可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```
  主从摇操模式(采集数据时可用):
  ```sh
  roslaunch y1_controller two_master_slave.launch
  ```
  双从臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller two_arm_control.launch
  ```

## 以下为单臂rostopic控制示例（双臂是分为两个话题，分别下发控制指令）
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

## 话题数据类型说明 （详细变量含义请查看msg文件内注释）

| msg                     | Description                       |
| ----------------------- | --------------------------------- |
| ArmJointState           | 机械臂关节信息反馈， 如速度、位置、力矩。|  
| ArmStatus               | 机械臂关节状态，如温度、电流           |
| ArmJointPositionControl | 关节位置控制                        |
| ArmEndPoseControl       | 末端位姿控制                        |