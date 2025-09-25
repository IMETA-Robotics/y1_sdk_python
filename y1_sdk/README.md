# Install
pip install .

# View
pip show y1_sdk

# Uninstall
pip uninstall y1_sdk

# SDK Interface
Refer to [y1_sdk_interface.cpp]

# example 
## ControlMode.GRAVITY_COMPENSATION 重力补偿模式(可作为主臂):
  ```sh
  cd y1_sdk/
  python3 example/one_master.py
  ```

## ControlMode.RT_JOINT_POSITION 实时控制模式(可作为从臂):
  ```sh
  cd y1_sdk/
  python3 example/one_slave.py
  ```

## ControlMode.NRT_JOINT_POSITION 非实时控制模式, 支持位置控制和末端位姿控制, 一般使用该模式!!!
  ```sh
  cd y1_sdk/
  python3 example/single_arm_control.py
  ```