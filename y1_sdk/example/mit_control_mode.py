# -*- coding: utf-8 -*-
"""
  ControlMode.MIT_CONTROL
  将机械臂设置为 MIT控制模式, 支持位置控制和力矩控制!

example:
    python3 mit_control_mode.py
"""

from y1_sdk import Y1SDKInterface, ControlMode, MitControlCommand
import os
import time

# 获取当前脚本文件所在目录
HERE = os.path.dirname(os.path.abspath(__file__))

can_id = "can1"
# 使能 or 失能
auto_enable = True
# 0: nothing, 1: gripper, 2: teaching pendant, 3: gripper and teaching pendant
arm_end_type = 3

if arm_end_type == 0:
    urdf_path = os.path.join(HERE, "urdf", "y1_no_gripper.urdf")
elif arm_end_type == 1:
  urdf_path = os.path.join(HERE, "urdf", "y1_with_gripper.urdf")
elif arm_end_type == 2:
  urdf_path = os.path.join(HERE, "urdf", "y1_with_gripper.urdf")
elif arm_end_type == 3:
  urdf_path = os.path.join(HERE, "urdf", "y1_with_gripper.urdf")
else:
    print(f"arm_end_type {arm_end_type} not supported")
    raise RuntimeError("Unsupported arm_end_type") 

if __name__ == "__main__":
    # 初始化 Y1 SDK
    single_control_arm = Y1SDKInterface(
        can_id=can_id,
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )
    
    # 初始化 Y1 SDK
    if not single_control_arm.Init():
        print("Init Y1 SDK Interface failed")
        raise RuntimeError("Y1 SDK Init failed")

    # 设置控制模式为: MIT控制模式
    single_control_arm.SetArmControlMode(ControlMode.MIT_CONTROL)
    
    def make_mit_command(kp=0.0, joint_position=0.0, kd=0.0, joint_velocity=0.0, torque=0.0):
        command = MitControlCommand()
        command.kp = kp
        command.joint_position = joint_position
        command.kd = kd
        command.joint_velocity = joint_velocity
        command.torque = torque
        return command

    # 力矩控制: 只控制 J1 和夹爪, J2-J6 下发 0 力矩
    torque_control_flag = True
    if torque_control_flag:
      time.sleep(3)

      j1_torque = 0.5  # J1 目标力矩, 单位: Nm. 请从小力矩开始测试
      gripper_torque = 0.1  # 夹爪目标力矩, 正值张开, 负值闭合. 请从小力矩开始测试
      has_gripper = arm_end_type in (1, 3)
      control_dt = 0.01  # MIT 控制命令下发周期, 0.01s = 100Hz
      control_time = 5.0  # 控制时长, 单位: s
      start_time = time.time()

      while time.time() - start_time < control_time:
        mit_command = [
            make_mit_command(torque=j1_torque),  # J1 torque control
            make_mit_command(),                  # J2 zero torque
            make_mit_command(),                  # J3 zero torque
            make_mit_command(),                  # J4 zero torque
            make_mit_command(),                  # J5 zero torque
            make_mit_command(),                  # J6 zero torque
        ]
        single_control_arm.MitControlArm(mit_command)

        if has_gripper:
            single_control_arm.MitControlGripper(make_mit_command(torque=gripper_torque))
        time.sleep(control_dt)

        

      # 示例结束后下发一次 0 力矩, 避免继续保持上一帧力矩命令
      single_control_arm.MitControlArm([make_mit_command() for _ in range(6)])
      if has_gripper:
          single_control_arm.MitControlGripper(make_mit_command())
    
    
    # 获取关节数据
    while True:
        # 末端位姿
        arm_end_pose = single_control_arm.GetArmEndPose()
        # 关节位置
        joint_position = single_control_arm.GetJointPosition()
        # 关节速度
        joint_velocity = single_control_arm.GetJointVelocity()
        # 关节力矩
        joint_effort = single_control_arm.GetJointEffort()
        
        # print("arm end pose: ", arm_end_pose)
        # print("arm joint position: ", joint_position)
        # print("arm joint velocity: ", joint_velocity)
        # print("arm joint effort: ", joint_effort)
        # 等待100ms
        time.sleep(0.1)
