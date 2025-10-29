# -*- coding: utf-8 -*-
"""
  双机械臂控制demo.
  ControlMode.NRT_JOINT_POSITION
  将机械臂设置为 非实时控制模式, 支持位置控制和末端位姿控制, 模型推理使用该模式!!!

example:
    python3 two_arm_control.py
"""

from y1_sdk import Y1SDKInterface, ControlMode
import os
import time

# 获取当前脚本文件所在目录
HERE = os.path.dirname(os.path.abspath(__file__))

# 使能 or 失能
auto_enable = True
# 0: nothing, 1: gripper, 2: teaching pendant, 3: gripper and teaching pendant
arm_end_type = 0

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
    right_arm = Y1SDKInterface(
        can_id="can0",
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )

    left_arm = Y1SDKInterface(
        can_id="can1",
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )
    
    # 初始化 Y1 SDK
    if not right_arm.Init():
        print("Init right arm SDK Interface failed")
        raise RuntimeError("Right arm SDK Init failed")

    if not left_arm.Init():
        print("Init left arm SDK Interface failed")
        raise RuntimeError("Left arm SDK Init failed")

    # 设置控制模式为: 非实时控制模式(支持关节位置、end pose控制)
    right_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
    left_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
    
    # 注意: 因为机械臂本身没有控制器,SDK是在你的当前pc上运行的,所以当前程序如果结束,那么就不会再反馈关节信息和接受指令了
    
    # 关节位置控制
    time.sleep(3)
    joint_position_control_flag = False
    if joint_position_control_flag:
      # control right arm
      joint_position_control = [0.6, -0.6, 0.6, 0.5, 0.4, 0]
      joint_velocity_control = 3  # 关节执行速度幅度(1-10), 1为最慢, 10为最快, 可以不设置, 默认参数为5
      right_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)  # control J1 - J6 joint

      gripper_stroke = 10  # 夹爪行程(0-80mm)
      gripper_velocity = 3 # 夹爪执行速度幅度(1-10), 1为最慢, 10为最快, 可以不设置, 默认参数为5
      right_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # control gripper

      # control left arm
      left_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)
      left_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # control gripper
    
    # 末端位姿控制
    time.sleep(3)
    end_pose_control_flag = False
    if end_pose_control_flag:
      arm_end_pose_control = [0.0535, -0.0476, 0.3963, 0.1990, -0.4414, -1.0352]
      right_arm.SetArmEndPose(arm_end_pose_control)  # end pose control arm
      
      gripper_stroke = 10  # 夹爪行程(0-80mm)
      gripper_velocity = 3 # 夹爪执行速度幅度(1-10), 1为最慢, 10为最快, 可以不设置, 默认参数为5
      right_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # control gripper
    
    # 获取关节数据
    while True:
        # right_arm
        # 末端位姿
        right_arm_end_pose = right_arm.GetArmEndPose()
        # 关节位置
        right_arm_joint_position = right_arm.GetJointPosition()
        # 关节速度
        right_arm_joint_velocity = right_arm.GetJointVelocity()
        # 关节力矩
        right_arm_joint_effort = right_arm.GetJointEffort()
        
        print("right arm end pose: ", right_arm_end_pose)
        print("right arm joint position: ", right_arm_joint_position)
        print("right arm joint velocity: ", right_arm_joint_velocity)
        print("right arm joint effort: ", right_arm_joint_effort)

        # left_arm
        # 末端位姿
        left_arm_end_pose = left_arm.GetArmEndPose()
        # 关节位置
        left_arm_joint_position = left_arm.GetJointPosition()
        # 关节速度
        left_arm_joint_velocity = left_arm.GetJointVelocity()
        # 关节力矩
        left_arm_joint_effort = left_arm.GetJointEffort()
        
        print("left arm end pose: ", left_arm_end_pose)
        print("left arm joint position: ", left_arm_joint_position)
        print("left arm joint velocity: ", left_arm_joint_velocity)
        print("left arm joint effort: ", left_arm_joint_effort)

        # 等待10ms
        time.sleep(0.01) 