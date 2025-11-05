# -*- coding: utf-8 -*-
"""
  ControlMode.NRT_JOINT_POSITION
  将机械臂设置为 非实时控制模式, 支持位置控制和末端位姿控制, 模型推理使用该模式!!!
  该模式接收高频实时控制(最大400HZ), 内部没有做轨迹插值优化, 所以除了作为从臂跟踪,
  

example:
    python3 single_arm_control.py
"""

from y1_sdk import Y1SDKInterface, ControlMode
import os
import time

# 获取当前脚本文件所在目录
HERE = os.path.dirname(os.path.abspath(__file__))

can_id = "can0"
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

    # 设置控制模式为: 重力补偿(一般主臂摇操需要)
    single_control_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
    
    # 注意: 因为机械臂本身没有控制器,SDK是在你的当前pc上运行的,所以当前程序如果结束,那么就不会再反馈关节信息和接受指令了
    
    # 关节位置控制
    joint_position_control_flag = False
    if joint_position_control_flag:
      time.sleep(3)
      joint_position_control = [0.6, -0.6, 0.6, 0.5, 0.4, 0]
      joint_velocity_control = 10  # 关节执行速度幅度(1-10), 1为最慢, 10为最快, 可以不设置, 默认参数为5
      single_control_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)  # control J1 - J6 joint
      
      gripper_stroke = 10  # 夹爪行程(0-80mm)
      gripper_velocity = 3 # 夹爪执行速度幅度(1-10), 1为最慢, 10为最快, 可以不设置, 默认参数为5
      single_control_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # control gripper
    
    # 末端位姿控制
    end_pose_control_flag = True
    if end_pose_control_flag:
      time.sleep(3)
      arm_end_pose_control = [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
      single_control_arm.SetArmEndPose(arm_end_pose_control)  # end pose control arm
      
      gripper_stroke = 10  # 夹爪行程(0-80mm)
      gripper_velocity = 3 # 夹爪执行速度幅度(1-10), 1为最慢, 10为最快, 可以不设置, 默认参数为5
      single_control_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # control gripper
    
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
        
        print("arm end pose: ", arm_end_pose)
        print("arm joint position: ", joint_position)
        print("arm joint velocity: ", joint_velocity)
        print("arm joint effort: ", joint_effort)
        # 等待10ms
        time.sleep(0.01)
    
        