# -*- coding: utf-8 -*-
"""
  ControlMode.RT_JOINT_POSITION
  将机械臂设置为 从臂跟踪模式, 可跟随主臂摇操, 该模式运行速度特别快!!!!
  该模式接收高频实时控制(最大400HZ), 内部没有做轨迹插值优化, 所以除了作为从臂跟踪, 或用户在上层算法做了轨迹规划, 不然不建议使用!!!!

example:
    python3 one_slave.py
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
    slave_arm = Y1SDKInterface(
        can_id=can_id,
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )
    
    # 初始化 Y1 SDK
    if not slave_arm.Init():
        print("Init Y1 SDK Interface failed")
        raise RuntimeError("Y1 SDK Init failed")

    # 设置控制模式为: 实时关节位置控制(一般作为从臂跟随主臂摇操)
    slave_arm.SetArmControlMode(ControlMode.RT_JOINT_POSITION)
    
    # 注意: 因为机械臂本身没有控制器,SDK是在你的当前pc上运行的,所以当前程序如果结束,那么就不会再反馈关节信息和接受指令了
    while True:
        # 获取关节数据
        # 末端位姿
        arm_end_pose = slave_arm.GetArmEndPose()
        # 关节位置
        joint_position = slave_arm.GetJointPosition()
        # 关节速度
        joint_velocity = slave_arm.GetJointVelocity()
        # 关节力矩
        joint_effort = slave_arm.GetJointEffort()
        
        print("arm end pose: ", arm_end_pose)
        print("arm joint position: ", joint_position)
        print("arm joint velocity: ", joint_velocity)
        print("arm joint effort: ", joint_effort)
        
        # 该模式运行速度特别快!!!! 建议只在主从摇操模式下, 接受主臂的关节位置来使用!!!!不要打开注释使用!!!!
        # 发送关节位置控制
        # joint_position_control = [0, 0, 0, 0, 0, 0, 0]  # 前6位为J1 - J6的关节位置控制, 最后一位为夹爪量程(0-80mm)
        # slave_arm.SetFollowerArmJointPosition(joint_position_control)
        
        # 发送末端位姿控制
        # arm_end_pose_control = [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
        # slave_arm.SetArmEndPose(arm_end_pose_control)
        # slave_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # 设置夹爪量程和执行速度
        
        # 等待10ms
        time.sleep(0.01)
        
