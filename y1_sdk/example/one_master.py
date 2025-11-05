# -*- coding: utf-8 -*-
"""
  ControlMode.GRAVITY_COMPENSATION
  将机械臂设置为 重力补偿模式, 可拖动示教 或 采集数据.

example:
    python3 one_master.py
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
    master_arm = Y1SDKInterface(
        can_id=can_id,
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )
    
    # 初始化 Y1 SDK
    if not master_arm.Init():
        print("Init Y1 SDK Interface failed")
        raise RuntimeError("Y1 SDK Init failed")

    # 设置控制模式为: 重力补偿(一般主臂摇操需要)
    master_arm.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
    
    # 注意: 因为机械臂本身没有控制器,SDK是在你的当前pc上运行的,所以当前程序如果结束,那么重力补偿就失效了
    # 获取关节数据
    while True:
        # 末端位姿
        arm_end_pose = master_arm.GetArmEndPose()
        # 关节位置
        joint_position = master_arm.GetJointPosition()
        # 关节速度
        joint_velocity = master_arm.GetJointVelocity()
        # 关节力矩
        joint_effort = master_arm.GetJointEffort()
        
        print("arm end pose: ", arm_end_pose)
        print("arm joint position: ", joint_position)
        print("arm joint velocity: ", joint_velocity)
        print("arm joint effort: ", joint_effort)
        # 等待10ms
        time.sleep(0.01)
        