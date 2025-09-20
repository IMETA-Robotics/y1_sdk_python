#!/usr/bin/env python3

import rospy
import json
from y1_msg.msg import ArmEndPoseControl

def playback_trajectory(jsonl_file, control_pub):
    with open(jsonl_file, 'r') as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        rospy.logerr("No data found in the file.")
        return

    data_len  = len(data_lines)
    print("trajectory size : ", data_len)
    rospy.sleep(3) # TODO: 为什么要等待一会，第一个点才可以发送成功？
    input("Press key [Enter] to start play trajectory.")

    idx = 1  # 第一个点已经发过了
    msg = ArmEndPoseControl()
    msg.header.stamp = rospy.Time.now()
    msg.end_pose = data_lines[0]["end_pose"]
    msg.joint_velocity = 5
    control_pub.publish(msg)
    rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
    rospy.sleep(3)  # 给机械臂3秒时间移动到位
    
    rate = rospy.Rate(25)
    while not rospy.is_shutdown() and idx < data_len:
        msg.header.stamp = rospy.Time.now()
        msg.end_pose = data_lines[idx]['end_pose']
        # msg.gripper_stroke = data_lines[idx]['position'][6]

        control_pub.publish(msg)
        idx += 1
        print("index: ", idx)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('play_trajectory', anonymous=True)

    jsonl_file = "/home/zxf/IMETA_LAB/y1_sdk_python/y1_ros/data/arm_state_25hz.jsonl"
    
    pub = rospy.Publisher('/y1/arm_end_pose_control', ArmEndPoseControl, queue_size=1)

    rospy.loginfo(f"Preparing to play trajectory from {jsonl_file}...")
    playback_trajectory(jsonl_file, pub)