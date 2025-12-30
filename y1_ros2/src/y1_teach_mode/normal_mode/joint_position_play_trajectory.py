#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
from y1_msg.msg import ArmJointPositionControl
import time

class PlayTrajectory(Node):
    def __init__(self):
        super().__init__('play_trajectory')
        
        self.jsonl_file = "/home/imeta/IMETA_LAB/y1_sdk_python/y1_ros2/data/arm_state_30hz.jsonl"
        self.publisher = self.create_publisher(
            ArmJointPositionControl, 
            '/master_arm_right/joint_states', 
            1
        )
        
        self.get_logger().info(f"Preparing to play back trajectory from {self.jsonl_file}...")
        
    def playback_trajectory(self):
        try:
            with open(self.jsonl_file, 'r') as f:
                data_lines = [json.loads(line) for line in f]
        except FileNotFoundError:
            self.get_logger().error(f"File {self.jsonl_file} not found.")
            return
        except Exception as e:
            self.get_logger().error(f"Error reading file: {e}")
            return

        if not data_lines:
            self.get_logger().error("No data found in the file.")
            return

        data_len = len(data_lines)
        self.get_logger().info(f"Trajectory size: {data_len}")
        
        idx = 1  # 第一个点已经发过了
        msg = ArmJointPositionControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_position = data_lines[0]['position'][0:6]
        msg.joint_velocity = 3
        msg.gripper_stroke = data_lines[0]['position'][6]
        msg.gripper_velocity = 5
        
        input("Press key [Enter] to start play trajectory.")
        
        time.sleep(3)  # TODO: 为什么要等待一会，第一个点才可以发送成功？

        self.publisher.publish(msg)
        self.get_logger().info("Publish start position, sleeping for 3 seconds go to start position.")
        time.sleep(3)  # 给机械臂3秒时间移动到位
        self.get_logger().info("Playback started.")
        
        # rate = self.create_rate(30)
        while rclpy.ok() and idx < data_len:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_position = data_lines[idx]['position'][0:6]
            msg.gripper_stroke = data_lines[idx]['position'][6]

            self.publisher.publish(msg)
            idx += 1
            self.get_logger().info(f"Index: {idx}")
            # rate.sleep()
            time.sleep(1.0 / 30)

def main():
    rclpy.init()
    
    node = PlayTrajectory()
    
    try:
        node.playback_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()