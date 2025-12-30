#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json, os
from y1_msg.msg import ArmJointState

class RecordTrajectory(Node):
    def __init__(self):
        super().__init__('record_trajectory')
        
        self.joint_state = None
        self.subscription = self.create_subscription(
            ArmJointState,
            '/master_arm_right/joint_states',
            self.state_callback,
            1
        )
        
        os.makedirs("data", exist_ok=True)
        
        input("Press key [Enter] to start record trajectory.")

        self.count = 1
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)
        self.file = open("data/arm_state_30hz.jsonl", 'a')

    def state_callback(self, msg):
        self.joint_state = msg

    def timer_callback(self):
        if self.joint_state is not None:
            data = {
                'position': list(self.joint_state.joint_position),
                'velocity': list(self.joint_state.joint_velocity),
                'effort': list(self.joint_state.joint_effort),
                "end_pose": list(self.joint_state.end_pose),
            }
            self.file.write(json.dumps(data) + '\n')
            self.file.flush()  # Ensure data is written immediately
            print(f"record {self.count}th pose, position: {data['position']} , end_pose: {data['end_pose']}")
            self.count += 1

    def destroy_node(self):
        if hasattr(self, 'file'):
            self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    
    node = RecordTrajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()