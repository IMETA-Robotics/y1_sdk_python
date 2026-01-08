#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from y1_msg.msg import ArmStatus, ArmJointState, ArmEndPoseControl, ArmJointPositionControl, GripperControl
from std_msgs.msg import String
from y1_sdk import Y1SDKInterface, ControlMode
from sensor_msgs.msg import JointState

import os
from ament_index_python.packages import get_package_share_directory

class Y1Controller(Node):
    def __init__(self):
        super().__init__("y1_controller_node")
        
        # ROS 2 参数声明和获取
        self.declare_parameter("arm_can_id", "can0")
        self.declare_parameter("arm_feedback_rate", 200)
        self.declare_parameter("vr_end_pose_control_topic", "/vr/end_pose_control")
        self.declare_parameter("vr_gripper_control_topic", "/vr/gripper_control")
        self.declare_parameter("arm_end_pose_control_topic", "/y1/arm_end_pose_control")
        self.declare_parameter("arm_joint_position_control_topic", "/y1/arm_joint_position_control_topic")
        self.declare_parameter("arm_joint_state_topic", "/y1/arm_joint_state")
        self.declare_parameter("arm_status_topic", "/y1/arm_status")
        self.declare_parameter("arm_control_type", "follower_arm")
        self.declare_parameter("arm_end_type", 0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter('is_sim', False)

        self.can_id = self.get_parameter("arm_can_id").value
        self.arm_feedback_rate = self.get_parameter("arm_feedback_rate").value
        self.vr_end_pose_control_topic = self.get_parameter("vr_end_pose_control_topic").value
        self.vr_gripper_control_topic = self.get_parameter("vr_gripper_control_topic").value
        self.arm_end_pose_control_topic = self.get_parameter("arm_end_pose_control_topic").value
        self.arm_joint_position_control_topic = self.get_parameter("arm_joint_position_control_topic").value
        self.arm_joint_state_topic = self.get_parameter("arm_joint_state_topic").value
        self.arm_status_topic = self.get_parameter("arm_status_topic").value
        self.arm_control_type = self.get_parameter("arm_control_type").value
        self.arm_end_type = self.get_parameter("arm_end_type").value
        self.auto_enable = self.get_parameter("auto_enable").value
        self.is_sim = self.get_parameter("is_sim").value

        # URDF路径 - ROS2方式
        package_name = "y1_controller"
        package_share_dir = get_package_share_directory(package_name)

        urdf_files = {
            0: "y1_no_gripper.urdf",
            1: "y1_with_gripper.urdf", 
            2: "y1_with_gripper.urdf",
            3: "y1_with_gripper.urdf"
        }

        if self.arm_end_type not in urdf_files:
            self.get_logger().error(f"arm_end_type {self.arm_end_type} not supported")
            raise RuntimeError("Unsupported arm_end_type")

        urdf_filename = urdf_files[self.arm_end_type]
        urdf_path = os.path.join(package_share_dir, "urdf", urdf_filename)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found: {urdf_path}")
            raise RuntimeError(f"URDF file not found: {urdf_path}")

        # 初始化 Y1 SDK
        self.y1_interface = Y1SDKInterface(
            can_id=self.can_id,
            urdf_path=urdf_path,
            arm_end_type=self.arm_end_type,
            enable_arm=self.auto_enable,
        )
        if not self.y1_interface.Init():
            self.get_logger().error("Init Y1 SDK Interface failed")
            raise RuntimeError("Y1 SDK Init failed")

        # QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # 设置控制模式
        if self.arm_control_type == "leader_arm":
            self.y1_interface.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        elif self.arm_control_type == "follower_arm":
            self.y1_interface.SetArmControlMode(ControlMode.RT_JOINT_POSITION)
            self.arm_end_pose_sub = self.create_subscription(
                ArmEndPoseControl, 
                self.arm_end_pose_control_topic, 
                self.arm_end_pose_callback, 
                qos_profile
            )
            self.arm_joint_pos_sub = self.create_subscription(
                ArmJointState, 
                self.arm_joint_position_control_topic, 
                self.follow_arm_joint_callback, 
                qos_profile
            )

            # for vr teleoprate
            # "/vr_left/end_pose"
            self.vr_end_pose_sub = self.create_subscription(
                ArmEndPoseControl, 
                self.vr_end_pose_control_topic,
                self.vr_end_pose_callback, 
                qos_profile
            )

            self.vr_gripper_sub = self.create_subscription(
                GripperControl, 
                self.vr_gripper_control_topic, 
                self.vr_gripper_callback, 
                qos_profile
            )

        elif self.arm_control_type == "normal_arm":
            self.y1_interface.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
            self.arm_end_pose_sub = self.create_subscription(
                ArmEndPoseControl, 
                self.arm_end_pose_control_topic, 
                self.arm_end_pose_callback, 
                qos_profile
            )
            self.arm_joint_pos_sub = self.create_subscription(
                ArmJointPositionControl, 
                self.arm_joint_position_control_topic, 
                self.arm_joint_position_callback, 
                qos_profile
            )
            if self.is_sim:
                self.arm_joint_pos_sub = self.create_subscription(
                JointState,
                "/joint_states",
                self.sim_joint_position_callback,
                qos_profile
                )
        else:
            self.get_logger().error(f"arm_control_type {self.arm_control_type} not supported")
            raise RuntimeError("Unsupported arm_control_type")

        # 发布器
        self.arm_joint_state_pub = self.create_publisher(
            ArmJointState, 
            self.arm_joint_state_topic, 
            qos_profile
        )
        self.arm_status_pub = self.create_publisher(
            ArmStatus, 
            self.arm_status_topic, 
            qos_profile
        )

        # 定时器 - ROS2方式
        timer_period = 1.0 / self.arm_feedback_rate
        self.timer = self.create_timer(timer_period, self.arm_information_timer_callback)

        self.get_logger().info("Y1 Controller initialized successfully!")

    # ---------------- 回调函数 ----------------
    def arm_end_pose_callback(self, msg: ArmEndPoseControl):
        arm_end_pose = list(msg.end_pose[:6])
        self.y1_interface.SetArmEndPose(arm_end_pose)
        self.y1_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def follow_arm_joint_callback(self, msg: ArmJointState):
        if len(msg.joint_position) >= 6:
            self.y1_interface.SetFollowerArmJointPosition(msg.joint_position)
        else:
            self.get_logger().error("follow arm receive joint control size < 6")

    def vr_end_pose_callback(self, msg: ArmEndPoseControl):
        # only control J1 - J6
        arm_end_pose = list(msg.end_pose[:6])
        self.y1_interface.SetArmEndPose(arm_end_pose)

    def vr_gripper_callback(self, msg: GripperControl):
        # control gripper
        self.y1_interface.SetGripperStroke(msg.gripper_stroke, 5)
        # print(f"SetGripperStroke : {msg.gripper_stroke}")

    def arm_joint_position_callback(self, msg: ArmJointPositionControl):
        # control J1 - J6 joint
        arm_joint_position = list(msg.joint_position[:6])
        self.y1_interface.SetArmJointPosition(arm_joint_position, msg.joint_velocity)
        # control gripper
        self.y1_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def sim_joint_position_callback(self, msg: JointState):
        # 控制 J1 - J6 关节
        arm_joint_position = list(msg.position[:6])
        self.y1_interface.SetArmJointPosition(arm_joint_position, 6)
        # 控制夹爪
        if len(msg.position) >= 7:
         self.y1_interface.SetGripperStroke(-msg.position[6] * 2000, 6)

    def arm_information_timer_callback(self):
        # 发布关节状态
        arm_joint_state = ArmJointState()
        arm_joint_state.header.stamp = self.get_clock().now().to_msg()

        arm_end_pose = self.y1_interface.GetArmEndPose()
        joint_position = self.y1_interface.GetJointPosition()
        joint_velocity = self.y1_interface.GetJointVelocity()
        joint_effort = self.y1_interface.GetJointEffort()

        arm_joint_state.joint_position = joint_position
        arm_joint_state.joint_velocity = joint_velocity
        arm_joint_state.joint_effort = joint_effort
        arm_joint_state.end_pose = arm_end_pose[:6]

        # 发布电机状态
        arm_status = ArmStatus()
        arm_status.header.stamp = self.get_clock().now().to_msg()
        joint_names = self.y1_interface.GetJointNames()
        motor_current = self.y1_interface.GetMotorCurrent()
        rotor_temperature = self.y1_interface.GetRotorTemperature()
        joint_error_code = self.y1_interface.GetJointErrorCode()

        # ROS2中String数组的处理
        arm_status.name = [String(data=str(n)) for n in joint_names]
        arm_status.motor_current = motor_current + [sum(motor_current)]
        arm_status.rotor_temperature = rotor_temperature
        arm_status.error_code = joint_error_code

        self.arm_joint_state_pub.publish(arm_joint_state)
        self.arm_status_pub.publish(arm_status)


def main(args=None):
    rclpy.init(args=args)
    controller = Y1Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()


if __name__ == "__main__":
    main()