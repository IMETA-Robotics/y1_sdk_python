#!/usr/bin/env python3
import rospy, rospkg
from y1_msg.msg import ArmStatus, ArmJointState, ArmEndPoseControl, ArmJointPositionControl
from std_msgs.msg import String
from y1_sdk import Y1SDKInterface, ControlMode

class Y1Controller:
    def __init__(self):
        rospy.init_node("y1_controller_node", anonymous=True)

        # ROS 参数
        self.can_id = rospy.get_param("~arm_can_id", "can0")
        self.arm_feedback_rate = rospy.get_param("~arm_feedback_rate", 200)
        self.arm_end_pose_control_topic = rospy.get_param(
            "~arm_end_pose_control_topic", "/y1/arm_end_pose_control"
        )
        self.arm_joint_position_control_topic = rospy.get_param(
            "~arm_joint_position_control_topic", "/y1/arm_joint_position_control_topic"
        )
        self.arm_joint_state_topic = rospy.get_param(
            "~arm_joint_state_topic", "/y1/arm_joint_state"
        )
        self.arm_status_topic = rospy.get_param(
            "~arm_status_topic", "/y1/arm_status"
        )
        self.arm_control_type = rospy.get_param("~arm_control_type", "follower_arm")
        self.arm_end_type = rospy.get_param("~arm_end_type", 0)
        self.auto_enable = rospy.get_param("~auto_enable", True)

        # URDF路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("y1_controller")

        if self.arm_end_type == 0:
            urdf_path = f"{package_path}/urdf/y10804.urdf"
        elif self.arm_end_type == 1:
            urdf_path = f"{package_path}/urdf/y1_gripper_t.urdf"
        elif self.arm_end_type == 2:
            urdf_path = f"{package_path}/urdf/y1_gripper_g.urdf"
        elif self.arm_end_type == 3:
            urdf_path = f"{package_path}/urdf/y10824_ee.urdf"
        else:
            rospy.logerr(f"arm_end_type {self.arm_end_type} not supported")
            raise RuntimeError("Unsupported arm_end_type")

        # 初始化 Y1 SDK
        self.y1_interface = Y1SDKInterface(
            can_id=self.can_id,
            urdf_path=urdf_path,
            arm_end_type=self.arm_end_type,
            enable_arm=self.auto_enable,
        )
        if not self.y1_interface.Init():
            rospy.logerr("Init Y1 SDK Interface failed")
            raise RuntimeError("Y1 SDK Init failed")

        # 设置控制模式
        if self.arm_control_type == "leader_arm":
            self.y1_interface.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        elif self.arm_control_type == "follower_arm":
            self.y1_interface.SetArmControlMode(ControlMode.RT_JOINT_POSITION)
            self.arm_end_pose_sub = rospy.Subscriber(
                self.arm_end_pose_control_topic, ArmEndPoseControl, self.arm_end_pose_callback
            )
            self.arm_joint_pos_sub = rospy.Subscriber(
                self.arm_joint_position_control_topic, ArmJointState, self.follow_arm_joint_callback
            )
        elif self.arm_control_type == "normal_arm":
            self.y1_interface.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
            self.arm_end_pose_sub = rospy.Subscriber(
                self.arm_end_pose_control_topic, ArmEndPoseControl, self.arm_end_pose_callback
            )
            self.arm_joint_pos_sub = rospy.Subscriber(
                self.arm_joint_position_control_topic, ArmJointPositionControl, self.arm_joint_position_callback
            )
        else:
            rospy.logerr(f"arm_control_type {self.arm_control_type} not supported")
            raise RuntimeError("Unsupported arm_control_type")

        # 发布器
        self.arm_joint_state_pub = rospy.Publisher(self.arm_joint_state_topic, ArmJointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher(self.arm_status_topic, ArmStatus, queue_size=1)

        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.arm_feedback_rate), self.arm_information_timer_callback)

        rospy.loginfo("Y1 Controller initialized successfully!")

    # ---------------- 回调函数 ----------------
    def arm_end_pose_callback(self, msg: ArmEndPoseControl):
        arm_end_pose = list(msg.end_pose[:6])
        self.y1_interface.SetArmEndPose(arm_end_pose)
        self.y1_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def follow_arm_joint_callback(self, msg: ArmJointState):
        if len(msg.joint_position) >= 6:
            self.y1_interface.SetFollowerArmJointPosition(msg.joint_position)
        else:
            rospy.logerr("follow arm receive joint control size < 6")

    def arm_joint_position_callback(self, msg: ArmJointPositionControl):
        # control J1 - J6 joint
        arm_joint_position = list(msg.joint_position[:6])
        self.y1_interface.SetArmJointPosition(arm_joint_position, msg.joint_velocity)
        # control gripper
        self.y1_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def arm_information_timer_callback(self, event):
        # 发布关节状态
        arm_joint_state = ArmJointState()
        arm_joint_state.header.stamp = rospy.Time.now()

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
        arm_status.header.stamp = rospy.Time.now()
        joint_names = self.y1_interface.GetJointNames()
        motor_current = self.y1_interface.GetMotorCurrent()
        rotor_temperature = self.y1_interface.GetRotorTemperature()
        joint_error_code = self.y1_interface.GetJointErrorCode()

        arm_status.name = [String(data=n) for n in joint_names]
        arm_status.motor_current = motor_current + [sum(motor_current)]
        arm_status.rotor_temperature = rotor_temperature
        arm_status.error_code = joint_error_code

        self.arm_joint_state_pub.publish(arm_joint_state)
        self.arm_status_pub.publish(arm_status)


if __name__ == "__main__":
    try:
        controller = Y1Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
