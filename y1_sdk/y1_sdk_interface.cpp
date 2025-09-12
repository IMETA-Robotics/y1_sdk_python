// bindings/wrapper.cpp

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>          // 支持 std::vector
#include <pybind11/functional.h>   // 如果以后有 callback
// 如果需要支持 numpy arrays 或 buffer，就引入 numpy.h 或 buffer.h

#include "y1_sdk/y1_sdk_interface.h"

namespace py = pybind11;

PYBIND11_MODULE(y1_sdk, m) {
    m.doc() = "Python binding for Y1SDKInterface";

    // 绑定枚举 ControlMode
    py::enum_<imeta::controller::Y1SDKInterface::ControlMode>(m, "ControlMode")
        .value("GRAVITY_COMPENSATION", imeta::controller::Y1SDKInterface::GRAVITY_COMPENSATION)
        .value("RT_JOINT_POSITION", imeta::controller::Y1SDKInterface::RT_JOINT_POSITION)
        .value("NRT_JOINT_POSITION", imeta::controller::Y1SDKInterface::NRT_JOINT_POSITION)
        .export_values();

    // 绑定类 Y1SDKInterface
    py::class_<imeta::controller::Y1SDKInterface>(m, "Y1SDKInterface")
        // 构造函数
        .def(py::init<const std::string&, const std::string&, int, bool>(),
             py::arg("can_id"), py::arg("urdf_path"),
             py::arg("arm_end_type"), py::arg("enable_arm"))
        // 析构函数自动处理

        // 方法
        .def("Init", &imeta::controller::Y1SDKInterface::Init,
             "Initialize the SDK interface. Returns true if success.")
        .def("GetJointNames", &imeta::controller::Y1SDKInterface::GetJointNames,
             "Returns the joint names (6 or 7 including gripper).")
        .def("GetRotorTemperature", &imeta::controller::Y1SDKInterface::GetRotorTemperature,
             "Returns rotor (coil) temperature for all joints.")
        .def("GetJointErrorCode", &imeta::controller::Y1SDKInterface::GetJointErrorCode,
             "Returns error codes for all joints.")
        .def("GetMotorCurrent", &imeta::controller::Y1SDKInterface::GetMotorCurrent,
             "Returns motor current for all joints.")
        .def("GetJointPosition", &imeta::controller::Y1SDKInterface::GetJointPosition,
             "Joint positions.")
        .def("GetJointVelocity", &imeta::controller::Y1SDKInterface::GetJointVelocity,
             "Joint velocities.")
        .def("GetJointEffort", &imeta::controller::Y1SDKInterface::GetJointEffort,
             "Joint torques / efforts.")
        .def("GetArmEndPose", &imeta::controller::Y1SDKInterface::GetArmEndPose,
             "End pose of the arm: [x, y, z, roll, pitch, yaw]")

        .def("SetArmControlMode", &imeta::controller::Y1SDKInterface::SetArmControlMode,
             "Set control mode",
             py::arg("mode"))

        // 控制函数
        .def("SetArmJointPosition", 
             (void (imeta::controller::Y1SDKInterface::*)(const std::array<double,6>&))
             &imeta::controller::Y1SDKInterface::SetArmJointPosition,
             "Set joint positions by array<double,6>",
             py::arg("arm_joint_position"))

          .def("SetArmJointVelocity",
               (void (imeta::controller::Y1SDKInterface::*)(double))
               &imeta::controller::Y1SDKInterface::SetArmJointVelocity,
               "Set joint velocity (single double)", 
               py::arg("arm_joint_velocity"))

        .def("ControlArmJoint",
             &imeta::controller::Y1SDKInterface::ControlArmJoint,
             "Set joint position + velocity control",
             py::arg("arm_joint_position"), py::arg("velocity"))

        .def("SetArmJointPosition_vec",
             (void (imeta::controller::Y1SDKInterface::*)(const std::vector<double>&))
             &imeta::controller::Y1SDKInterface::SetArmJointPosition,
             "Set joint positions by vector<double>",
             py::arg("arm_joint_position"))

        .def("SetArmJointVelocity_vec",
             (void (imeta::controller::Y1SDKInterface::*)(const std::vector<double>&))
             &imeta::controller::Y1SDKInterface::SetArmJointVelocity,
             "Set joint velocities by vector<double>",
             py::arg("arm_joint_velocity"))

        .def("SetArmEndPose", &imeta::controller::Y1SDKInterface::SetArmEndPose,
             "Set end pose [x,y,z,roll,pitch,yaw]",
             py::arg("arm_end_pose"))

        .def("SetGripperStroke", &imeta::controller::Y1SDKInterface::SetGripperStroke,
             "Set gripper stroke in mm",
             py::arg("gripper_stroke"))

        .def("ControlGripper", &imeta::controller::Y1SDKInterface::ControlGripper,
             "Set gripper stroke and velocity",
             py::arg("gripper_stroke"), py::arg("velocity"))

        .def("SetEnableArm", &imeta::controller::Y1SDKInterface::SetEnableArm,
             "Enable or disable arm motors",
             py::arg("enable_flag"))

        .def("SaveJ6ZeroPosition", &imeta::controller::Y1SDKInterface::SaveJ6ZeroPosition,
             "Save zero position for J6 joint when changing end flange.");

    // 结束 module
}
