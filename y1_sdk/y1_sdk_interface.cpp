// bindings/wrapper.cpp

#include <pybind11/functional.h>  // 如果以后有 callback
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // 支持 std::vector
// 如果需要支持 numpy arrays 或 buffer，就引入 numpy.h 或 buffer.h

#include "y1_sdk/y1_sdk_interface.h"

namespace py = pybind11;

PYBIND11_MODULE(y1_sdk, m) {
  m.doc() = "Python binding for Y1SDKInterface";

  // 绑定枚举 ControlMode
  py::enum_<imeta::y1_controller::Y1SDKInterface::ControlMode>(m, "ControlMode")
      .value("GRAVITY_COMPENSATION",
             imeta::y1_controller::Y1SDKInterface::GRAVITY_COMPENSATION)
      .value("RT_JOINT_POSITION",
             imeta::y1_controller::Y1SDKInterface::RT_JOINT_POSITION)
      .value("NRT_JOINT_POSITION",
             imeta::y1_controller::Y1SDKInterface::NRT_JOINT_POSITION)
      .export_values();

  // 绑定类 Y1SDKInterface
  py::class_<imeta::y1_controller::Y1SDKInterface>(m, "Y1SDKInterface")
      // 构造函数
      .def(py::init<const std::string&, const std::string&, int, bool>(),
           py::arg("can_id"), py::arg("urdf_path"), py::arg("arm_end_type"),
           py::arg("enable_arm"))
      // 析构函数自动处理

      // 方法
      .def("Init", &imeta::y1_controller::Y1SDKInterface::Init,
           "Initialize the SDK interface. Returns true if success.")
      .def("GetJointNames", &imeta::y1_controller::Y1SDKInterface::GetJointNames,
           "Returns the joint names (6 or 7 including gripper).")
      .def("GetRotorTemperature",
           &imeta::y1_controller::Y1SDKInterface::GetRotorTemperature,
           "Returns rotor (coil) temperature for all joints.")
      .def("GetJointErrorCode",
           &imeta::y1_controller::Y1SDKInterface::GetJointErrorCode,
           "Returns error codes for all joints.")
      .def("GetMotorCurrent",
           &imeta::y1_controller::Y1SDKInterface::GetMotorCurrent,
           "Returns motor current for all joints.")
      .def("GetJointPosition",
           &imeta::y1_controller::Y1SDKInterface::GetJointPosition,
           "Joint positions.")
      .def("GetJointVelocity",
           &imeta::y1_controller::Y1SDKInterface::GetJointVelocity,
           "Joint velocities.")
      .def("GetJointEffort", &imeta::y1_controller::Y1SDKInterface::GetJointEffort,
           "Joint torques / efforts.")
      .def("GetArmEndPose", &imeta::y1_controller::Y1SDKInterface::GetArmEndPose,
           "End pose of the arm: [x, y, z, roll, pitch, yaw]")

      .def("SetArmControlMode",
           &imeta::y1_controller::Y1SDKInterface::SetArmControlMode,
           "Set control mode", py::arg("mode"))

      // 控制函数
      .def("SetArmJointPosition",
           (void (imeta::y1_controller::Y1SDKInterface::*)(
               const std::array<double, 6>&, int)) &
               imeta::y1_controller::Y1SDKInterface::SetArmJointPosition,
           "Set joint positions by std::array<double,6> with optional velocity "
           "ratio",
           py::arg("arm_joint_position"), py::arg("velocity_ratio") = 5)

      .def("SetFollowerArmJointPosition",
           (void (imeta::y1_controller::Y1SDKInterface::*)(
               const std::vector<double>&)) &
               imeta::y1_controller::Y1SDKInterface::SetArmJointPosition,
           "Set joint positions by vector<double>",
           py::arg("arm_joint_position"))

      .def("SetArmEndPose", &imeta::y1_controller::Y1SDKInterface::SetArmEndPose,
           "Set end pose [x,y,z,roll,pitch,yaw]", py::arg("arm_end_pose"))

      .def("SetGripperStroke",
           &imeta::y1_controller::Y1SDKInterface::SetGripperStroke,
           "Set gripper stroke in mm", py::arg("gripper_stroke"),
           py::arg("velocity_ratio") = 5)

      .def("SetEnableArm", &imeta::y1_controller::Y1SDKInterface::SetEnableArm,
           "Enable or disable arm motors", py::arg("enable_flag"))

      .def("SaveJ6ZeroPosition",
           &imeta::y1_controller::Y1SDKInterface::SaveJ6ZeroPosition,
           "Save zero position for J6 joint when changing end flange.");

  // 结束 module
}
