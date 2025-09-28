
(cl:in-package :asdf)

(defsystem "y1_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArmEndPoseControl" :depends-on ("_package_ArmEndPoseControl"))
    (:file "_package_ArmEndPoseControl" :depends-on ("_package"))
    (:file "ArmJointPositionControl" :depends-on ("_package_ArmJointPositionControl"))
    (:file "_package_ArmJointPositionControl" :depends-on ("_package"))
    (:file "ArmJointState" :depends-on ("_package_ArmJointState"))
    (:file "_package_ArmJointState" :depends-on ("_package"))
    (:file "ArmStatus" :depends-on ("_package_ArmStatus"))
    (:file "_package_ArmStatus" :depends-on ("_package"))
  ))