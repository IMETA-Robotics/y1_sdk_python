; Auto-generated. Do not edit!


(cl:in-package y1_msg-msg)


;//! \htmlinclude ArmEndPoseControl.msg.html

(cl:defclass <ArmEndPoseControl> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (end_pose
    :reader end_pose
    :initarg :end_pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (joint_velocity
    :reader joint_velocity
    :initarg :joint_velocity
    :type cl:fixnum
    :initform 0)
   (gripper_stroke
    :reader gripper_stroke
    :initarg :gripper_stroke
    :type cl:float
    :initform 0.0)
   (gripper_velocity
    :reader gripper_velocity
    :initarg :gripper_velocity
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmEndPoseControl (<ArmEndPoseControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmEndPoseControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmEndPoseControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name y1_msg-msg:<ArmEndPoseControl> is deprecated: use y1_msg-msg:ArmEndPoseControl instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArmEndPoseControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:header-val is deprecated.  Use y1_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'end_pose-val :lambda-list '(m))
(cl:defmethod end_pose-val ((m <ArmEndPoseControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:end_pose-val is deprecated.  Use y1_msg-msg:end_pose instead.")
  (end_pose m))

(cl:ensure-generic-function 'joint_velocity-val :lambda-list '(m))
(cl:defmethod joint_velocity-val ((m <ArmEndPoseControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:joint_velocity-val is deprecated.  Use y1_msg-msg:joint_velocity instead.")
  (joint_velocity m))

(cl:ensure-generic-function 'gripper_stroke-val :lambda-list '(m))
(cl:defmethod gripper_stroke-val ((m <ArmEndPoseControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:gripper_stroke-val is deprecated.  Use y1_msg-msg:gripper_stroke instead.")
  (gripper_stroke m))

(cl:ensure-generic-function 'gripper_velocity-val :lambda-list '(m))
(cl:defmethod gripper_velocity-val ((m <ArmEndPoseControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:gripper_velocity-val is deprecated.  Use y1_msg-msg:gripper_velocity instead.")
  (gripper_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmEndPoseControl>) ostream)
  "Serializes a message object of type '<ArmEndPoseControl>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'end_pose))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_velocity)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gripper_stroke))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_velocity)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmEndPoseControl>) istream)
  "Deserializes a message object of type '<ArmEndPoseControl>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'end_pose) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'end_pose)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_velocity)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper_stroke) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_velocity)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmEndPoseControl>)))
  "Returns string type for a message object of type '<ArmEndPoseControl>"
  "y1_msg/ArmEndPoseControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmEndPoseControl)))
  "Returns string type for a message object of type 'ArmEndPoseControl"
  "y1_msg/ArmEndPoseControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmEndPoseControl>)))
  "Returns md5sum for a message object of type '<ArmEndPoseControl>"
  "142e2b7c756c7198bc7edcbeeeaba6ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmEndPoseControl)))
  "Returns md5sum for a message object of type 'ArmEndPoseControl"
  "142e2b7c756c7198bc7edcbeeeaba6ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmEndPoseControl>)))
  "Returns full string definition for message of type '<ArmEndPoseControl>"
  (cl:format cl:nil "std_msgs/Header header~%~%# x y z roll pitch yaw~%float64[6] end_pose~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 joint_velocity~%~%# individual gripper control~%float64 gripper_stroke~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 gripper_velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmEndPoseControl)))
  "Returns full string definition for message of type 'ArmEndPoseControl"
  (cl:format cl:nil "std_msgs/Header header~%~%# x y z roll pitch yaw~%float64[6] end_pose~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 joint_velocity~%~%# individual gripper control~%float64 gripper_stroke~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 gripper_velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmEndPoseControl>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'end_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmEndPoseControl>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmEndPoseControl
    (cl:cons ':header (header msg))
    (cl:cons ':end_pose (end_pose msg))
    (cl:cons ':joint_velocity (joint_velocity msg))
    (cl:cons ':gripper_stroke (gripper_stroke msg))
    (cl:cons ':gripper_velocity (gripper_velocity msg))
))
