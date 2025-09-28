; Auto-generated. Do not edit!


(cl:in-package y1_msg-msg)


;//! \htmlinclude ArmJointPositionControl.msg.html

(cl:defclass <ArmJointPositionControl> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (joint_position
    :reader joint_position
    :initarg :joint_position
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

(cl:defclass ArmJointPositionControl (<ArmJointPositionControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmJointPositionControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmJointPositionControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name y1_msg-msg:<ArmJointPositionControl> is deprecated: use y1_msg-msg:ArmJointPositionControl instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArmJointPositionControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:header-val is deprecated.  Use y1_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_position-val :lambda-list '(m))
(cl:defmethod joint_position-val ((m <ArmJointPositionControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:joint_position-val is deprecated.  Use y1_msg-msg:joint_position instead.")
  (joint_position m))

(cl:ensure-generic-function 'joint_velocity-val :lambda-list '(m))
(cl:defmethod joint_velocity-val ((m <ArmJointPositionControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:joint_velocity-val is deprecated.  Use y1_msg-msg:joint_velocity instead.")
  (joint_velocity m))

(cl:ensure-generic-function 'gripper_stroke-val :lambda-list '(m))
(cl:defmethod gripper_stroke-val ((m <ArmJointPositionControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:gripper_stroke-val is deprecated.  Use y1_msg-msg:gripper_stroke instead.")
  (gripper_stroke m))

(cl:ensure-generic-function 'gripper_velocity-val :lambda-list '(m))
(cl:defmethod gripper_velocity-val ((m <ArmJointPositionControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:gripper_velocity-val is deprecated.  Use y1_msg-msg:gripper_velocity instead.")
  (gripper_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmJointPositionControl>) ostream)
  "Serializes a message object of type '<ArmJointPositionControl>"
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
   (cl:slot-value msg 'joint_position))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmJointPositionControl>) istream)
  "Deserializes a message object of type '<ArmJointPositionControl>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'joint_position) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'joint_position)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmJointPositionControl>)))
  "Returns string type for a message object of type '<ArmJointPositionControl>"
  "y1_msg/ArmJointPositionControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmJointPositionControl)))
  "Returns string type for a message object of type 'ArmJointPositionControl"
  "y1_msg/ArmJointPositionControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmJointPositionControl>)))
  "Returns md5sum for a message object of type '<ArmJointPositionControl>"
  "8eb12895f6b27b59d02d6906067fd799")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmJointPositionControl)))
  "Returns md5sum for a message object of type 'ArmJointPositionControl"
  "8eb12895f6b27b59d02d6906067fd799")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmJointPositionControl>)))
  "Returns full string definition for message of type '<ArmJointPositionControl>"
  (cl:format cl:nil "std_msgs/Header header~%~%# 6 joint position~%float64[6] joint_position~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 joint_velocity~%~%# individual gripper control~%float64 gripper_stroke~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 gripper_velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmJointPositionControl)))
  "Returns full string definition for message of type 'ArmJointPositionControl"
  (cl:format cl:nil "std_msgs/Header header~%~%# 6 joint position~%float64[6] joint_position~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 joint_velocity~%~%# individual gripper control~%float64 gripper_stroke~%~%# range: [1, 10], 1 is slow, 10 is fast~%uint8 gripper_velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmJointPositionControl>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmJointPositionControl>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmJointPositionControl
    (cl:cons ':header (header msg))
    (cl:cons ':joint_position (joint_position msg))
    (cl:cons ':joint_velocity (joint_velocity msg))
    (cl:cons ':gripper_stroke (gripper_stroke msg))
    (cl:cons ':gripper_velocity (gripper_velocity msg))
))
