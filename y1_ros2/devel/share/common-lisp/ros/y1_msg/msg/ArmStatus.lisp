; Auto-generated. Do not edit!


(cl:in-package y1_msg-msg)


;//! \htmlinclude ArmStatus.msg.html

(cl:defclass <ArmStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type (cl:vector std_msgs-msg:String)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:String :initial-element (cl:make-instance 'std_msgs-msg:String)))
   (error_code
    :reader error_code
    :initarg :error_code
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (rotor_temperature
    :reader rotor_temperature
    :initarg :rotor_temperature
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (motor_current
    :reader motor_current
    :initarg :motor_current
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ArmStatus (<ArmStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name y1_msg-msg:<ArmStatus> is deprecated: use y1_msg-msg:ArmStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArmStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:header-val is deprecated.  Use y1_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ArmStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:name-val is deprecated.  Use y1_msg-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <ArmStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:error_code-val is deprecated.  Use y1_msg-msg:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'rotor_temperature-val :lambda-list '(m))
(cl:defmethod rotor_temperature-val ((m <ArmStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:rotor_temperature-val is deprecated.  Use y1_msg-msg:rotor_temperature instead.")
  (rotor_temperature m))

(cl:ensure-generic-function 'motor_current-val :lambda-list '(m))
(cl:defmethod motor_current-val ((m <ArmStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader y1_msg-msg:motor_current-val is deprecated.  Use y1_msg-msg:motor_current instead.")
  (motor_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmStatus>) ostream)
  "Serializes a message object of type '<ArmStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'error_code))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'error_code))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rotor_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'rotor_temperature))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'motor_current))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmStatus>) istream)
  "Deserializes a message object of type '<ArmStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:String))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'error_code) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'error_code)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rotor_temperature) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rotor_temperature)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_current) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_current)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmStatus>)))
  "Returns string type for a message object of type '<ArmStatus>"
  "y1_msg/ArmStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmStatus)))
  "Returns string type for a message object of type 'ArmStatus"
  "y1_msg/ArmStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmStatus>)))
  "Returns md5sum for a message object of type '<ArmStatus>"
  "9bd0b4838b8ed632646459a083d5c41d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmStatus)))
  "Returns md5sum for a message object of type 'ArmStatus"
  "9bd0b4838b8ed632646459a083d5c41d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmStatus>)))
  "Returns full string definition for message of type '<ArmStatus>"
  (cl:format cl:nil "std_msgs/Header header~%~%# joint names~%std_msgs/String[] name~%~%# joint error codes ~%uint8[] error_code~%~%# motor internal coil temperature~%float32[] rotor_temperature~%~%# joint motor current~%float32[] motor_current~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmStatus)))
  "Returns full string definition for message of type 'ArmStatus"
  (cl:format cl:nil "std_msgs/Header header~%~%# joint names~%std_msgs/String[] name~%~%# joint error codes ~%uint8[] error_code~%~%# motor internal coil temperature~%float32[] rotor_temperature~%~%# joint motor current~%float32[] motor_current~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'error_code) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rotor_temperature) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmStatus
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':rotor_temperature (rotor_temperature msg))
    (cl:cons ':motor_current (motor_current msg))
))
