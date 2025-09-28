;; Auto-generated. Do not edit!


(when (boundp 'y1_msg::ArmStatus)
  (if (not (find-package "Y1_MSG"))
    (make-package "Y1_MSG"))
  (shadow 'ArmStatus (find-package "Y1_MSG")))
(unless (find-package "Y1_MSG::ARMSTATUS")
  (make-package "Y1_MSG::ARMSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ArmStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass y1_msg::ArmStatus
  :super ros::object
  :slots (_header _name _error_code _rotor_temperature _motor_current ))

(defmethod y1_msg::ArmStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) ())
    ((:error_code __error_code) (make-array 0 :initial-element 0 :element-type :char))
    ((:rotor_temperature __rotor_temperature) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:motor_current __motor_current) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _error_code __error_code)
   (setq _rotor_temperature __rotor_temperature)
   (setq _motor_current __motor_current)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&rest __name)
   (if (keywordp (car __name))
       (send* _name __name)
     (progn
       (if __name (setq _name (car __name)))
       _name)))
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:rotor_temperature
   (&optional __rotor_temperature)
   (if __rotor_temperature (setq _rotor_temperature __rotor_temperature)) _rotor_temperature)
  (:motor_current
   (&optional __motor_current)
   (if __motor_current (setq _motor_current __motor_current)) _motor_current)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/String[] _name
    (apply #'+ (send-all _name :serialization-length)) 4
    ;; uint8[] _error_code
    (* 1    (length _error_code)) 4
    ;; float32[] _rotor_temperature
    (* 4    (length _rotor_temperature)) 4
    ;; float32[] _motor_current
    (* 4    (length _motor_current)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/String[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (send elem :serialize s)
       )
     ;; uint8[] _error_code
     (write-long (length _error_code) s)
     (princ _error_code s)
     ;; float32[] _rotor_temperature
     (write-long (length _rotor_temperature) s)
     (dotimes (i (length _rotor_temperature))
       (sys::poke (elt _rotor_temperature i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _motor_current
     (write-long (length _motor_current) s)
     (dotimes (i (length _motor_current))
       (sys::poke (elt _motor_current i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/String[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (let (r) (dotimes (i n) (push (instance std_msgs::String :init) r)) r))
     (dolist (elem- _name)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint8[] _error_code
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _error_code (make-array n :element-type :char))
     (replace _error_code buf :start2 ptr-) (incf ptr- n))
   ;; float32[] _rotor_temperature
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rotor_temperature (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rotor_temperature i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _motor_current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motor_current (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _motor_current i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get y1_msg::ArmStatus :md5sum-) "9bd0b4838b8ed632646459a083d5c41d")
(setf (get y1_msg::ArmStatus :datatype-) "y1_msg/ArmStatus")
(setf (get y1_msg::ArmStatus :definition-)
      "std_msgs/Header header

# joint names
std_msgs/String[] name

# joint error codes 
uint8[] error_code

# motor internal coil temperature
float32[] rotor_temperature

# joint motor current
float32[] motor_current
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: std_msgs/String
string data

")



(provide :y1_msg/ArmStatus "9bd0b4838b8ed632646459a083d5c41d")


