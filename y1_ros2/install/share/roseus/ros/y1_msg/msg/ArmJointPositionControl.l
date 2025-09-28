;; Auto-generated. Do not edit!


(when (boundp 'y1_msg::ArmJointPositionControl)
  (if (not (find-package "Y1_MSG"))
    (make-package "Y1_MSG"))
  (shadow 'ArmJointPositionControl (find-package "Y1_MSG")))
(unless (find-package "Y1_MSG::ARMJOINTPOSITIONCONTROL")
  (make-package "Y1_MSG::ARMJOINTPOSITIONCONTROL"))

(in-package "ROS")
;;//! \htmlinclude ArmJointPositionControl.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass y1_msg::ArmJointPositionControl
  :super ros::object
  :slots (_header _joint_position _joint_velocity _gripper_stroke _gripper_velocity ))

(defmethod y1_msg::ArmJointPositionControl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_position __joint_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) 0)
    ((:gripper_stroke __gripper_stroke) 0.0)
    ((:gripper_velocity __gripper_velocity) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_position __joint_position)
   (setq _joint_velocity (round __joint_velocity))
   (setq _gripper_stroke (float __gripper_stroke))
   (setq _gripper_velocity (round __gripper_velocity))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_position
   (&optional __joint_position)
   (if __joint_position (setq _joint_position __joint_position)) _joint_position)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:gripper_stroke
   (&optional __gripper_stroke)
   (if __gripper_stroke (setq _gripper_stroke __gripper_stroke)) _gripper_stroke)
  (:gripper_velocity
   (&optional __gripper_velocity)
   (if __gripper_velocity (setq _gripper_velocity __gripper_velocity)) _gripper_velocity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[6] _joint_position
    (* 8    6)
    ;; uint8 _joint_velocity
    1
    ;; float64 _gripper_stroke
    8
    ;; uint8 _gripper_velocity
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[6] _joint_position
     (dotimes (i 6)
       (sys::poke (elt _joint_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _joint_velocity
       (write-byte _joint_velocity s)
     ;; float64 _gripper_stroke
       (sys::poke _gripper_stroke (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _gripper_velocity
       (write-byte _gripper_velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[6] _joint_position
   (dotimes (i (length _joint_position))
     (setf (elt _joint_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8 _joint_velocity
     (setq _joint_velocity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _gripper_stroke
     (setq _gripper_stroke (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _gripper_velocity
     (setq _gripper_velocity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get y1_msg::ArmJointPositionControl :md5sum-) "8eb12895f6b27b59d02d6906067fd799")
(setf (get y1_msg::ArmJointPositionControl :datatype-) "y1_msg/ArmJointPositionControl")
(setf (get y1_msg::ArmJointPositionControl :definition-)
      "std_msgs/Header header

# 6 joint position
float64[6] joint_position

# range: [1, 10], 1 is slow, 10 is fast
uint8 joint_velocity

# individual gripper control
float64 gripper_stroke

# range: [1, 10], 1 is slow, 10 is fast
uint8 gripper_velocity
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

")



(provide :y1_msg/ArmJointPositionControl "8eb12895f6b27b59d02d6906067fd799")


