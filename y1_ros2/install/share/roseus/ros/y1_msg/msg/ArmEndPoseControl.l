;; Auto-generated. Do not edit!


(when (boundp 'y1_msg::ArmEndPoseControl)
  (if (not (find-package "Y1_MSG"))
    (make-package "Y1_MSG"))
  (shadow 'ArmEndPoseControl (find-package "Y1_MSG")))
(unless (find-package "Y1_MSG::ARMENDPOSECONTROL")
  (make-package "Y1_MSG::ARMENDPOSECONTROL"))

(in-package "ROS")
;;//! \htmlinclude ArmEndPoseControl.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass y1_msg::ArmEndPoseControl
  :super ros::object
  :slots (_header _end_pose _joint_velocity _gripper_stroke _gripper_velocity ))

(defmethod y1_msg::ArmEndPoseControl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:end_pose __end_pose) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) 0)
    ((:gripper_stroke __gripper_stroke) 0.0)
    ((:gripper_velocity __gripper_velocity) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _end_pose __end_pose)
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
  (:end_pose
   (&optional __end_pose)
   (if __end_pose (setq _end_pose __end_pose)) _end_pose)
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
    ;; float64[6] _end_pose
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
     ;; float64[6] _end_pose
     (dotimes (i 6)
       (sys::poke (elt _end_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64[6] _end_pose
   (dotimes (i (length _end_pose))
     (setf (elt _end_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
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

(setf (get y1_msg::ArmEndPoseControl :md5sum-) "142e2b7c756c7198bc7edcbeeeaba6ed")
(setf (get y1_msg::ArmEndPoseControl :datatype-) "y1_msg/ArmEndPoseControl")
(setf (get y1_msg::ArmEndPoseControl :definition-)
      "std_msgs/Header header

# x y z roll pitch yaw
float64[6] end_pose

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



(provide :y1_msg/ArmEndPoseControl "142e2b7c756c7198bc7edcbeeeaba6ed")


