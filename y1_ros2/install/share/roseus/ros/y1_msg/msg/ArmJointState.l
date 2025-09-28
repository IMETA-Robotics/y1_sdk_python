;; Auto-generated. Do not edit!


(when (boundp 'y1_msg::ArmJointState)
  (if (not (find-package "Y1_MSG"))
    (make-package "Y1_MSG"))
  (shadow 'ArmJointState (find-package "Y1_MSG")))
(unless (find-package "Y1_MSG::ARMJOINTSTATE")
  (make-package "Y1_MSG::ARMJOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmJointState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass y1_msg::ArmJointState
  :super ros::object
  :slots (_header _end_pose _joint_position _joint_velocity _joint_effort ))

(defmethod y1_msg::ArmJointState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:end_pose __end_pose) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_position __joint_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_effort __joint_effort) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _end_pose __end_pose)
   (setq _joint_position __joint_position)
   (setq _joint_velocity __joint_velocity)
   (setq _joint_effort __joint_effort)
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
  (:joint_position
   (&optional __joint_position)
   (if __joint_position (setq _joint_position __joint_position)) _joint_position)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:joint_effort
   (&optional __joint_effort)
   (if __joint_effort (setq _joint_effort __joint_effort)) _joint_effort)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[6] _end_pose
    (* 8    6)
    ;; float64[] _joint_position
    (* 8    (length _joint_position)) 4
    ;; float64[] _joint_velocity
    (* 8    (length _joint_velocity)) 4
    ;; float64[] _joint_effort
    (* 8    (length _joint_effort)) 4
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
     ;; float64[] _joint_position
     (write-long (length _joint_position) s)
     (dotimes (i (length _joint_position))
       (sys::poke (elt _joint_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocity
     (write-long (length _joint_velocity) s)
     (dotimes (i (length _joint_velocity))
       (sys::poke (elt _joint_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_effort
     (write-long (length _joint_effort) s)
     (dotimes (i (length _joint_effort))
       (sys::poke (elt _joint_effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
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
   ;; float64[] _joint_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get y1_msg::ArmJointState :md5sum-) "303651795b96b734ba0e948b824c3d6f")
(setf (get y1_msg::ArmJointState :datatype-) "y1_msg/ArmJointState")
(setf (get y1_msg::ArmJointState :definition-)
      "std_msgs/Header header

# x y z roll pitch yaw
float64[6] end_pose

# joint position information
float64[] joint_position

# joint velocity information
float64[] joint_velocity

# joint effort information
float64[] joint_effort
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



(provide :y1_msg/ArmJointState "303651795b96b734ba0e948b824c3d6f")


