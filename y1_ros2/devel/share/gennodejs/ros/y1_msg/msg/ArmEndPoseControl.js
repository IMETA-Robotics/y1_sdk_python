// Auto-generated. Do not edit!

// (in-package y1_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArmEndPoseControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.end_pose = null;
      this.joint_velocity = null;
      this.gripper_stroke = null;
      this.gripper_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('end_pose')) {
        this.end_pose = initObj.end_pose
      }
      else {
        this.end_pose = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('joint_velocity')) {
        this.joint_velocity = initObj.joint_velocity
      }
      else {
        this.joint_velocity = 0;
      }
      if (initObj.hasOwnProperty('gripper_stroke')) {
        this.gripper_stroke = initObj.gripper_stroke
      }
      else {
        this.gripper_stroke = 0.0;
      }
      if (initObj.hasOwnProperty('gripper_velocity')) {
        this.gripper_velocity = initObj.gripper_velocity
      }
      else {
        this.gripper_velocity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmEndPoseControl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [end_pose] has the right length
    if (obj.end_pose.length !== 6) {
      throw new Error('Unable to serialize array field end_pose - length must be 6')
    }
    // Serialize message field [end_pose]
    bufferOffset = _arraySerializer.float64(obj.end_pose, buffer, bufferOffset, 6);
    // Serialize message field [joint_velocity]
    bufferOffset = _serializer.uint8(obj.joint_velocity, buffer, bufferOffset);
    // Serialize message field [gripper_stroke]
    bufferOffset = _serializer.float64(obj.gripper_stroke, buffer, bufferOffset);
    // Serialize message field [gripper_velocity]
    bufferOffset = _serializer.uint8(obj.gripper_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmEndPoseControl
    let len;
    let data = new ArmEndPoseControl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_pose]
    data.end_pose = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gripper_stroke]
    data.gripper_stroke = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper_velocity]
    data.gripper_velocity = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 58;
  }

  static datatype() {
    // Returns string type for a message object
    return 'y1_msg/ArmEndPoseControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '142e2b7c756c7198bc7edcbeeeaba6ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmEndPoseControl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.end_pose !== undefined) {
      resolved.end_pose = msg.end_pose;
    }
    else {
      resolved.end_pose = new Array(6).fill(0)
    }

    if (msg.joint_velocity !== undefined) {
      resolved.joint_velocity = msg.joint_velocity;
    }
    else {
      resolved.joint_velocity = 0
    }

    if (msg.gripper_stroke !== undefined) {
      resolved.gripper_stroke = msg.gripper_stroke;
    }
    else {
      resolved.gripper_stroke = 0.0
    }

    if (msg.gripper_velocity !== undefined) {
      resolved.gripper_velocity = msg.gripper_velocity;
    }
    else {
      resolved.gripper_velocity = 0
    }

    return resolved;
    }
};

module.exports = ArmEndPoseControl;
