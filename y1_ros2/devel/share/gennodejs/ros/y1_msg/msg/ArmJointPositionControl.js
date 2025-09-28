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

class ArmJointPositionControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_position = null;
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
      if (initObj.hasOwnProperty('joint_position')) {
        this.joint_position = initObj.joint_position
      }
      else {
        this.joint_position = new Array(6).fill(0);
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
    // Serializes a message object of type ArmJointPositionControl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [joint_position] has the right length
    if (obj.joint_position.length !== 6) {
      throw new Error('Unable to serialize array field joint_position - length must be 6')
    }
    // Serialize message field [joint_position]
    bufferOffset = _arraySerializer.float64(obj.joint_position, buffer, bufferOffset, 6);
    // Serialize message field [joint_velocity]
    bufferOffset = _serializer.uint8(obj.joint_velocity, buffer, bufferOffset);
    // Serialize message field [gripper_stroke]
    bufferOffset = _serializer.float64(obj.gripper_stroke, buffer, bufferOffset);
    // Serialize message field [gripper_velocity]
    bufferOffset = _serializer.uint8(obj.gripper_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmJointPositionControl
    let len;
    let data = new ArmJointPositionControl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_position]
    data.joint_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
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
    return 'y1_msg/ArmJointPositionControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8eb12895f6b27b59d02d6906067fd799';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmJointPositionControl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_position !== undefined) {
      resolved.joint_position = msg.joint_position;
    }
    else {
      resolved.joint_position = new Array(6).fill(0)
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

module.exports = ArmJointPositionControl;
