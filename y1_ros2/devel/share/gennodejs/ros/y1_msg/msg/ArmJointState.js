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

class ArmJointState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.end_pose = null;
      this.joint_position = null;
      this.joint_velocity = null;
      this.joint_effort = null;
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
      if (initObj.hasOwnProperty('joint_position')) {
        this.joint_position = initObj.joint_position
      }
      else {
        this.joint_position = [];
      }
      if (initObj.hasOwnProperty('joint_velocity')) {
        this.joint_velocity = initObj.joint_velocity
      }
      else {
        this.joint_velocity = [];
      }
      if (initObj.hasOwnProperty('joint_effort')) {
        this.joint_effort = initObj.joint_effort
      }
      else {
        this.joint_effort = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmJointState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [end_pose] has the right length
    if (obj.end_pose.length !== 6) {
      throw new Error('Unable to serialize array field end_pose - length must be 6')
    }
    // Serialize message field [end_pose]
    bufferOffset = _arraySerializer.float64(obj.end_pose, buffer, bufferOffset, 6);
    // Serialize message field [joint_position]
    bufferOffset = _arraySerializer.float64(obj.joint_position, buffer, bufferOffset, null);
    // Serialize message field [joint_velocity]
    bufferOffset = _arraySerializer.float64(obj.joint_velocity, buffer, bufferOffset, null);
    // Serialize message field [joint_effort]
    bufferOffset = _arraySerializer.float64(obj.joint_effort, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmJointState
    let len;
    let data = new ArmJointState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_pose]
    data.end_pose = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_position]
    data.joint_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_effort]
    data.joint_effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.joint_position.length;
    length += 8 * object.joint_velocity.length;
    length += 8 * object.joint_effort.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'y1_msg/ArmJointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '303651795b96b734ba0e948b824c3d6f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmJointState(null);
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

    if (msg.joint_position !== undefined) {
      resolved.joint_position = msg.joint_position;
    }
    else {
      resolved.joint_position = []
    }

    if (msg.joint_velocity !== undefined) {
      resolved.joint_velocity = msg.joint_velocity;
    }
    else {
      resolved.joint_velocity = []
    }

    if (msg.joint_effort !== undefined) {
      resolved.joint_effort = msg.joint_effort;
    }
    else {
      resolved.joint_effort = []
    }

    return resolved;
    }
};

module.exports = ArmJointState;
