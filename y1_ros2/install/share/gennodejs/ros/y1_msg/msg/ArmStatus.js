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

class ArmStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.error_code = null;
      this.rotor_temperature = null;
      this.motor_current = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('error_code')) {
        this.error_code = initObj.error_code
      }
      else {
        this.error_code = [];
      }
      if (initObj.hasOwnProperty('rotor_temperature')) {
        this.rotor_temperature = initObj.rotor_temperature
      }
      else {
        this.rotor_temperature = [];
      }
      if (initObj.hasOwnProperty('motor_current')) {
        this.motor_current = initObj.motor_current
      }
      else {
        this.motor_current = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    // Serialize the length for message field [name]
    bufferOffset = _serializer.uint32(obj.name.length, buffer, bufferOffset);
    obj.name.forEach((val) => {
      bufferOffset = std_msgs.msg.String.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [error_code]
    bufferOffset = _arraySerializer.uint8(obj.error_code, buffer, bufferOffset, null);
    // Serialize message field [rotor_temperature]
    bufferOffset = _arraySerializer.float32(obj.rotor_temperature, buffer, bufferOffset, null);
    // Serialize message field [motor_current]
    bufferOffset = _arraySerializer.float32(obj.motor_current, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmStatus
    let len;
    let data = new ArmStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    // Deserialize array length for message field [name]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.name = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.name[i] = std_msgs.msg.String.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [error_code]
    data.error_code = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [rotor_temperature]
    data.rotor_temperature = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [motor_current]
    data.motor_current = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.name.forEach((val) => {
      length += std_msgs.msg.String.getMessageSize(val);
    });
    length += object.error_code.length;
    length += 4 * object.rotor_temperature.length;
    length += 4 * object.motor_current.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'y1_msg/ArmStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bd0b4838b8ed632646459a083d5c41d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = new Array(msg.name.length);
      for (let i = 0; i < resolved.name.length; ++i) {
        resolved.name[i] = std_msgs.msg.String.Resolve(msg.name[i]);
      }
    }
    else {
      resolved.name = []
    }

    if (msg.error_code !== undefined) {
      resolved.error_code = msg.error_code;
    }
    else {
      resolved.error_code = []
    }

    if (msg.rotor_temperature !== undefined) {
      resolved.rotor_temperature = msg.rotor_temperature;
    }
    else {
      resolved.rotor_temperature = []
    }

    if (msg.motor_current !== undefined) {
      resolved.motor_current = msg.motor_current;
    }
    else {
      resolved.motor_current = []
    }

    return resolved;
    }
};

module.exports = ArmStatus;
