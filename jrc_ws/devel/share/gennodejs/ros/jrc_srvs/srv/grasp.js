// Auto-generated. Do not edit!

// (in-package jrc_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class graspRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.agv_position = null;
      this.id = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('agv_position')) {
        this.agv_position = initObj.agv_position
      }
      else {
        this.agv_position = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type graspRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.int64(obj.mode, buffer, bufferOffset);
    // Serialize message field [agv_position]
    bufferOffset = _serializer.int64(obj.agv_position, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type graspRequest
    let len;
    let data = new graspRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [agv_position]
    data.agv_position = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 80;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/graspRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '806abab137b7e8996ed506841db6ef12';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 mode
    int64 agv_position
    int64 id
    geometry_msgs/Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new graspRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.agv_position !== undefined) {
      resolved.agv_position = msg.agv_position;
    }
    else {
      resolved.agv_position = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

class graspResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type graspResponse
    // Serialize message field [result]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type graspResponse
    let len;
    let data = new graspResponse(null);
    // Deserialize message field [result]
    data.result = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/graspResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2420602a83d8ccc0f3664e0daafab6c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Bool result
    
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new graspResponse(null);
    if (msg.result !== undefined) {
      resolved.result = std_msgs.msg.Bool.Resolve(msg.result)
    }
    else {
      resolved.result = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = {
  Request: graspRequest,
  Response: graspResponse,
  md5sum() { return '344890efce38d3f1768b922eb6058680'; },
  datatype() { return 'jrc_srvs/grasp'; }
};
