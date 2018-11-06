// Auto-generated. Do not edit!

// (in-package jrc_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class call_graspRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasp = null;
    }
    else {
      if (initObj.hasOwnProperty('grasp')) {
        this.grasp = initObj.grasp
      }
      else {
        this.grasp = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type call_graspRequest
    // Serialize message field [grasp]
    bufferOffset = _serializer.bool(obj.grasp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type call_graspRequest
    let len;
    let data = new call_graspRequest(null);
    // Deserialize message field [grasp]
    data.grasp = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/call_graspRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb3ac9f2d2f64667074ec82e617b09f2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool grasp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new call_graspRequest(null);
    if (msg.grasp !== undefined) {
      resolved.grasp = msg.grasp;
    }
    else {
      resolved.grasp = false
    }

    return resolved;
    }
};

class call_graspResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.acted = null;
    }
    else {
      if (initObj.hasOwnProperty('acted')) {
        this.acted = initObj.acted
      }
      else {
        this.acted = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type call_graspResponse
    // Serialize message field [acted]
    bufferOffset = _serializer.bool(obj.acted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type call_graspResponse
    let len;
    let data = new call_graspResponse(null);
    // Deserialize message field [acted]
    data.acted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/call_graspResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b9cffd1eb0784577656f95ec0b26770';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool acted
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new call_graspResponse(null);
    if (msg.acted !== undefined) {
      resolved.acted = msg.acted;
    }
    else {
      resolved.acted = false
    }

    return resolved;
    }
};

module.exports = {
  Request: call_graspRequest,
  Response: call_graspResponse,
  md5sum() { return 'f6453fb762d13fe85f432f25bd98a414'; },
  datatype() { return 'jrc_srvs/call_grasp'; }
};
