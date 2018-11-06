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

class poseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.a = null;
      this.b = null;
    }
    else {
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0;
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type poseRequest
    // Serialize message field [a]
    bufferOffset = _serializer.int64(obj.a, buffer, bufferOffset);
    // Serialize message field [b]
    bufferOffset = _serializer.int64(obj.b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type poseRequest
    let len;
    let data = new poseRequest(null);
    // Deserialize message field [a]
    data.a = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [b]
    data.b = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/poseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '36d09b846be0b371c5f190354dd3153e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 a
    int64 b
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new poseRequest(null);
    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = 0
    }

    return resolved;
    }
};

class poseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.product = null;
    }
    else {
      if (initObj.hasOwnProperty('product')) {
        this.product = initObj.product
      }
      else {
        this.product = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type poseResponse
    // Serialize message field [product]
    bufferOffset = _serializer.int64(obj.product, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type poseResponse
    let len;
    let data = new poseResponse(null);
    // Deserialize message field [product]
    data.product = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/poseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0999a385a89125a9804d02fa52c82670';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 product
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new poseResponse(null);
    if (msg.product !== undefined) {
      resolved.product = msg.product;
    }
    else {
      resolved.product = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: poseRequest,
  Response: poseResponse,
  md5sum() { return '1f3af93331fc1032113dd90d9a5f0755'; },
  datatype() { return 'jrc_srvs/pose'; }
};
