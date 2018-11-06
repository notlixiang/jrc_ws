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

let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class rgbdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rgbdRequest
    // Serialize message field [start]
    bufferOffset = _serializer.bool(obj.start, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rgbdRequest
    let len;
    let data = new rgbdRequest(null);
    // Deserialize message field [start]
    data.start = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/rgbdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '676aa7bfb3ec2071e814f2368dfd5fb5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool start
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rgbdRequest(null);
    if (msg.start !== undefined) {
      resolved.start = msg.start;
    }
    else {
      resolved.start = false
    }

    return resolved;
    }
};

class rgbdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rgb_image = null;
      this.depth_image = null;
    }
    else {
      if (initObj.hasOwnProperty('rgb_image')) {
        this.rgb_image = initObj.rgb_image
      }
      else {
        this.rgb_image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('depth_image')) {
        this.depth_image = initObj.depth_image
      }
      else {
        this.depth_image = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rgbdResponse
    // Serialize message field [rgb_image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.rgb_image, buffer, bufferOffset);
    // Serialize message field [depth_image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.depth_image, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rgbdResponse
    let len;
    let data = new rgbdResponse(null);
    // Deserialize message field [rgb_image]
    data.rgb_image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [depth_image]
    data.depth_image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.rgb_image);
    length += sensor_msgs.msg.Image.getMessageSize(object.depth_image);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jrc_srvs/rgbdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33cb5836216cc2f10faa186b171cbfdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image rgb_image
    sensor_msgs/Image depth_image
    
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of cameara
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rgbdResponse(null);
    if (msg.rgb_image !== undefined) {
      resolved.rgb_image = sensor_msgs.msg.Image.Resolve(msg.rgb_image)
    }
    else {
      resolved.rgb_image = new sensor_msgs.msg.Image()
    }

    if (msg.depth_image !== undefined) {
      resolved.depth_image = sensor_msgs.msg.Image.Resolve(msg.depth_image)
    }
    else {
      resolved.depth_image = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = {
  Request: rgbdRequest,
  Response: rgbdResponse,
  md5sum() { return '4eef606dcba8e10e7c0e4a74324986c4'; },
  datatype() { return 'jrc_srvs/rgbd'; }
};
