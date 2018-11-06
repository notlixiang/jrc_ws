; Auto-generated. Do not edit!


(cl:in-package jrc_srvs-srv)


;//! \htmlinclude rgbd-request.msg.html

(cl:defclass <rgbd-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass rgbd-request (<rgbd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<rgbd-request> is deprecated: use jrc_srvs-srv:rgbd-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <rgbd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:start-val is deprecated.  Use jrc_srvs-srv:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbd-request>) ostream)
  "Serializes a message object of type '<rgbd-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbd-request>) istream)
  "Deserializes a message object of type '<rgbd-request>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbd-request>)))
  "Returns string type for a service object of type '<rgbd-request>"
  "jrc_srvs/rgbdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbd-request)))
  "Returns string type for a service object of type 'rgbd-request"
  "jrc_srvs/rgbdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbd-request>)))
  "Returns md5sum for a message object of type '<rgbd-request>"
  "4eef606dcba8e10e7c0e4a74324986c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbd-request)))
  "Returns md5sum for a message object of type 'rgbd-request"
  "4eef606dcba8e10e7c0e4a74324986c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbd-request>)))
  "Returns full string definition for message of type '<rgbd-request>"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbd-request)))
  "Returns full string definition for message of type 'rgbd-request"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbd-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbd-request
    (cl:cons ':start (start msg))
))
;//! \htmlinclude rgbd-response.msg.html

(cl:defclass <rgbd-response> (roslisp-msg-protocol:ros-message)
  ((rgb_image
    :reader rgb_image
    :initarg :rgb_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (depth_image
    :reader depth_image
    :initarg :depth_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass rgbd-response (<rgbd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<rgbd-response> is deprecated: use jrc_srvs-srv:rgbd-response instead.")))

(cl:ensure-generic-function 'rgb_image-val :lambda-list '(m))
(cl:defmethod rgb_image-val ((m <rgbd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:rgb_image-val is deprecated.  Use jrc_srvs-srv:rgb_image instead.")
  (rgb_image m))

(cl:ensure-generic-function 'depth_image-val :lambda-list '(m))
(cl:defmethod depth_image-val ((m <rgbd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:depth_image-val is deprecated.  Use jrc_srvs-srv:depth_image instead.")
  (depth_image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbd-response>) ostream)
  "Serializes a message object of type '<rgbd-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rgb_image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'depth_image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbd-response>) istream)
  "Deserializes a message object of type '<rgbd-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rgb_image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'depth_image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbd-response>)))
  "Returns string type for a service object of type '<rgbd-response>"
  "jrc_srvs/rgbdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbd-response)))
  "Returns string type for a service object of type 'rgbd-response"
  "jrc_srvs/rgbdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbd-response>)))
  "Returns md5sum for a message object of type '<rgbd-response>"
  "4eef606dcba8e10e7c0e4a74324986c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbd-response)))
  "Returns md5sum for a message object of type 'rgbd-response"
  "4eef606dcba8e10e7c0e4a74324986c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbd-response>)))
  "Returns full string definition for message of type '<rgbd-response>"
  (cl:format cl:nil "sensor_msgs/Image rgb_image~%sensor_msgs/Image depth_image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbd-response)))
  "Returns full string definition for message of type 'rgbd-response"
  (cl:format cl:nil "sensor_msgs/Image rgb_image~%sensor_msgs/Image depth_image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbd-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rgb_image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'depth_image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbd-response
    (cl:cons ':rgb_image (rgb_image msg))
    (cl:cons ':depth_image (depth_image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rgbd)))
  'rgbd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rgbd)))
  'rgbd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbd)))
  "Returns string type for a service object of type '<rgbd>"
  "jrc_srvs/rgbd")