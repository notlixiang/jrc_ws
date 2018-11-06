; Auto-generated. Do not edit!


(cl:in-package jrc_srvs-srv)


;//! \htmlinclude call_twist-request.msg.html

(cl:defclass <call_twist-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass call_twist-request (<call_twist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <call_twist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'call_twist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<call_twist-request> is deprecated: use jrc_srvs-srv:call_twist-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <call_twist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:angle-val is deprecated.  Use jrc_srvs-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <call_twist-request>) ostream)
  "Serializes a message object of type '<call_twist-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <call_twist-request>) istream)
  "Deserializes a message object of type '<call_twist-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<call_twist-request>)))
  "Returns string type for a service object of type '<call_twist-request>"
  "jrc_srvs/call_twistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'call_twist-request)))
  "Returns string type for a service object of type 'call_twist-request"
  "jrc_srvs/call_twistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<call_twist-request>)))
  "Returns md5sum for a message object of type '<call_twist-request>"
  "2d11dcdbe5a6f73dd324353dc52315ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'call_twist-request)))
  "Returns md5sum for a message object of type 'call_twist-request"
  "2d11dcdbe5a6f73dd324353dc52315ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<call_twist-request>)))
  "Returns full string definition for message of type '<call_twist-request>"
  (cl:format cl:nil "float32 angle~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'call_twist-request)))
  "Returns full string definition for message of type 'call_twist-request"
  (cl:format cl:nil "float32 angle~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <call_twist-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <call_twist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'call_twist-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude call_twist-response.msg.html

(cl:defclass <call_twist-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass call_twist-response (<call_twist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <call_twist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'call_twist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<call_twist-response> is deprecated: use jrc_srvs-srv:call_twist-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <call_twist-response>) ostream)
  "Serializes a message object of type '<call_twist-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <call_twist-response>) istream)
  "Deserializes a message object of type '<call_twist-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<call_twist-response>)))
  "Returns string type for a service object of type '<call_twist-response>"
  "jrc_srvs/call_twistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'call_twist-response)))
  "Returns string type for a service object of type 'call_twist-response"
  "jrc_srvs/call_twistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<call_twist-response>)))
  "Returns md5sum for a message object of type '<call_twist-response>"
  "2d11dcdbe5a6f73dd324353dc52315ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'call_twist-response)))
  "Returns md5sum for a message object of type 'call_twist-response"
  "2d11dcdbe5a6f73dd324353dc52315ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<call_twist-response>)))
  "Returns full string definition for message of type '<call_twist-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'call_twist-response)))
  "Returns full string definition for message of type 'call_twist-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <call_twist-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <call_twist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'call_twist-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'call_twist)))
  'call_twist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'call_twist)))
  'call_twist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'call_twist)))
  "Returns string type for a service object of type '<call_twist>"
  "jrc_srvs/call_twist")