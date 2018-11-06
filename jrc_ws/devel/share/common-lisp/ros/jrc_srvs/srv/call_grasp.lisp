; Auto-generated. Do not edit!


(cl:in-package jrc_srvs-srv)


;//! \htmlinclude call_grasp-request.msg.html

(cl:defclass <call_grasp-request> (roslisp-msg-protocol:ros-message)
  ((grasp
    :reader grasp
    :initarg :grasp
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass call_grasp-request (<call_grasp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <call_grasp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'call_grasp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<call_grasp-request> is deprecated: use jrc_srvs-srv:call_grasp-request instead.")))

(cl:ensure-generic-function 'grasp-val :lambda-list '(m))
(cl:defmethod grasp-val ((m <call_grasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:grasp-val is deprecated.  Use jrc_srvs-srv:grasp instead.")
  (grasp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <call_grasp-request>) ostream)
  "Serializes a message object of type '<call_grasp-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'grasp) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <call_grasp-request>) istream)
  "Deserializes a message object of type '<call_grasp-request>"
    (cl:setf (cl:slot-value msg 'grasp) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<call_grasp-request>)))
  "Returns string type for a service object of type '<call_grasp-request>"
  "jrc_srvs/call_graspRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'call_grasp-request)))
  "Returns string type for a service object of type 'call_grasp-request"
  "jrc_srvs/call_graspRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<call_grasp-request>)))
  "Returns md5sum for a message object of type '<call_grasp-request>"
  "f6453fb762d13fe85f432f25bd98a414")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'call_grasp-request)))
  "Returns md5sum for a message object of type 'call_grasp-request"
  "f6453fb762d13fe85f432f25bd98a414")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<call_grasp-request>)))
  "Returns full string definition for message of type '<call_grasp-request>"
  (cl:format cl:nil "bool grasp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'call_grasp-request)))
  "Returns full string definition for message of type 'call_grasp-request"
  (cl:format cl:nil "bool grasp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <call_grasp-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <call_grasp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'call_grasp-request
    (cl:cons ':grasp (grasp msg))
))
;//! \htmlinclude call_grasp-response.msg.html

(cl:defclass <call_grasp-response> (roslisp-msg-protocol:ros-message)
  ((acted
    :reader acted
    :initarg :acted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass call_grasp-response (<call_grasp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <call_grasp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'call_grasp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<call_grasp-response> is deprecated: use jrc_srvs-srv:call_grasp-response instead.")))

(cl:ensure-generic-function 'acted-val :lambda-list '(m))
(cl:defmethod acted-val ((m <call_grasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:acted-val is deprecated.  Use jrc_srvs-srv:acted instead.")
  (acted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <call_grasp-response>) ostream)
  "Serializes a message object of type '<call_grasp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'acted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <call_grasp-response>) istream)
  "Deserializes a message object of type '<call_grasp-response>"
    (cl:setf (cl:slot-value msg 'acted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<call_grasp-response>)))
  "Returns string type for a service object of type '<call_grasp-response>"
  "jrc_srvs/call_graspResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'call_grasp-response)))
  "Returns string type for a service object of type 'call_grasp-response"
  "jrc_srvs/call_graspResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<call_grasp-response>)))
  "Returns md5sum for a message object of type '<call_grasp-response>"
  "f6453fb762d13fe85f432f25bd98a414")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'call_grasp-response)))
  "Returns md5sum for a message object of type 'call_grasp-response"
  "f6453fb762d13fe85f432f25bd98a414")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<call_grasp-response>)))
  "Returns full string definition for message of type '<call_grasp-response>"
  (cl:format cl:nil "bool acted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'call_grasp-response)))
  "Returns full string definition for message of type 'call_grasp-response"
  (cl:format cl:nil "bool acted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <call_grasp-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <call_grasp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'call_grasp-response
    (cl:cons ':acted (acted msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'call_grasp)))
  'call_grasp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'call_grasp)))
  'call_grasp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'call_grasp)))
  "Returns string type for a service object of type '<call_grasp>"
  "jrc_srvs/call_grasp")