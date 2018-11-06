; Auto-generated. Do not edit!


(cl:in-package jrc_srvs-srv)


;//! \htmlinclude grasp-request.msg.html

(cl:defclass <grasp-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (agv_position
    :reader agv_position
    :initarg :agv_position
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass grasp-request (<grasp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grasp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grasp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<grasp-request> is deprecated: use jrc_srvs-srv:grasp-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <grasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:mode-val is deprecated.  Use jrc_srvs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'agv_position-val :lambda-list '(m))
(cl:defmethod agv_position-val ((m <grasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:agv_position-val is deprecated.  Use jrc_srvs-srv:agv_position instead.")
  (agv_position m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <grasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:id-val is deprecated.  Use jrc_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <grasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:pose-val is deprecated.  Use jrc_srvs-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grasp-request>) ostream)
  "Serializes a message object of type '<grasp-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'agv_position)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grasp-request>) istream)
  "Deserializes a message object of type '<grasp-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'agv_position) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grasp-request>)))
  "Returns string type for a service object of type '<grasp-request>"
  "jrc_srvs/graspRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp-request)))
  "Returns string type for a service object of type 'grasp-request"
  "jrc_srvs/graspRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grasp-request>)))
  "Returns md5sum for a message object of type '<grasp-request>"
  "344890efce38d3f1768b922eb6058680")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grasp-request)))
  "Returns md5sum for a message object of type 'grasp-request"
  "344890efce38d3f1768b922eb6058680")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grasp-request>)))
  "Returns full string definition for message of type '<grasp-request>"
  (cl:format cl:nil "int64 mode~%int64 agv_position~%int64 id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grasp-request)))
  "Returns full string definition for message of type 'grasp-request"
  (cl:format cl:nil "int64 mode~%int64 agv_position~%int64 id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grasp-request>))
  (cl:+ 0
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grasp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'grasp-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':agv_position (agv_position msg))
    (cl:cons ':id (id msg))
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude grasp-response.msg.html

(cl:defclass <grasp-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass grasp-response (<grasp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grasp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grasp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<grasp-response> is deprecated: use jrc_srvs-srv:grasp-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <grasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:result-val is deprecated.  Use jrc_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grasp-response>) ostream)
  "Serializes a message object of type '<grasp-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grasp-response>) istream)
  "Deserializes a message object of type '<grasp-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grasp-response>)))
  "Returns string type for a service object of type '<grasp-response>"
  "jrc_srvs/graspResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp-response)))
  "Returns string type for a service object of type 'grasp-response"
  "jrc_srvs/graspResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grasp-response>)))
  "Returns md5sum for a message object of type '<grasp-response>"
  "344890efce38d3f1768b922eb6058680")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grasp-response)))
  "Returns md5sum for a message object of type 'grasp-response"
  "344890efce38d3f1768b922eb6058680")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grasp-response>)))
  "Returns full string definition for message of type '<grasp-response>"
  (cl:format cl:nil "std_msgs/Bool result~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grasp-response)))
  "Returns full string definition for message of type 'grasp-response"
  (cl:format cl:nil "std_msgs/Bool result~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grasp-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grasp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'grasp-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'grasp)))
  'grasp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'grasp)))
  'grasp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp)))
  "Returns string type for a service object of type '<grasp>"
  "jrc_srvs/grasp")