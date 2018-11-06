; Auto-generated. Do not edit!


(cl:in-package jrc_srvs-srv)


;//! \htmlinclude pose-request.msg.html

(cl:defclass <pose-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass pose-request (<pose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<pose-request> is deprecated: use jrc_srvs-srv:pose-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:a-val is deprecated.  Use jrc_srvs-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:b-val is deprecated.  Use jrc_srvs-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pose-request>) ostream)
  "Serializes a message object of type '<pose-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pose-request>) istream)
  "Deserializes a message object of type '<pose-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pose-request>)))
  "Returns string type for a service object of type '<pose-request>"
  "jrc_srvs/poseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose-request)))
  "Returns string type for a service object of type 'pose-request"
  "jrc_srvs/poseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pose-request>)))
  "Returns md5sum for a message object of type '<pose-request>"
  "1f3af93331fc1032113dd90d9a5f0755")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pose-request)))
  "Returns md5sum for a message object of type 'pose-request"
  "1f3af93331fc1032113dd90d9a5f0755")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pose-request>)))
  "Returns full string definition for message of type '<pose-request>"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pose-request)))
  "Returns full string definition for message of type 'pose-request"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pose-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pose-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude pose-response.msg.html

(cl:defclass <pose-response> (roslisp-msg-protocol:ros-message)
  ((product
    :reader product
    :initarg :product
    :type cl:integer
    :initform 0))
)

(cl:defclass pose-response (<pose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jrc_srvs-srv:<pose-response> is deprecated: use jrc_srvs-srv:pose-response instead.")))

(cl:ensure-generic-function 'product-val :lambda-list '(m))
(cl:defmethod product-val ((m <pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jrc_srvs-srv:product-val is deprecated.  Use jrc_srvs-srv:product instead.")
  (product m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pose-response>) ostream)
  "Serializes a message object of type '<pose-response>"
  (cl:let* ((signed (cl:slot-value msg 'product)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pose-response>) istream)
  "Deserializes a message object of type '<pose-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'product) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pose-response>)))
  "Returns string type for a service object of type '<pose-response>"
  "jrc_srvs/poseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose-response)))
  "Returns string type for a service object of type 'pose-response"
  "jrc_srvs/poseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pose-response>)))
  "Returns md5sum for a message object of type '<pose-response>"
  "1f3af93331fc1032113dd90d9a5f0755")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pose-response)))
  "Returns md5sum for a message object of type 'pose-response"
  "1f3af93331fc1032113dd90d9a5f0755")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pose-response>)))
  "Returns full string definition for message of type '<pose-response>"
  (cl:format cl:nil "int64 product~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pose-response)))
  "Returns full string definition for message of type 'pose-response"
  (cl:format cl:nil "int64 product~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pose-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pose-response
    (cl:cons ':product (product msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pose)))
  'pose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pose)))
  'pose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose)))
  "Returns string type for a service object of type '<pose>"
  "jrc_srvs/pose")