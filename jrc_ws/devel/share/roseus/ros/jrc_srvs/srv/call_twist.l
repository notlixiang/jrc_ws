;; Auto-generated. Do not edit!


(when (boundp 'jrc_srvs::call_twist)
  (if (not (find-package "JRC_SRVS"))
    (make-package "JRC_SRVS"))
  (shadow 'call_twist (find-package "JRC_SRVS")))
(unless (find-package "JRC_SRVS::CALL_TWIST")
  (make-package "JRC_SRVS::CALL_TWIST"))
(unless (find-package "JRC_SRVS::CALL_TWISTREQUEST")
  (make-package "JRC_SRVS::CALL_TWISTREQUEST"))
(unless (find-package "JRC_SRVS::CALL_TWISTRESPONSE")
  (make-package "JRC_SRVS::CALL_TWISTRESPONSE"))

(in-package "ROS")





(defclass jrc_srvs::call_twistRequest
  :super ros::object
  :slots (_angle ))

(defmethod jrc_srvs::call_twistRequest
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass jrc_srvs::call_twistResponse
  :super ros::object
  :slots ())

(defmethod jrc_srvs::call_twistResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass jrc_srvs::call_twist
  :super ros::object
  :slots ())

(setf (get jrc_srvs::call_twist :md5sum-) "2d11dcdbe5a6f73dd324353dc52315ab")
(setf (get jrc_srvs::call_twist :datatype-) "jrc_srvs/call_twist")
(setf (get jrc_srvs::call_twist :request) jrc_srvs::call_twistRequest)
(setf (get jrc_srvs::call_twist :response) jrc_srvs::call_twistResponse)

(defmethod jrc_srvs::call_twistRequest
  (:response () (instance jrc_srvs::call_twistResponse :init)))

(setf (get jrc_srvs::call_twistRequest :md5sum-) "2d11dcdbe5a6f73dd324353dc52315ab")
(setf (get jrc_srvs::call_twistRequest :datatype-) "jrc_srvs/call_twistRequest")
(setf (get jrc_srvs::call_twistRequest :definition-)
      "float32 angle


---
")

(setf (get jrc_srvs::call_twistResponse :md5sum-) "2d11dcdbe5a6f73dd324353dc52315ab")
(setf (get jrc_srvs::call_twistResponse :datatype-) "jrc_srvs/call_twistResponse")
(setf (get jrc_srvs::call_twistResponse :definition-)
      "float32 angle


---
")



(provide :jrc_srvs/call_twist "2d11dcdbe5a6f73dd324353dc52315ab")


