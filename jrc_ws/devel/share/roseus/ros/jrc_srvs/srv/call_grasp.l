;; Auto-generated. Do not edit!


(when (boundp 'jrc_srvs::call_grasp)
  (if (not (find-package "JRC_SRVS"))
    (make-package "JRC_SRVS"))
  (shadow 'call_grasp (find-package "JRC_SRVS")))
(unless (find-package "JRC_SRVS::CALL_GRASP")
  (make-package "JRC_SRVS::CALL_GRASP"))
(unless (find-package "JRC_SRVS::CALL_GRASPREQUEST")
  (make-package "JRC_SRVS::CALL_GRASPREQUEST"))
(unless (find-package "JRC_SRVS::CALL_GRASPRESPONSE")
  (make-package "JRC_SRVS::CALL_GRASPRESPONSE"))

(in-package "ROS")





(defclass jrc_srvs::call_graspRequest
  :super ros::object
  :slots (_grasp ))

(defmethod jrc_srvs::call_graspRequest
  (:init
   (&key
    ((:grasp __grasp) nil)
    )
   (send-super :init)
   (setq _grasp __grasp)
   self)
  (:grasp
   (&optional __grasp)
   (if __grasp (setq _grasp __grasp)) _grasp)
  (:serialization-length
   ()
   (+
    ;; bool _grasp
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _grasp
       (if _grasp (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _grasp
     (setq _grasp (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jrc_srvs::call_graspResponse
  :super ros::object
  :slots (_acted ))

(defmethod jrc_srvs::call_graspResponse
  (:init
   (&key
    ((:acted __acted) nil)
    )
   (send-super :init)
   (setq _acted __acted)
   self)
  (:acted
   (&optional __acted)
   (if __acted (setq _acted __acted)) _acted)
  (:serialization-length
   ()
   (+
    ;; bool _acted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _acted
       (if _acted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _acted
     (setq _acted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jrc_srvs::call_grasp
  :super ros::object
  :slots ())

(setf (get jrc_srvs::call_grasp :md5sum-) "f6453fb762d13fe85f432f25bd98a414")
(setf (get jrc_srvs::call_grasp :datatype-) "jrc_srvs/call_grasp")
(setf (get jrc_srvs::call_grasp :request) jrc_srvs::call_graspRequest)
(setf (get jrc_srvs::call_grasp :response) jrc_srvs::call_graspResponse)

(defmethod jrc_srvs::call_graspRequest
  (:response () (instance jrc_srvs::call_graspResponse :init)))

(setf (get jrc_srvs::call_graspRequest :md5sum-) "f6453fb762d13fe85f432f25bd98a414")
(setf (get jrc_srvs::call_graspRequest :datatype-) "jrc_srvs/call_graspRequest")
(setf (get jrc_srvs::call_graspRequest :definition-)
      "bool grasp
---
bool acted

")

(setf (get jrc_srvs::call_graspResponse :md5sum-) "f6453fb762d13fe85f432f25bd98a414")
(setf (get jrc_srvs::call_graspResponse :datatype-) "jrc_srvs/call_graspResponse")
(setf (get jrc_srvs::call_graspResponse :definition-)
      "bool grasp
---
bool acted

")



(provide :jrc_srvs/call_grasp "f6453fb762d13fe85f432f25bd98a414")


