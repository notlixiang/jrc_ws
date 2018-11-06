;; Auto-generated. Do not edit!


(when (boundp 'jrc_srvs::grasp)
  (if (not (find-package "JRC_SRVS"))
    (make-package "JRC_SRVS"))
  (shadow 'grasp (find-package "JRC_SRVS")))
(unless (find-package "JRC_SRVS::GRASP")
  (make-package "JRC_SRVS::GRASP"))
(unless (find-package "JRC_SRVS::GRASPREQUEST")
  (make-package "JRC_SRVS::GRASPREQUEST"))
(unless (find-package "JRC_SRVS::GRASPRESPONSE")
  (make-package "JRC_SRVS::GRASPRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jrc_srvs::graspRequest
  :super ros::object
  :slots (_mode _agv_position _id _pose ))

(defmethod jrc_srvs::graspRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:agv_position __agv_position) 0)
    ((:id __id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _agv_position (round __agv_position))
   (setq _id (round __id))
   (setq _pose __pose)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:agv_position
   (&optional __agv_position)
   (if __agv_position (setq _agv_position __agv_position)) _agv_position)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; int64 _mode
    8
    ;; int64 _agv_position
    8
    ;; int64 _id
    8
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _mode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _mode) (= (length (_mode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_mode . bv) 0) 0) s)
              (write-long (ash (elt (_mode . bv) 1) -1) s))
             ((and (class _mode) (= (length (_mode . bv)) 1)) ;; big1
              (write-long (elt (_mode . bv) 0) s)
              (write-long (if (>= _mode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _mode s)(write-long (if (>= _mode 0) 0 #xffffffff) s)))
     ;; int64 _agv_position
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _agv_position (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _agv_position) (= (length (_agv_position . bv)) 2)) ;; bignum
              (write-long (ash (elt (_agv_position . bv) 0) 0) s)
              (write-long (ash (elt (_agv_position . bv) 1) -1) s))
             ((and (class _agv_position) (= (length (_agv_position . bv)) 1)) ;; big1
              (write-long (elt (_agv_position . bv) 0) s)
              (write-long (if (>= _agv_position 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _agv_position s)(write-long (if (>= _agv_position 0) 0 #xffffffff) s)))
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
      (setf _mode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _mode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _agv_position
#+(or :alpha :irix6 :x86_64)
      (setf _agv_position (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _agv_position (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass jrc_srvs::graspResponse
  :super ros::object
  :slots (_result ))

(defmethod jrc_srvs::graspResponse
  (:init
   (&key
    ((:result __result) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&rest __result)
   (if (keywordp (car __result))
       (send* _result __result)
     (progn
       (if __result (setq _result (car __result)))
       _result)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Bool _result
    (send _result :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Bool _result
       (send _result :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Bool _result
     (send _result :deserialize buf ptr-) (incf ptr- (send _result :serialization-length))
   ;;
   self)
  )

(defclass jrc_srvs::grasp
  :super ros::object
  :slots ())

(setf (get jrc_srvs::grasp :md5sum-) "344890efce38d3f1768b922eb6058680")
(setf (get jrc_srvs::grasp :datatype-) "jrc_srvs/grasp")
(setf (get jrc_srvs::grasp :request) jrc_srvs::graspRequest)
(setf (get jrc_srvs::grasp :response) jrc_srvs::graspResponse)

(defmethod jrc_srvs::graspRequest
  (:response () (instance jrc_srvs::graspResponse :init)))

(setf (get jrc_srvs::graspRequest :md5sum-) "344890efce38d3f1768b922eb6058680")
(setf (get jrc_srvs::graspRequest :datatype-) "jrc_srvs/graspRequest")
(setf (get jrc_srvs::graspRequest :definition-)
      "int64 mode
int64 agv_position
int64 id
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
std_msgs/Bool result


================================================================================
MSG: std_msgs/Bool
")

(setf (get jrc_srvs::graspResponse :md5sum-) "344890efce38d3f1768b922eb6058680")
(setf (get jrc_srvs::graspResponse :datatype-) "jrc_srvs/graspResponse")
(setf (get jrc_srvs::graspResponse :definition-)
      "int64 mode
int64 agv_position
int64 id
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
std_msgs/Bool result


================================================================================
MSG: std_msgs/Bool
")



(provide :jrc_srvs/grasp "344890efce38d3f1768b922eb6058680")


