;; Auto-generated. Do not edit!


(when (boundp 'jrc_srvs::rgbd)
  (if (not (find-package "JRC_SRVS"))
    (make-package "JRC_SRVS"))
  (shadow 'rgbd (find-package "JRC_SRVS")))
(unless (find-package "JRC_SRVS::RGBD")
  (make-package "JRC_SRVS::RGBD"))
(unless (find-package "JRC_SRVS::RGBDREQUEST")
  (make-package "JRC_SRVS::RGBDREQUEST"))
(unless (find-package "JRC_SRVS::RGBDRESPONSE")
  (make-package "JRC_SRVS::RGBDRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass jrc_srvs::rgbdRequest
  :super ros::object
  :slots (_start ))

(defmethod jrc_srvs::rgbdRequest
  (:init
   (&key
    ((:start __start) nil)
    )
   (send-super :init)
   (setq _start __start)
   self)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:serialization-length
   ()
   (+
    ;; bool _start
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _start
       (if _start (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _start
     (setq _start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jrc_srvs::rgbdResponse
  :super ros::object
  :slots (_rgb_image _depth_image ))

(defmethod jrc_srvs::rgbdResponse
  (:init
   (&key
    ((:rgb_image __rgb_image) (instance sensor_msgs::Image :init))
    ((:depth_image __depth_image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _rgb_image __rgb_image)
   (setq _depth_image __depth_image)
   self)
  (:rgb_image
   (&rest __rgb_image)
   (if (keywordp (car __rgb_image))
       (send* _rgb_image __rgb_image)
     (progn
       (if __rgb_image (setq _rgb_image (car __rgb_image)))
       _rgb_image)))
  (:depth_image
   (&rest __depth_image)
   (if (keywordp (car __depth_image))
       (send* _depth_image __depth_image)
     (progn
       (if __depth_image (setq _depth_image (car __depth_image)))
       _depth_image)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _rgb_image
    (send _rgb_image :serialization-length)
    ;; sensor_msgs/Image _depth_image
    (send _depth_image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _rgb_image
       (send _rgb_image :serialize s)
     ;; sensor_msgs/Image _depth_image
       (send _depth_image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _rgb_image
     (send _rgb_image :deserialize buf ptr-) (incf ptr- (send _rgb_image :serialization-length))
   ;; sensor_msgs/Image _depth_image
     (send _depth_image :deserialize buf ptr-) (incf ptr- (send _depth_image :serialization-length))
   ;;
   self)
  )

(defclass jrc_srvs::rgbd
  :super ros::object
  :slots ())

(setf (get jrc_srvs::rgbd :md5sum-) "4eef606dcba8e10e7c0e4a74324986c4")
(setf (get jrc_srvs::rgbd :datatype-) "jrc_srvs/rgbd")
(setf (get jrc_srvs::rgbd :request) jrc_srvs::rgbdRequest)
(setf (get jrc_srvs::rgbd :response) jrc_srvs::rgbdResponse)

(defmethod jrc_srvs::rgbdRequest
  (:response () (instance jrc_srvs::rgbdResponse :init)))

(setf (get jrc_srvs::rgbdRequest :md5sum-) "4eef606dcba8e10e7c0e4a74324986c4")
(setf (get jrc_srvs::rgbdRequest :datatype-) "jrc_srvs/rgbdRequest")
(setf (get jrc_srvs::rgbdRequest :definition-)
      "bool start
---
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
")

(setf (get jrc_srvs::rgbdResponse :md5sum-) "4eef606dcba8e10e7c0e4a74324986c4")
(setf (get jrc_srvs::rgbdResponse :datatype-) "jrc_srvs/rgbdResponse")
(setf (get jrc_srvs::rgbdResponse :definition-)
      "bool start
---
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
")



(provide :jrc_srvs/rgbd "4eef606dcba8e10e7c0e4a74324986c4")


