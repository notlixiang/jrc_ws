;; Auto-generated. Do not edit!


(when (boundp 'jrc_srvs::pose)
  (if (not (find-package "JRC_SRVS"))
    (make-package "JRC_SRVS"))
  (shadow 'pose (find-package "JRC_SRVS")))
(unless (find-package "JRC_SRVS::POSE")
  (make-package "JRC_SRVS::POSE"))
(unless (find-package "JRC_SRVS::POSEREQUEST")
  (make-package "JRC_SRVS::POSEREQUEST"))
(unless (find-package "JRC_SRVS::POSERESPONSE")
  (make-package "JRC_SRVS::POSERESPONSE"))

(in-package "ROS")





(defclass jrc_srvs::poseRequest
  :super ros::object
  :slots (_a _b ))

(defmethod jrc_srvs::poseRequest
  (:init
   (&key
    ((:a __a) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   (setq _b (round __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; int64 _a
    8
    ;; int64 _b
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _a
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _a (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _a) (= (length (_a . bv)) 2)) ;; bignum
              (write-long (ash (elt (_a . bv) 0) 0) s)
              (write-long (ash (elt (_a . bv) 1) -1) s))
             ((and (class _a) (= (length (_a . bv)) 1)) ;; big1
              (write-long (elt (_a . bv) 0) s)
              (write-long (if (>= _a 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _a s)(write-long (if (>= _a 0) 0 #xffffffff) s)))
     ;; int64 _b
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _b (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _b) (= (length (_b . bv)) 2)) ;; bignum
              (write-long (ash (elt (_b . bv) 0) 0) s)
              (write-long (ash (elt (_b . bv) 1) -1) s))
             ((and (class _b) (= (length (_b . bv)) 1)) ;; big1
              (write-long (elt (_b . bv) 0) s)
              (write-long (if (>= _b 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _b s)(write-long (if (>= _b 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _a
#+(or :alpha :irix6 :x86_64)
      (setf _a (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _a (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _b
#+(or :alpha :irix6 :x86_64)
      (setf _b (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _b (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass jrc_srvs::poseResponse
  :super ros::object
  :slots (_product ))

(defmethod jrc_srvs::poseResponse
  (:init
   (&key
    ((:product __product) 0)
    )
   (send-super :init)
   (setq _product (round __product))
   self)
  (:product
   (&optional __product)
   (if __product (setq _product __product)) _product)
  (:serialization-length
   ()
   (+
    ;; int64 _product
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _product
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _product (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _product) (= (length (_product . bv)) 2)) ;; bignum
              (write-long (ash (elt (_product . bv) 0) 0) s)
              (write-long (ash (elt (_product . bv) 1) -1) s))
             ((and (class _product) (= (length (_product . bv)) 1)) ;; big1
              (write-long (elt (_product . bv) 0) s)
              (write-long (if (>= _product 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _product s)(write-long (if (>= _product 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _product
#+(or :alpha :irix6 :x86_64)
      (setf _product (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _product (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass jrc_srvs::pose
  :super ros::object
  :slots ())

(setf (get jrc_srvs::pose :md5sum-) "1f3af93331fc1032113dd90d9a5f0755")
(setf (get jrc_srvs::pose :datatype-) "jrc_srvs/pose")
(setf (get jrc_srvs::pose :request) jrc_srvs::poseRequest)
(setf (get jrc_srvs::pose :response) jrc_srvs::poseResponse)

(defmethod jrc_srvs::poseRequest
  (:response () (instance jrc_srvs::poseResponse :init)))

(setf (get jrc_srvs::poseRequest :md5sum-) "1f3af93331fc1032113dd90d9a5f0755")
(setf (get jrc_srvs::poseRequest :datatype-) "jrc_srvs/poseRequest")
(setf (get jrc_srvs::poseRequest :definition-)
      "int64 a
int64 b
---
int64 product
")

(setf (get jrc_srvs::poseResponse :md5sum-) "1f3af93331fc1032113dd90d9a5f0755")
(setf (get jrc_srvs::poseResponse :datatype-) "jrc_srvs/poseResponse")
(setf (get jrc_srvs::poseResponse :definition-)
      "int64 a
int64 b
---
int64 product
")



(provide :jrc_srvs/pose "1f3af93331fc1032113dd90d9a5f0755")


