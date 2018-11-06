
(cl:in-package :asdf)

(defsystem "jrc_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "call_grasp" :depends-on ("_package_call_grasp"))
    (:file "_package_call_grasp" :depends-on ("_package"))
    (:file "call_twist" :depends-on ("_package_call_twist"))
    (:file "_package_call_twist" :depends-on ("_package"))
    (:file "grasp" :depends-on ("_package_grasp"))
    (:file "_package_grasp" :depends-on ("_package"))
    (:file "pose" :depends-on ("_package_pose"))
    (:file "_package_pose" :depends-on ("_package"))
    (:file "reco" :depends-on ("_package_reco"))
    (:file "_package_reco" :depends-on ("_package"))
    (:file "rgbd" :depends-on ("_package_rgbd"))
    (:file "_package_rgbd" :depends-on ("_package"))
  ))