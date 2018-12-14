cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(jrc_srvs)


find_package(catkin REQUIRED COMPONENTS
    image_transport
    cv_bridge
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    tf
)

add_service_files(
    FILES
    # for camera
    rgbd.srv
    rgbd_image.srv  
    #for detection
    bbox_msgs.srv
    #for pose
    obj_6d.srv
    # for end effector
    call_grasp.srv 
    call_twist.srv
<<<<<<< HEAD
    call_grasp_state.srv
=======
		call_grasp_state.srv
>>>>>>> 5fa1c67305db0ca5621719b18820c975f65c57db
    # for motion planning
    grasp.srv
    # for agv
    smooth.srv
    smooth_multi.srv

)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgb_depth
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)
