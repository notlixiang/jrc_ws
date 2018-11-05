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
    ### 所有的 .srv 文件
    reco.srv
    pose.srv
    rgbd.srv
    call_grasp.srv
    call_twist.srv
	grasp.srv

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
