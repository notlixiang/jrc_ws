cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(velocity_smooth)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  tf
  message_generation
  jrc_srvs
)

#add_service_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
CATKIN_DEPENDS message_runtime
)


include_directories(
${catkin_INCLUDE_DIRS}
)



add_executable(velocity_smooth src/velocity_smooth.cpp)
add_dependencies(velocity_smooth jrc_srvs_gencpp)
target_link_libraries(velocity_smooth ${catkin_LIBRARIES} )

add_executable(velocity_smooth_client src/velocity_smooth_client.cpp)
add_dependencies(velocity_smooth_client jrc_srvs_gencpp)
target_link_libraries(velocity_smooth_client ${catkin_LIBRARIES} )

add_executable(velocity_smooth_client_2 src/velocity_smooth_client_2.cpp)
add_dependencies(velocity_smooth_client_2 jrc_srvs_gencpp)
target_link_libraries(velocity_smooth_client_2 ${catkin_LIBRARIES} )

#add_executable(test_srv src/test_srv.cpp)
#add_dependencies(test_srv smooth_srv_gencpp)
#target_link_libraries(test_srv ${catkin_LIBRARIES} )

#add_executable(test_client src/test_client.cpp)
#add_dependencies(test_client smooth_srv_gencpp)
#target_link_libraries(test_client ${catkin_LIBRARIES} )
