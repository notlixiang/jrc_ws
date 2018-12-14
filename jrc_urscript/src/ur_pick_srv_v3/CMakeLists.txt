cmake_minimum_required(VERSION 2.8.3)
project(ur_pick_srv_v3)

add_compile_options(-std=c++11)

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    ur_msgs
    tf
    kdl_parser
    trac_ik_lib
)

catkin_package(
  CATKIN_DEPENDS
	roscpp 
	std_msgs
    ur_msgs
    trac_ik_lib
)

include_directories(${catkin_INCLUDE_DIRS} "../../devel/include" "/usr/include/eigen3")

add_executable(ur_pick_srv_v3_node src/ur_pick_srv.cpp)
target_link_libraries(ur_pick_srv_v3_node ${catkin_LIBRARIES})
