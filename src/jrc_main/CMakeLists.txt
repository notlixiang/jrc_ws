cmake_minimum_required(VERSION 2.8)
project(jrc_main)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## 如果需要额外的 cmake_moudles:
## list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/ )


find_package(catkin REQUIRED COMPONENTS
    image_transport
    cv_bridge
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    tf
    ### 所有自定义的 srv 应该在这声明：
    jrc_srvs
    )

SET(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)


find_package(OpenCV 3 REQUIRED )
#find_package(PCL 1.7 REQUIRED)

message(STATUS "#######  OpenCV library status ######" )
message(STATUS "         version: ${OpenCV_VERSION}")
message(STATUS "         include_path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "         libraries: ${OpenCV_LIBS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    
    )


catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES realsense_driver
		#  CATKIN_DEPENDS roscpp rospy std_msgs
		#  DEPENDS system_lib
)


link_directories(${OpenCV_LIBRARY_DIR})
#link_directories(${PCL_LIBRARY_DIRS})


#add_definitions(${PCL_DEFINITIONS})

add_executable(jrc_main_node
	src/jrc_main.cpp
	src/AGV.cpp
	)
	
	
add_dependencies(jrc_main_node jrc_srvs_gencpp)


target_link_libraries(jrc_main_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	)
	
	
	
	
	
	
	
	
