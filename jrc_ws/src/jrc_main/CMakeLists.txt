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

############### dependencies ######################

#Opencv
find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})
# Eigen
include_directories( "/usr/include/eigen3" )
include_directories(${EIGEN_INCLUDE_DIRS})
#ros
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  jrc_srvs
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

#self file
include_directories(../../devel/include)
include_directories(include)
include_directories( ${PROJECT_SOURCE_DIR}/include )

############### execute ######################
# file(GLOB cpu_source_files "${PROJECT_SOURCE_DIR}/src/*.cpp")

# add_executable(jrc ${cpu_source_files})
add_executable(jrc src/jrc_main_tmp.cpp)
add_dependencies(jrc jrc_srvs_gencpp)
target_link_libraries(jrc
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Eigen_LIBRARIES}
 )

add_executable(jrc_single src/jrc_main_single_grasp.cpp)
add_dependencies(jrc_single jrc_srvs_gencpp)
target_link_libraries(jrc_single
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Eigen_LIBRARIES}
 )

	
	
	
	
	
	
	
	
