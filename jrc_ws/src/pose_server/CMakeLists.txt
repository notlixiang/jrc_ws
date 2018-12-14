cmake_minimum_required(VERSION 2.8)
project(pose_server)


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

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

############### dependencies ######################
#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
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
#openmp
# find_package(OpenMP REQUIRED)
# if (OPENMP_FOUND)
#     set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()
# set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
# set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")

# #self file
include_directories(../../devel/include)
include_directories(include)
include_directories( ${PROJECT_SOURCE_DIR}/include/pose_server)

############### execute ######################
# file(GLOB cpu_source_files "${PROJECT_SOURCE_DIR}/src/*.cpp")
# file(GLOB gpu_source_files "${PROJECT_SOURCE_DIR}/src/*.cu")

add_executable(pose src/main.cpp
                      src/ppf.cpp
                      src/hash_int.cpp)

add_executable(pose_center_normal src/center_normal.cpp)

target_link_libraries(pose
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

target_link_libraries(pose_center_normal
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

