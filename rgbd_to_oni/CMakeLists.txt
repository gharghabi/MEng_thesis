cmake_minimum_required(VERSION 2.8.3)
project(rgbd_to_oni)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  genmsg
  pcl_conversions
  pcl_ros
  image_transport
  message_generation
  actionlib
  actionlib_msgs 
upperbodycore_msgs
)

## Find catkin and any catkin packages

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.6 REQUIRED)
find_package(OpenCV 2.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
pkg_check_modules(EIGEN REQUIRED eigen3)

add_definitions("-Wno-enam-compare")
add_definitions(${PCL_DEFINITIONS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_definitions(${PCL_DEFINITIONS})
include_directories(include  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
include_directories(include)
  
catkin_package(
  INCLUDE_DIRS include
)
  

include_directories(/usr/include/vtk-5.8)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
FIND_PACKAGE(CUDA)

#Include the FindCUDA script
INCLUDE(FindCUDA)


 add_library(myKinect_lib
 src/MyKinect.cpp
 ) 
 target_link_libraries(myKinect_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} ~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect.so)

 add_library(myImage_lib
 src/MyImage.cpp
 )
 target_link_libraries(myImage_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} ~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect.so)




add_executable(main src/main.cpp)
target_link_libraries(main  ${catkin_LIBRARIES} myKinect_lib myImage_lib  )
add_dependencies(main  upperbodycore_msgs_generate_messages_cpp)

