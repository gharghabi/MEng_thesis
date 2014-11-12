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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

##Generate messages in the 'msg' folder
# add_action_files(
 # DIRECTORY action
  #FILES Fibonacci.action
#)

#add_message_files(
 #  	FILES
#	upperbodycore_msgs/jointPO.msg
#	upperbodycore_msgs/GestureDetect.msg
# )

 
## Generate services in the 'srv' folder
# add_service_files(
 #  FILES
  # AddTwoInts.srv
 #)

## Generate actions in the 'action' folder
#add_action_files(
#   DIRECTORY action
#   FILES
#   whatDidYouSay.action
# )

#generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
## Generate added messages and services with any dependencies listed here


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)
#  INCLUDE_DIRS include
#  LIBRARIES beginner_turorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

  
  
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
 target_link_libraries(myKinect_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} ~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect.so  
~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect_cv.so
~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect_sync.so)

 add_library(myImage_lib
 src/MyImage.cpp
 )
 target_link_libraries(myImage_lib ${OpenCV_LIBS} ${catkin_LIBRARIES} ~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect.so
~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect_cv.so
~/catkin_ws/src/thesis/rgbd_to_oni/include/libfreenect_sync.so
)




add_executable(main src/main.cpp)
target_link_libraries(main  ${catkin_LIBRARIES} myKinect_lib myImage_lib  )
add_dependencies(main  upperbodycore_msgs_generate_messages_cpp)

add_executable(convert_rgbd_to_oni src/convert_rgbd_to_oni.cpp)
target_link_libraries(convert_rgbd_to_oni  ${catkin_LIBRARIES})
add_dependencies(convert_rgbd_to_oni  upperbodycore_msgs_generate_messages_cpp)

add_executable(recordONIFile src/recordONIFile.cpp)
target_link_libraries(recordONIFile  ${catkin_LIBRARIES})
add_dependencies(recordONIFile  upperbodycore_msgs_generate_messages_cpp)

add_executable(convert_oni_to_pcd src/convert_oni_to_pcd.cpp)
target_link_libraries(convert_oni_to_pcd  ${catkin_LIBRARIES})
add_dependencies(convert_oni_to_pcd  upperbodycore_msgs_generate_messages_cpp)

add_executable(show_pcd_files src/show_pcd_files.cpp)
target_link_libraries(show_pcd_files  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES}  ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALISATION_LIBRARIES} ${publish_primitives} ${libvtkCommon.so} ${libvtkFiltering.so} /usr/lib/libvtkFiltering.so)
add_dependencies(show_pcd_files  upperbodycore_msgs_generate_messages_cpp)


