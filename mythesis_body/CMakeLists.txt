cmake_minimum_required(VERSION 2.8.3)
project(mythesis_body)


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

  
  
 add_library(body_features_lib
 src/skeleton/calc_features.cpp
 )
 
 target_link_libraries(body_features_lib ${OpenCV_LIBS} ${catkin_LIBRARIES})

include_directories(/usr/include/vtk-5.8)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
FIND_PACKAGE(CUDA)

#Include the FindCUDA script
INCLUDE(FindCUDA)


## Declare a cpp library
# add_library(beginner_turorials
#   src/${PROJECT_NAME}/beginner_turorials.cpp
# )

## Declare a cpp executable
# add_executable(beginner_turorials_node src/beginner_turorials_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_turorials_node athomerobot_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_turorials_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beginner_turorials beginner_turorials_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_turorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(body_features src/skeleton/body_features.cpp)
target_link_libraries(body_features  ${catkin_LIBRARIES} body_features_lib)
add_dependencies(body_features  upperbodycore_msgs_generate_messages_cpp)

add_executable(testSkeletonFeaturesOnDataset src/skeleton/testSkeletonFeaturesOnDataset.cpp)
target_link_libraries(testSkeletonFeaturesOnDataset  ${catkin_LIBRARIES} body_features_lib)
add_dependencies(testSkeletonFeaturesOnDataset  upperbodycore_msgs_generate_messages_cpp)

add_executable(calc_rank_1__activity_database src/skeleton/calc_rank_1__activity_database.cpp)
target_link_libraries(calc_rank_1__activity_database  ${catkin_LIBRARIES} body_features_lib)
add_dependencies(calc_rank_1__activity_database  upperbodycore_msgs_generate_messages_cpp)


add_executable(PLY_TO_PCD src/vfh/PLY_TO_PCD.cpp)
target_link_libraries(PLY_TO_PCD  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(PLY_TO_PCD  upperbodycore_msgs_generate_messages_cpp)


add_executable(create_human_model src/vfh/create_human_model.cpp)
target_link_libraries(create_human_model  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(create_human_model  upperbodycore_msgs_generate_messages_cpp)


add_executable(create_vfh_model src/vfh/create_vfh_model.cpp)
target_link_libraries(create_vfh_model  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(create_vfh_model  upperbodycore_msgs_generate_messages_cpp)

add_executable(createModelFromDataSet src/vfh/createModelFromDataSet.cpp)
target_link_libraries(createModelFromDataSet  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(createModelFromDataSet  upperbodycore_msgs_generate_messages_cpp)

add_executable(testVFHOnDatasetRank_1 src/vfh/testVFHOnDatasetRank_1.cpp)
target_link_libraries(testVFHOnDatasetRank_1  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(testVFHOnDatasetRank_1  upperbodycore_msgs_generate_messages_cpp)

add_executable(testVFHOnDatasetRank_K src/vfh/testVFHOnDatasetRank_K.cpp)
target_link_libraries(testVFHOnDatasetRank_K  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(testVFHOnDatasetRank_K  upperbodycore_msgs_generate_messages_cpp)



#add_executable(ply2ply src/ply2ply.cpp)
#target_link_libraries(ply2ply  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

#add_dependencies(ply2ply  upperbodycore_msgs_generate_messages_cpp)



#add_executable(main_ground_based_people_detection src/main_ground_based_people_detection.cpp)
#target_link_libraries(main_ground_based_people_detection  ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} #${PCL_VISUALISATION_LIBRARIES} ${publish_primitives} ${libvtkCommon.so} ${libvtkFiltering.so} ${catkin_LIBRARIES})

#add_dependencies(main_ground_based_people_detection  upperbodycore_msgs_generate_messages_cpp)



#add_executable(speech src/speech.cpp)
#target_link_libraries(speech ${catkin_LIBRARIES})
#add_dependencies(speech athomerobot_msgs_generate_messages_cpp)

# %EndTag(FULLTEXT)%
