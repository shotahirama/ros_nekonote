cmake_minimum_required(VERSION 2.8.3)
project(nekonote_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  sensor_msgs
  nekonote_msgs
  control_msgs
  dynamic_reconfigure
)

catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/PositionJointTrajectoryActionServer.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  sensor_msgs
  nekonote_msgs
  control_msgs
  dynamic_reconfigure
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(nekonote_driver src/nekonote_driver.cpp)

## Add cmake target dependencies of the executable
add_dependencies(nekonote_driver ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(nekonote_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/follow_joint_trajectory_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS nekonote_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark launch file for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
