cmake_minimum_required(VERSION 2.8.3)
project(nekonote_examples)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  std_msgs
  sensor_msgs
  nekonote_msgs
  nekonote_driver
  control_msgs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  actionlib
  std_msgs
  sensor_msgs
  nekonote_msgs
  nekonote_driver
  control_msgs
  trajectory_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(direct_teaching_no_ROS src/direct_teaching_no_ROS.cpp)
add_executable(direct_teaching_ROS src/direct_teaching_ROS.cpp)
add_executable(test_torque_control_ROS src/test_torque_control_ROS.cpp)

## Add cmake target dependencies of the executable
add_dependencies(direct_teaching_ROS ${catkin_EXPORTED_TARGETS})
add_dependencies(test_torque_control_ROS ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(direct_teaching_no_ROS ${catkin_LIBRARIES})
target_link_libraries(direct_teaching_ROS ${catkin_LIBRARIES})
target_link_libraries(test_torque_control_ROS ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/follow_joint_trajectory_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS direct_teaching_no_ROS direct_teaching_ROS test_torque_control_ROS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
