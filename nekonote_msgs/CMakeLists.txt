cmake_minimum_required(VERSION 2.8.3)
project(nekonote_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  trajectory_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JointCommand.msg
  Mode.msg
  Temperature.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  JointTrajectory.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS std_msgs actionlib_msgs trajectory_msgs message_runtime
)
