cmake_minimum_required(VERSION 2.8.3)
project(nekonote_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  actionlib_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
)

install(PROGRAMS
  scripts/dummy_joint_trajectory_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
