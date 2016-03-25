# ros_nekonote

## Overview

Package which provides ROS tools for [NEKONOTE](http://products.rt-net.jp/lms/nekonote/nekonote-6dof-for-academic/) from RT Corp.

### Firmware Version

The firmware of NEKONTE which this package supported is version 1.1 or later. Please update the firmware if your NEKONOTE's firmware version is under 1.1. 

## Installation

If your catkin workspace is `~/catkin_ws`,
```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/rt-net/ros_nekonote.git
$ cd ..
$ sudo apt-get update  # if you need
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -r -y
$ catkin_make  # or any build commands available in ROS, e.g. catkin build
$ source devel/setup.bash
```

## Usage

Run the whole driver:
```sh
$ roslaunch nekonote_driver nekonote_bringup.launch
```

After the whole driver brings up, you can run sample programs (Just run one sample program at one time):
```sh
$ rosrun nekonote_examples direct_teaching_ROS  # Test direct-teaching 
$ rosrun nekonote_examples test_torque_control_ROS  # Test torque control
$ rosrun nekonote_examples follow_joint_trajectory_client.py  # Test Joint Trajectory Action
$ roslaunch nekonote_moveit_config nekonote_planning_execution.launch  # Test MoveIt!
```
