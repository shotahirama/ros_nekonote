//------------------------------------------------------------------------------
// Copyright (c) 2016, RT Corporation
// All rights reserved.

// License: BSD

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of RT Corporation nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/JointState.h"
#include "nekonote_msgs/Mode.h"
#include "nekonote_msgs/JointCommand.h"
#include "nekonote_msgs/JointTrajectoryAction.h"

bool robot_enabled = false;
std::vector<double> joint_angles;

void getJointAngles(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  joint_angles = joint_states->position;
  robot_enabled = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nekonote_direct_teaching");

  ros::NodeHandle nh;

  // ROS publisher
  ros::Publisher mode_cmd_pub = nh.advertise<nekonote_msgs::Mode>("robot/mode_command", 10);
  ros::Publisher joint_cmd_pub = nh.advertise<nekonote_msgs::JointCommand>("robot/joint_command", 10);

  // ROS subscriber
  ros::Subscriber joint_states_sub = nh.subscribe("robot/joint_states", 100, getJointAngles);

  // ROS action client
  actionlib::SimpleActionClient<nekonote_msgs::JointTrajectoryAction> ac("robot/joint_trajectory_action", true);

  ROS_INFO("Waiting for joint trajectory action server to start...");
  ac.waitForServer();
  ROS_INFO("Joint trajectory action server started.");

  ROS_INFO("Getting ROS parameter...");
  ros::Rate loop_rate(10);
  std::vector<std::string> joint_names;
  if (ros::param::get("nekonote_driver/joint_names", joint_names))
    ROS_INFO("Got ROS parameter");
  else
    ROS_ERROR("Cannot get ROS parameter");

  ROS_INFO("Waiting to subscribe /robot/joint_states...");
  while (!robot_enabled)
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (!ros::ok())
      return 0;
  }
  ROS_INFO("Subscribed /robot/joint_states. Now NekonoteDriver certainly connected with NEKONOTE.");

  nekonote_msgs::Mode mode;
  mode.state = nekonote_msgs::Mode::FREE;
  mode_cmd_pub.publish(mode);

  ROS_INFO("Start recording. Recording will automatically stop in about 10 seconds");
  nekonote_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names = joint_names;
  ros::spinOnce();
  std::vector<double> first_angles = joint_angles;
  ros::Time start_time = ros::Time::now();
  for (int i = 0; i < 100; i++)
  {
    loop_rate.sleep();
    ros::spinOnce();
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joint_angles;
    point.time_from_start = ros::Time::now() - start_time;
    goal.trajectory.points.push_back(point);
    if (!ros::ok())
      return 0;
  }

  ROS_INFO("Stop recording. After 5 seconds, go to initial state");
  mode.state = nekonote_msgs::Mode::POSITION_CONTROL;
  mode_cmd_pub.publish(mode);
  ros::Duration(5.0).sleep();

  ROS_INFO("Go to initial state");
  nekonote_msgs::JointCommand joint_command;
  joint_command.mode = nekonote_msgs::JointCommand::POSITION_CONTROL;
  joint_command.name = joint_names;
  joint_command.command = first_angles;
  joint_command.time = 2000;
  joint_cmd_pub.publish(joint_command);

  ROS_INFO("Send trajectory data. Playback will automatically start after 3 seconds");
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(3.0);
  ac.sendGoal(goal);

  ROS_INFO("If you want to cancel playback, type \"q\" and press Enter. Otherwise, type \"ok\" and press Enter");
  std::string input;
  std::cin >> input;
  if (input == "q")
  {
    ROS_INFO("Cancelling playback. Request preempt to the action server.");
    ac.cancelGoal();
    return 0;
  }
//  while (!ac.getState().isDone() && ros::ok())
//    ros::Duration(0.05).sleep();
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Start playback");
  else
    ROS_INFO("Some error occurred");

  return 0;
}
