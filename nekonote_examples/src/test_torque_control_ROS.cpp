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

#include "sensor_msgs/JointState.h"
#include "nekonote_msgs/JointCommand.h"

const double KP = 20;
const double KD = 2;

bool is_updated = false;
std::vector<std::string> joint_names;
std::vector<double> joint_angles;
std::vector<double> joint_speed;

void getJointStates(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  if (joint_states->name == joint_names)
  {
    joint_angles = joint_states->position;
    joint_speed = joint_states->velocity;
    is_updated = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_torque_control");

  ros::NodeHandle nh;

  // ROS publisher
  ros::Publisher joint_cmd_pub = nh.advertise<nekonote_msgs::JointCommand>("robot/joint_command", 10);

  // ROS subscriber
  ros::Subscriber joint_states_sub = nh.subscribe("robot/joint_states", 100, getJointStates);

  ROS_INFO("Getting ROS parameter...");
  ros::Rate loop_rate(1000);
  if (ros::param::get("nekonote_driver/joint_names", joint_names))
    ROS_INFO("Got ROS parameter");
  else
    ROS_ERROR("Cannot get ROS parameter");

  ROS_INFO("Waiting to subscribe /robot/joint_states...");
  while (!is_updated)
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (!ros::ok())
      return 0;
  }
  ROS_INFO("Subscribed /robot/joint_states. Now NekonoteDriver certainly connected with NEKONOTE.");

  ROS_INFO("PD controller is started. Now joint 5 is fixed to 0 rad");
  while (ros::ok())
  {
    std::vector<double> command(6, 0);
    command[5] = -KP * joint_angles[5] - KD * joint_speed[5];
    nekonote_msgs::JointCommand joint_command;
    joint_command.mode = nekonote_msgs::JointCommand::TORQUE_CONTROL;
    joint_command.name = joint_names;
    joint_command.command = command;
    joint_cmd_pub.publish(joint_command);
    is_updated = false;
    while (!is_updated)
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (!ros::ok())
        return 0;
    }
  }

  return 0;
}
