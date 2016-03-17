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
#include "nekonote_driver/nekonote_driver_for_ROS.h"

using rt_nekonote::NekonoteDriverForROS;
using rt_nekonote::InterruptException;
using rt_nekonote::NekonoteException;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nekonote_driver");
  ROS_INFO("%s started. Ctrl-c to stop.", ros::this_node::getName().c_str());

  // Set default value to parameters
  std::string addr("192.168.1.54");
  int port = 1023;
  std::vector<std::string> joint_names;
  joint_names.push_back("joint0");
  joint_names.push_back("joint1");
  joint_names.push_back("joint2");
  joint_names.push_back("joint3");
  joint_names.push_back("joint4");
  joint_names.push_back("joint5");
  std::string ns("robot");

  // Get ROS parameters
  ros::param::get("~address", addr);
  ros::param::get("~port_number", port);
  ros::param::get("~joint_names", joint_names);
  ros::param::get("~namespace_of_topics", ns);

  NekonoteDriverForROS nekonote_ROS(addr, port, joint_names, ns);
  try
  {
    nekonote_ROS.start();
    ros::spin();
  }
  catch (const NekonoteException& err)
  {
    ROS_ERROR("In %s(): %s", err.where(), err.what());
  }
  catch (const InterruptException& err)
  {
  }
  std::cout << "Waiting for the communication thread to terminate..." << std::endl;
  nekonote_ROS.threadJoin();

  return 0;
}
