#!/usr/bin/env python

# Copyright (c) 2016, RT Corporation
# All rights reserved.
#
# License: BSD
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the RT Corporation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from dynamic_reconfigure.server import Server

from dynamic_reconfigure.client import Client

from nekonote_driver.cfg import (
    PositionJointTrajectoryActionServerConfig,
)
from follow_joint_trajectory_action.follow_joint_trajectory_action import (
    FollowJointTrajectoryActionServer,
)

def main():
    rospy.init_node("follow_joint_trajectory_action_server")
    rospy.loginfo("%s started. Ctrl-c to stop." % rospy.get_name())

    # Get ROS parameters
    joint_names = rospy.get_param('~joint_names', ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
    ns = rospy.get_param('~namespace_of_topics', 'robot')

    dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
            lambda config, level: config)

    # Reflect ROS parameters to dynamic reconfigure server
    client = Client(rospy.get_name())
    client.update_configuration({('name_of_joint' + str(idx)):joint_name
        for idx, joint_name in enumerate(joint_names)})

    fjtas = FollowJointTrajectoryActionServer(joint_names, ns, dyn_cfg_srv)

    def cleanup():
        fjtas.clean_shutdown()

    rospy.on_shutdown(cleanup)
    rospy.spin()

if __name__ == "__main__":
    main()
