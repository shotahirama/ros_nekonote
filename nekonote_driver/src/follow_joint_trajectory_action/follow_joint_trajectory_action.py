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

from copy import deepcopy
import math
import operator

import rospy

import actionlib

from std_msgs.msg import (
    Empty,
    Int64,
)

from sensor_msgs.msg import (
    JointState,
)

from actionlib_msgs.msg import (
    GoalStatus,
)

from nekonote_msgs.msg import (
    JointTrajectoryAction,
    JointTrajectoryGoal,
    JointTrajectoryFeedback,
)

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)


class FollowJointTrajectoryActionServer(object):
    def __init__(self, joint_names, ns, reconfig_server):
        self._joint_names = joint_names
        self._ns = ns
        self._dyn = reconfig_server
        self._fjta_ns = self._ns + '/follow_joint_trajectory_action'
        self._jta_ns = self._ns + '/joint_trajectory_action'

        # ROS action
        self._server = actionlib.SimpleActionServer(
                self._fjta_ns,
                FollowJointTrajectoryAction,
                execute_cb=self._on_follow_joint_trajectory_action,
                auto_start=False)
        self._client = actionlib.SimpleActionClient(
                self._jta_ns,
                JointTrajectoryAction)
        self._node_name = rospy.get_name()
        rospy.loginfo("%s: Waiting for joint trajectory action server to start..." % self._node_name)
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10))
        if not server_up:
            rospy.logerr("%s: Timed out waiting for joint trajectory"
                         " action server to connect. Start that server"
                         " before running this node" % self._node_name)
            rospy.signal_shutdown("Timed out waiting for joint trajectory action server")
            return
        rospy.loginfo("%s: Confirmed joint trajectory action server started" % self._node_name)

        # Subscribed result
        self._joint_angle = dict()
        self._last_motion_id = 0

        # ROS publisher
        self._pub_flush_motion = rospy.Publisher(
            self._ns + '/flush_motion',
            Empty,
            queue_size=1)

        # ROS subscriber
        _joint_state_sub = rospy.Subscriber(
            self._ns + '/joint_states',
            JointState,
            self._on_joint_states,
            queue_size=1)
        _last_motion_id_sub = rospy.Subscriber(
            self._ns + '/last_motion_id',
            Int64,
            self._on_last_motion_id,
            queue_size=1)
        rospy.loginfo("%s: Waiting to get current joint_states..." % self._node_name)
        rate = rospy.Rate(100)
        end_time = rospy.get_time() + 10.0
        while (len(self._joint_angle.keys()) != len(self._joint_names)) and (not rospy.is_shutdown()):
            if rospy.get_time() >= end_time:
                rospy.logerr("%s: Timed out waiting to get current joint_states" % self._node_name)
                rospy.signal_shutdown("Timed out waiting to get current joint_states")
                return
            rate.sleep()
        if rospy.is_shutdown():
            return
        rospy.loginfo("%s: Got current joint_states." % self._node_name)
        self._server.start()
        self._alive = True

        # Action Goal/Feedback/Result
        self._jta_goal = JointTrajectoryGoal()
        self._fjta_feedback = FollowJointTrajectoryFeedback()
        self._fjta_result = FollowJointTrajectoryResult()

        self._motion_ids = []

        self._clear()

        # Controller parameters from dynamic reconfigure
        self._goal_time = rospy.Duration(0, 0)
        self._goal_error = dict()
        self._path_thresh = dict()

        rospy.loginfo("%s: follow joint trajectory action server started" % self._node_name)

    def clean_shutdown(self):
        print("%s: Shutting down..." % self._node_name)
        self._alive = False

    def _clear(self):
        self._jta_goal = JointTrajectoryGoal()
        self._jta_goal.trajectory.joint_names = self._joint_names
        self._fjta_feedback = FollowJointTrajectoryFeedback()
        self._fjta_result = FollowJointTrajectoryResult()
        self._motion_ids = []

    def _on_joint_states(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx]

    def _on_last_motion_id(self, msg):
        self._last_motion_id = msg.data

    def _on_jta_feedback(self, feedback):
        self._motion_ids = feedback.motion_id

    def _get_trajectory_parameters(self, joint_names, goal):
        # For each input trajectory, if path, goal, or goal_time tolerances
        # provided, we will use these as opposed to reading from the
        # parameter server/dynamic reconfigure

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance
        else:
            self._goal_time = rospy.Duration.from_sec(self._dyn.config['goal_time'])

        # Path execution and goal tolerances per joint
        for jnt in joint_names:
            if jnt not in self._joint_names:
                error_str = "Provided Invalid Joint Name " + jnt
                rospy.logerr(
                    "%s: Trajectory Aborted - %s" %
                    (self._node_name, error_str,))
                self._fjta_result.error_code = self._fjta_result.INVALID_JOINTS
                self._fjta_result.error_string = error_str
                self._server.set_aborted(self._fjta_result)
                return False
            idx = self._joint_names.index(jnt)
            # Path execution tolerance
            path_error = self._dyn.config['joint' + str(idx) + '_trajectory']
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._path_thresh[jnt] = tolerance.position
                        else:
                            self._path_thresh[jnt] = path_error
            else:
                self._path_thresh[jnt] = path_error
            # Goal error tolerance
            goal_error = self._dyn.config['joint' + str(idx) + '_goal']
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._goal_error[jnt] = tolerance.position
                        else:
                            self._goal_error[jnt] = goal_error
            else:
                self._goal_error[jnt] = goal_error
        return True

    def _command_flush(self):
        cmd = Empty()
        self._pub_flush_motion.publish(cmd)

    def _get_current_position(self, joint_names):
        return [self._joint_angle[jnt] for jnt in joint_names]

    def _get_error(self, desired, actual):
        return map(operator.sub, desired, actual)

    def _update_fjta_feedback(self, joint_names, desired, actual, error, now_from_start):
        self._fjta_feedback.header.stamp = rospy.get_rostime()
        self._fjta_feedback.joint_names = joint_names
        self._fjta_feedback.desired.positions = desired
        self._fjta_feedback.desired.time_from_start = now_from_start 
        self._fjta_feedback.actual.positions = actual 
        self._fjta_feedback.actual.time_from_start = now_from_start 
        self._fjta_feedback.error.positions = error 
        self._fjta_feedback.error.time_from_start = now_from_start 
        self._server.publish_feedback(self._fjta_feedback)
    
    def _check_real_trajectory(self, joint_names, trajectory_points, start_time):
        motion_ids = list(self._motion_ids)
        rate = rospy.Rate(1000)
        while (len(trajectory_points) > 0) and (not rospy.is_shutdown()):
            if rospy.get_rostime() > start_time + trajectory_points[-1].time_from_start + self._goal_time:
                err_str = "Violated goal_time_tolerance"
                rospy.logerr("%s: %s" % (self._node_name, err_str,))
                self._fjta_result.error_code = self._fjta_result.GOAL_TOLERANCE_VIOLATED
                self._fjta_result.error_string = err_str
                self._server.set_aborted(self._fjta_result)
                self._command_flush()
                self._clear()
                rospy.sleep(3)
                return False
            if self._last_motion_id in motion_ids:
                last = motion_ids.index(self._last_motion_id) + 1
                target_ids = motion_ids[:last]
                for target_id in target_ids:
                    now_from_start = rospy.get_rostime() - start_time
                    desired = deepcopy(trajectory_points[0].positions)
                    actual = self._get_current_position(joint_names)
                    error = self._get_error(desired, actual)
                    for idx, err in enumerate(error):
                        jnt = joint_names[idx]
                        if len(motion_ids) > 1:
                            thresh = self._path_thresh[jnt]
                            err_code = self._fjta_result.PATH_TOLERANCE_VIOLATED
                            err_str = "Exceeded Path Error Threshold on " + jnt + ": " + str(err)
                        else:
                            thresh = self._goal_error[jnt]
                            err_code = self._fjta_result.GOAL_TOLERANCE_VIOLATED
                            err_str = "Exceeded Goal Error Threshold on " + jnt + ": " + str(err)
                        if math.fabs(err) > thresh and thresh > 0.0:
                            rospy.logerr("%s: %s" % (self._node_name, err_str,))
                            self._fjta_result.error_code = err_code
                            self._fjta_result.error_string = err_str
                            self._server.set_aborted(self._fjta_result)
                            self._command_flush()
                            self._clear()
                            rospy.sleep(3)
                            return False
                    self._update_fjta_feedback(joint_names, desired, actual, error, now_from_start)
                    motion_ids.pop(0)
                    trajectory_points.pop(0)
            rate.sleep()
        return True

    def _on_follow_joint_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        # Load parameters for trajectory
        if not self._get_trajectory_parameters(joint_names, goal):
            return

        self._jta_goal.trajectory.header.stamp = goal.trajectory.header.stamp
        self._jta_goal.trajectory.joint_names = joint_names
        self._jta_goal.trajectory.points = trajectory_points
        rospy.loginfo("%s: Executing requested joint trajectory..." % self._node_name)
        self._client.send_goal(
            self._jta_goal,
            feedback_cb=self._on_jta_feedback)
        self._client.wait_for_result()
        start_time = rospy.get_rostime()
        if self._client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("%s: Error occurred on %s. Aborted" %
                (self._node_name, self._jta_ns,))
            result = self._client.get_result()
            self._fjta_result.error_code = result.error_code
            self._fjta_result.error_string = result.error_string
            self._server.set_aborted(self._fjta_result)
            self._clear()
            rospy.sleep(3)
            return
        if not len(self._motion_ids) > 0:
            rospy.logerr("%s: Could not get feedback from %s. Aborted" %
                (self._node_name, self._jta_ns,))
            self._server.set_aborted()
            self._command_flush()
            self._clear()
            rospy.sleep(3)
            return

        if not self._check_real_trajectory(joint_names, list(trajectory_points), start_time):
            return

        rospy.loginfo("%s: Joint Trajectory Action Succeeded" % self._node_name)
        self._fjta_result.error_code = self._fjta_result.SUCCESSFUL
        self._server.set_succeeded(self._fjta_result)
        self._clear()
