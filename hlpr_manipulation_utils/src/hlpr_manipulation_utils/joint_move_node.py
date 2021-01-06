#! /usr/bin/env python
# Copyright (c) 2020 SIM Lab, The University of Texas at Austin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This class defines a node that moves

import argparse
import sys
import math

import rospy
import actionlib

import kinova_msgs.msg

def fill_with_backup(desired, backup):
    if desired is None:
        return backup
    return desired

def degrees_or_none(rad):
    if rad is None:
        return None
    return math.degrees(rad)

# def lookup_joint(msg, i):
#     assert i >= 0 and i < self.arm_joint_number
#     if i == 0:
#         return msg.joint1
#     elif i == 1:
#         return msg.joint2
#     elif i == 2:
#         return msg.joint3
#     elif i == 3:
#         return msg.joint4
#     elif i == 4:
#         return msg.joint5
#     elif i == 5:
#         return msg.joint6
#     elif i == 6:
#         return msg.joint7

class JointMoveNode(object):
    current_joint_pos = None
    arm_name = None

    def __init__(self, arm_name=None, arm_joint_number=7):
        if arm_name is None:
            arm_name = "j2s7s300"
        self.arm_name = arm_name
        self.arm_joint_number = arm_joint_number
        topic_address = '/' + self.arm_name + '_driver/out/joint_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, self.cb_currentJointCommand)

        action_address = '/' + self.arm_name + '_driver/joints_action/joint_angles'
        self.client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.ArmJointAnglesAction)
        rospy.loginfo("Waiting for joint angle action server to connect...")
        self.client.wait_for_server()
        rospy.loginfo("Joint angle action server connected.")

        rospy.loginfo("Listening for joint move requests")

    def send_joint_angles(self, angles, relative=True, unit='degree', verbose=False):
        """Send a joint angle goal to the action server."""

        # if degrees, convert to list of None and radians
        if unit == 'radian':
            angles_rad = [degrees_or_none(r) for r in angles]
        else:
            angles_rad = angles

        angles_deg = angles

        # if relative, convert to absolute
        if relative:
            absolute_angles_deg = [self.relative_to_absolute(r) for r in angles_deg]
        else:
            absolute_angles_deg = angles_deg

        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        if verbose:
            self.debugPrint(goal)

        # fill None with current position
        goal.angles.joint1 = fill_with_backup(absolute_angles_deg[0], self.current_joint_pos[0])
        goal.angles.joint2 = fill_with_backup(absolute_angles_deg[1], self.current_joint_pos[1])
        goal.angles.joint3 = fill_with_backup(absolute_angles_deg[2], self.current_joint_pos[2])
        goal.angles.joint4 = fill_with_backup(absolute_angles_deg[3], self.current_joint_pos[3])
        goal.angles.joint5 = fill_with_backup(absolute_angles_deg[4], self.current_joint_pos[4])
        goal.angles.joint6 = fill_with_backup(absolute_angles_deg[5], self.current_joint_pos[5])
        goal.angles.joint7 = fill_with_backup(absolute_angles_deg[6], self.current_joint_pos[6])

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(20.0)):
            return self.client.get_result()
        else:
            print('The joint angle action timed-out')
            self.client.cancel_all_goals()
            return None


    def relative_to_absolute(self, rad):
        angles[rad] + lookup_joint(self.current_joint_pos, i)

    def cb_currentJointCommand(self, joint_angles_msg):
        self.current_joint_pos = [
            joint_angles_msg.joint1,
            joint_angles_msg.joint2,
            joint_angles_msg.joint3,
            joint_angles_msg.joint4,
            joint_angles_msg.joint5,
            joint_angles_msg.joint6,
            joint_angles_msg.joint7]


    def unitParser(self, unit, joint_value, relative_):
        assert unit in ['degree', 'radian']
        if unit == 'degree':
            joint_degree_command = joint_value
            # get absolute value
            if relative_:
                joint_degree_absolute_ = [joint_degree_command[i] + self.current_joint_pos[i] for i in range(0, len(joint_value))]
            else:
                joint_degree_absolute_ = joint_degree_command
            joint_degree = joint_degree_absolute_
            joint_radian = list(map(math.radians, joint_degree_absolute_))
        elif unit == 'radian':
            joint_degree_command = list(map(math.degrees, joint_value))
            # get absolute value
            if relative_:
                joint_degree_absolute_ = [joint_degree_command[i] + self.current_joint_pos[i] for i in range(0, len(joint_value))]
            else:
                joint_degree_absolute_ = joint_degree_command
            joint_degree = joint_degree_absolute_
            joint_radian = list(map(math.radians, joint_degree_absolute_))

        return joint_degree, joint_radian


    def debugPrint(self, goal):
        print("sending absolute goal:\n" + str(goal))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='degree',
                        choices={'degree', 'radian'},
                        help='Unit of joiint motion command, in degree, radian')
    parser.add_argument('joint_value', nargs='*', type=float, help='joint values, length equals to number of joints.')
    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display joint values in alternative convention(degree or radian)')

    myargv = rospy.myargv(argv=sys.argv)
    # print(myargv)
    args = parser.parse_args(myargv[1:])

    rospy.init_node(self.arm_name + "_joint_move_controller")

    arm_name = rosparam.get_param("~arm_name")
    jmn = JointMoveNode(arm_name=arm_name)

    if len(args.joint_value) > 0:
        if len(args.joint_value) != self.arm_joint_number:
            rospy.logerr("You must provide {} values (number of arm joints)".format(self.arm_joint_number))
            sys.exit(0)

        if not args.relative:
            print("You are about to move to an absolute set of joint positions.")
            print("NO COLLISION CHECKING WILL OCCUR")
            raw_input("Are you sure you want to continue? Enter for yes, Ctrl-\ for no (program will exit). ")

        try:
            result = jmn.send_joint_angles(args.joint_value, args.relative, args.unit, args.verbose)
        except rospy.ROSInterruptException:
            print('program interrupted before completion')
    else:
        rospy.spin()
