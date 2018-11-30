#!/usr/bin/env python
#
# Copyright (c) 2018, Adam Allevato
# Copyright (c) 2018, The University of Texas at Austin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Pose

import transformations as tft
from hlpr_manipulation_utils.manipulator import Gripper
from arm_moveit2 import ArmMoveIt

class PickPlace(object):
    """
    TODO:
     - turn gripper 90 degrees
     - add offset specify on grasp
     - wrap offset with cup grasp and block grasp
    """

    def __init__(self, arm, vision_fudge_factor=(0, 0, 0), approach_height=0.07, place_height=0.01, lift_height=0.15):
        self.arm = arm
        self.gripper = Gripper()
        self.vision_fudge_factor = vision_fudge_factor
        self.approach_height = approach_height
        self.place_height = place_height
        self.lift_height = lift_height
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.is_holding = False
        self.next_place_height = None

    def publish_pose_as_tf(self, target):
        self.broadcaster.sendTransform((target.position.x, target.position.y, target.position.z),
            (target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w),
            rospy.Time(0),
            "target",
            self.arm.group[0].get_pose_reference_frame())

    def pick(self, frame_name):
        assert self.arm, frame_name

        if self.is_holding:
            print("already holding an object, cannot pick")
            return

        # look up frame_name
        print("looking up frame name")
        planning_frame = self.arm.group[0].get_pose_reference_frame()
        try:
            (trans,rot) = self.listener.lookupTransform(planning_frame, frame_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("error looking up transform, cannot complete pick command:")
            print(e)
            return

        trans = np.array(trans) + np.array(self.vision_fudge_factor)

        # open gripper
        print("opening gripper")
        self.gripper.open()
        rospy.sleep(2.0)

        # move over frame
        print("approaching frame")
        target = Pose()
        target.position.x = trans[0]
        target.position.y = trans[1]
        target.position.z = trans[2] + self.approach_height
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
            tft.quaternion_from_euler(0.0, math.pi/2, math.pi)

        self.publish_pose_as_tf(target)

        self.arm.move_to_ee_pose(target)

        # move down
        print("moving to frame")
        target.position.z = trans[2]
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        # close gripper
        print("closing gripper")
        self.gripper.close()
        rospy.sleep(2.0)

        # move up
        print("moving up")
        target.position.z = trans[2] + self.lift_height
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        self.is_holding = True
        self.next_place_height = trans[2] + self.place_height
        print("done picking")

    def place(self):
        assert self.arm

        if not self.is_holding:
            print("not currently holding anything, cannot place.")
            return

        start_pose = self.arm.group[0].get_current_pose()

        while True:
            try:
                self.listener.waitForTransform(
                    self.arm.group[0].get_pose_reference_frame(),
                    start_pose.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                    )
                start_pose = self.listener.transformPose(self.arm.group[0].get_pose_reference_frame(), start_pose)
                break
            except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                rospy.loginfo_throttle(1.0, "waiting for transform lookup...")

        target = Pose()
        target.position.x = start_pose.pose.position.x
        target.position.y = start_pose.pose.position.y
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
            tft.quaternion_from_euler(0.0, math.pi/2, math.pi)

        # move down
        print("moving to frame")
        target.position.z = self.next_place_height + self.place_height
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        # open gripper
        print("opening gripper")
        self.gripper.open()
        rospy.sleep(2.0)

        # move up
        print("moving up")
        target.position.z = self.next_place_height + self.lift_height + self.place_height
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        self.is_holding = False
        self.next_place_height = None
        print("done placing")



def main():
    rospy.init_node("pick_place_example", anonymous=True)

    arm = ArmMoveIt(planning_frame="j2s7s300_link_base")
    pick_place = PickPlace(arm, vision_fudge_factor=(-0.01, 0.0, -0.02))
    rospy.sleep(0.5)

    # pick_place.pick("obj_green_0")
    # pick_place.place()
    keep_going = True
    while keep_going:
        print("===")
        print("Enter a command of type <command> [frame]")
        print("Examples:")
        print("  pick obj_green_0")
        print("  place")
        print("  quit")
        command = raw_input().lower().strip()
        if command == "quit":
            keep_going = False
            continue
        else:
            tokens = command.split(" ")
            if tokens[0] == "pick":
                if len(tokens) != 2:
                    print("you must provide a single frame name to pick")
                    continue
                frame_name = tokens[1]
                pick_place.pick(frame_name)
            elif tokens[0] == "place":
                pick_place.place()
            else:
                print("Not a known command. Try again.")

if __name__ == "__main__":
    main()