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
    def __init__(self,
        require_place=True,
        planning_frame="j2s7s300_link_base",
        vision_fudge_factor=(0, 0, 0),
        approach_height=0.07,
        place_height=0.01,
        lift_height=0.15,
        rim_offset=0.02):
        """
        require_place (bool): If true, this class will not allow two consecutive pick calls. You must
            place between each pick.
        planning_frame: The MoveIt frame to use for planning motions.
        vision_fudge_factor (list of 3 floats): Add this offset (in the planning frame) to the
            position of every manipulation object's frame.
        approach_height: How far above the object to move when approaching. The first move
            of a pick is the approach, where the EEF positions itself over the object.
        place_height: How far above the pick point to move when placing. i.e., if 0,
            the robot will move to the exact height it used for picking when placing. This
            should always be >= 0 and larger values are preferred for safety reasons, to
            avoid smashing into the table.
        lift_height: How high to lift picked objects. After grasping an object, the robot
            lifts it. You may want this to be different from the approach height.
        rim_offset (float): how far to offset the gripper position when attempting to pick an object up 
            by its rim (rather than a direct center grasp)
        """
        self.arm = ArmMoveIt(planning_frame)
        self.gripper = Gripper()
        self.vision_fudge_factor = vision_fudge_factor
        self.approach_height = approach_height
        self.place_height = place_height
        self.lift_height = lift_height
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.is_holding = False
        self.next_place_height = None
        self.rim_offset = rim_offset
        self.require_place = require_place

    def publish_pose_as_tf(self, target):
        self.broadcaster.sendTransform((target.position.x, target.position.y, target.position.z),
            (target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w),
            rospy.Time(0),
            "target",
            self.arm.group[0].get_pose_reference_frame())

    def pick_rim(self, frame_name):
        assert frame_name, type(frame_name) == str

        if self.is_holding and self.require_place:
            print("already holding an object, cannot pick")
            return

        # look up frame_name
        print("looking up frame name")
        planning_frame = self.arm.group[0].get_pose_reference_frame()
        try:
            self.listener.waitForTransform(
                planning_frame,
                frame_name,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            (trans,rot) = self.listener.lookupTransform(planning_frame, frame_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("error looking up transform, cannot complete pick command:")
            print(e)
            return

        # this is what makes it a rim pick: offset gripper from center
        trans[1] += self.rim_offset

        return self.pick(trans)

    def pick_center(self, frame_name, fixed_height=None):
        assert frame_name, type(frame_name) == str

        if self.is_holding and self.require_place:
            print("already holding an object, cannot pick")
            return

        # look up frame_name
        print("looking up frame name")
        planning_frame = self.arm.group[0].get_pose_reference_frame()
        try:
            self.listener.waitForTransform(
                planning_frame,
                frame_name,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            (trans,rot) = self.listener.lookupTransform(planning_frame, frame_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("error looking up transform, cannot complete pick command:")
            print(e)
            return
        if fixed_height:
            trans[2] = fixed_height
        return self.pick(trans)

    def pick(self, target_position):
        assert self.arm

        target_position = np.array(target_position) + np.array(self.vision_fudge_factor)

        # open gripper
        self.gripper.open(block=True)

        # move over frame
        print("approaching frame")
        target = Pose()
        target.position.x = target_position[0]
        target.position.y = target_position[1]
        target.position.z = target_position[2] + self.approach_height
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
            tft.quaternion_from_euler(0.0, math.pi/2, math.pi/2)

        self.publish_pose_as_tf(target)

        self.arm.move_to_ee_pose(target)

        # move down
        print("moving to frame")
        target.position.z = target_position[2]
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        # close gripper
        print("closing gripper")
        self.gripper.close(block=True)

        # move up
        print("moving up")
        target.position.z = target_position[2] + self.lift_height
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        self.is_holding = True
        self.next_place_height = target_position[2] + self.place_height
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
                # print(e)
                rospy.loginfo_throttle(1.0, "waiting for transform lookup...")

        target = Pose()
        target.position.x = start_pose.pose.position.x
        target.position.y = start_pose.pose.position.y
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
            tft.quaternion_from_euler(0.0, math.pi/2, math.pi/2)

        # move down
        print("moving to frame")
        target.position.z = self.next_place_height + self.place_height
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        # open gripper
        self.gripper.open(block=True)

        # move up
        print("moving up")
        target.position.z = self.next_place_height + self.lift_height + self.place_height
        self.publish_pose_as_tf(target)
        self.arm.move_to_ee_pose(target)

        self.is_holding = False
        self.next_place_height = None
        print("done placing")



def main():
    rospy.init_node("pick_place_example", anonymous=True, disable_signals=True)

    pick_place = PickPlace(vision_fudge_factor=(-0.01, 0.0, -0.02))
    rospy.sleep(0.5)

    # pick_place.pick("obj_green_0")
    # pick_place.place()
    keep_going = True
    while keep_going:
        print("===")
        print("Enter a command of type <command> [frame]")
        print("Examples:")
        print("  pick_center obj_green_0")
        print("  pick_rim obj_green_0")
        print("  place")
        print("  quit")
        try:
            command = raw_input().lower().strip()
        except KeyboardInterrupt as e:
            command = "quit"

        if command == "quit":
            keep_going = False
            continue
        else:
            tokens = command.split(" ")
            if tokens[0] == "pick_center":
                if len(tokens) != 2:
                    print("you must provide a single frame name to pick_center")
                    continue
                frame_name = tokens[1]
                pick_place.pick_center(frame_name)
            if tokens[0] == "pick_rim":
                if len(tokens) != 2:
                    print("you must provide a single frame name to pick_rim")
                    continue
                frame_name = tokens[1]
                pick_place.pick_rim(frame_name)
            elif tokens[0] == "place":
                pick_place.place()
            else:
                print("Not a known command. Try again.")
    rospy.signal_shutdown("Program completed.")

if __name__ == "__main__":
    main()