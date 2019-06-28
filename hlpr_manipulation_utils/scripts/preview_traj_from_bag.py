#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
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
# * Neither the name of the SIM Lab nor the names of its
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

import rospy
import os
from hlpr_kinesthetic_teaching_api.kinesthetic_teaching_api import KTInterface
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_kinesthetic_teaching_api.msg import KTPlanDisplay
from hlpr_manipulation_utils.manipulator import Gripper

if __name__=="__main__":
    rospy.init_node("eef_kf_to_joint_kf")
    
    a = ArmMoveIt(planning_frame="base_link")
    #a.group[0].set_end_effector_link("j2s7s300_ee_link")

    pb = rospy.Publisher("/KT/display_traj", KTPlanDisplay)
    
    k = KTInterface("~/research/poli2_segway_ws/src/seal/test/data",ArmMoveIt(), Gripper())
    
    k.load_bagfile("~/research/poli2_segway_ws/src/seal/test/data/eeftest.bag",
                    load_joints=False)
    
    #k.vis_plan() #this is how you would do it on the robot
    
    eefs = []
    
    for s in k.segments:
        eefs.append(a.get_IK(s.end))
        
    
    
    plans = a.plan_joint_waypoints(eefs)
    
    pb.publish(KTPlanDisplay(plans=plans))

