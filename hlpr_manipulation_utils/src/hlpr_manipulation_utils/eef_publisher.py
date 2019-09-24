#!/usr/bin/env python

# Copyright (c) 2016, Diligent Droids
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
# * Neither the name of hlpr_kinesthetic_teaching nor the names of its
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

# Author: Andrea Thomaz, athomaz@diligentdroids.com

import rospy
import tf 
import time
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState


def eef_pose_pub():
  rospy.init_node('eef_publisher')
  listener = tf.TransformListener()
  pub = rospy.Publisher('eef_pose', Pose, queue_size=10)

  if os.environ.get("ROBOT_NAME") == "poli2":
    DEFAULT_EEF_LINK = '/j2s7s300_ee_link'
  else:
    DEFAULT_EEF_LINK = '/right_ee_link'
  DEFAULT_BASE_LINK = '/base_link'
  DEFAULT_RATE = 100

  # Pull from param server the hz and EEF link
  eef_link = rospy.get_param("~eef_link", DEFAULT_EEF_LINK)
  publish_rate = rospy.get_param("~eef_rate", DEFAULT_RATE)
  base_link = rospy.get_param("~base_link", DEFAULT_BASE_LINK)

  rospy.loginfo("Publishing the link {} as the robot eef link".format(eef_link))

  rospy.sleep(0.5)
  rate = rospy.Rate(publish_rate)
  while not rospy.is_shutdown():
    try: 
      trans, rot = listener.lookupTransform(base_link, eef_link, rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      rospy.logwarn_throttle(30, "eef publisher error (printed every 30s): " + str(e))
      continue

    # print("position: {} {} {}".format(*trans))

    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = trans[0], trans[1], trans[2]
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = rot[0], rot[1], rot[2], rot[3]
    pub.publish(msg)
    rate.sleep()

if __name__ =='__main__':
    eef_pose_pub()
