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
import yaml
import tf
import sys
import threading
from hlpr_manipulation_utils.srv import FreezeFrame, FreezeFrameRequest, FreezeFrameResponse

from std_srvs.srv import Empty, EmptyResponse

class TFFreezeFrame(object):
    def __init__(self, frames, fixed_frame="base_link"):
        self.frames = {}
        self.fixed = fixed_frame
        
        for frame, subframes in frames.items():
            if frame in subframes:
                logerr("Can't remap frame {} to same name! Skipping.".format(frame))
            else:
                self.frames[frame]=subframes

        self.transforms = dict([(f,(None, None)) for f in self.frames.keys()])
        self.frozen = dict([(f,False) for f in self.frames.keys()])
                
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()


        self.monitor_thread = threading.Thread(target=self.monitor_tfs)
        self.publish_thread = threading.Thread(target=self.publish_tfs)

        self.monitor_thread.start()
        rospy.sleep(2)
        self.publish_thread.start()      
        self.server = rospy.Service("freeze_frames", FreezeFrame, self.handle_srv)

    def handle_srv(self, req):
        if req.action == FreezeFrameRequest.TOGGLE:
            if all(self.frozen.values()):
                self.unfreeze_all()
            else:
                self.freeze_all()
        elif req.action == FreezeFrameRequest.FREEZE:
            self.freeze_all()
        elif req.action == FreezeFrameRequest.UNFREEZE:
            self.unfreeze_all()
        else:
            rospy.logerr("unknown freeze frame action {}".format(req.action))
        return FreezeFrameResponse()
        
    def freeze(self, frames):
        for f in frames:
            self.frozen[f] = True

    def unfreeze(self, frames):
        for f in frames:
            self.frozen[f] = False

    def freeze_all(self):
        self.freeze(self.frames.keys())

    def unfreeze_all(self):
        self.unfreeze(self.frames.keys())

    def monitor_tfs(self):
        while not rospy.is_shutdown():
            for outframe, inframes in self.frames.items():
                if self.frozen[outframe]:
                    continue
                trans = None
                rot = None
                for inframe in inframes:
                    try:
                        trans, rot = self.listener.lookupTransform(self.fixed, inframe, rospy.Time())
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logwarn_throttle(1,"FreezeFrame Error: {}".format(e))
                        continue
                    break
                self.transforms[outframe]=(trans,rot)
        
    def publish_tfs(self):
        while not rospy.is_shutdown():
            for outframe, inframes in self.frames.items():
                trans,rot = self.transforms[outframe]
                if trans is None or rot is None:
                    rospy.logwarn_throttle(1,"Couldn't find transform for tf {}; won't republish.".format(outframe))
                else:
                    self.broadcaster.sendTransform(trans, rot, rospy.Time.now(), outframe, self.fixed)

if __name__=="__main__":
    rospy.init_node("tf_freeze_frame")
    frames = yaml.load(sys.argv[1])

    if len(sys.argv) > 2:
        # we have a base link provided
        tff = TFFreezeFrame(frames, sys.argv[2])
    else:
        tff = TFFreezeFrame(frames)
    rospy.spin()
