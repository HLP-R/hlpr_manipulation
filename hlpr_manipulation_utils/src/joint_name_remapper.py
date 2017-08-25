#!/usr/bin/env python
import rospy
import sys
from copy import deepcopy
from sensor_msgs.msg import JointState

class JointNameRemapper():
    '''
    This Node remaps joint names specified by the Kinova-ros driver 'eg. j2n7s300_joint_1' to arm sepcific joints names
    '''
    
    def __init__(self, prefix):
        rospy.loginfo("Setting up subscribers and publishers")
        
        self._prefix = prefix
        self._kinova_gripper = False
        self._n_joints = 7
        
        self.joint_pub = rospy.Publisher('/vector/' + self._prefix + '_arm/joint_states', JointState , queue_size=10)
        
        rospy.loginfo("Done Init")


    def remapCallback(self, msg):
        new_msg = deepcopy(msg)
        new_names = []
        new_msg.name = [] 

        for i in range(self._n_joints):
            new_names += [self._prefix + "_joint_" + str(i+1)]

        if(self._kinova_gripper):
            for i in range(3):
                new_names += [self._prefix + "_arm_joint_finger_ " + str(i+1)]

        new_msg.name = new_names
        self.joint_pub.publish(new_msg)

    def remap(self):
        joint_sub = rospy.Subscriber('/' + self._prefix + '_arm_driver/out/joint_state', JointState, self.remapCallback, queue_size=1)
        rospy.spin()


if __name__=='__main__':
    rospy.init_node('jointNameRemapper')
    rospy.loginfo("Starting up joint name remapper Node")
    # prefix = rospy.get_param('~side')
    prefix = sys.argv[1]
    remapper = JointNameRemapper(prefix)
    remapper.remap()
