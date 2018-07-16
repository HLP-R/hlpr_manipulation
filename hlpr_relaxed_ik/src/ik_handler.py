#!/usr/bin/env python

from RelaxedIK.relaxedIK import RelaxedIK
from hlpr_relaxed_ik.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import numpy as np
import rospy

fixed_frame = None
joint_ordering = None
relaxed_ik = None

def filter_joint_state(names, js):
    n_idx = []
    js_name = np.array(js.name)
    for name in names:
        n_idx.append(np.where(js_name == name)[0][0])
    
    js_f = JointState()
    js_f.header = js.header

    for i in n_idx:
        js_f.name.append(js.name[i])

        if i < len(js.position):
            js_f.position.append(js.position[i])

        if i < len(js.velocity):
            js_f.velocity.append(js.velocity[i])

        if i < len(js.effort):
            js_f.effort.append(js.effort[i])

    return js_f

def get_pose(req):
    start_joint_state = filter_joint_state(joint_ordering, req.start).position
    relaxed_ik.reset(start_joint_state)

    start_t = req.eef_traj[0].header.stamp
    joint_traj = JointTrajectory()
    joint_traj.joint_names = joint_ordering

    for ps in req.eef_traj:
        t = ps.header.stamp
        p = ps.pose.position
        q = ps.pose.orientation

        goal_p = [[p.x, p.y, p.z]]
        goal_q = [[q.x, q.y, q.z, q.w]]
        xopt = relaxed_ik.solve(goal_p, goal_q)

        joint_traj_point = JointTrajectoryPoint()
        joint_traj_point.positions = xopt
        joint_traj_point.time_from_start = t - start_t

        joint_traj.points.append(joint_traj_point)

    return IKHandlerResponse(joint_traj, True)

def ik_handler_server():
    rospy.init_node('hlpr_relaxed_ik_server')

    global fixed_frame, joint_ordering, relaxed_ik

    fixed_frame = rospy.get_param('~fixed_frame')
    joint_ordering = rospy.get_param('~joint_ordering')

    config_file = rospy.get_param('~config_file')
    relaxed_ik = RelaxedIK.init_from_config(config_file)

    service = rospy.Service('hlpr_relaxed_ik', IKHandler, get_pose)
    rospy.spin()

if __name__ == '__main__':
    ik_handler_server()

