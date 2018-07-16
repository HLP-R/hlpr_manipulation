#!/usr/bin/env python

from RelaxedIK.relaxedIK import RelaxedIK
from hlpr_relaxed_ik.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import rospy

relaxed_ik = None

def get_pose(req):
    start_joint_state = req.start.position
    relaxed_ik.reset(start_joint_state)

    start_t = req.eef_traj[0].header.stamp
    joint_traj = JointTrajectory()

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

    config_file = rospy.get_param('~config_file')

    global relaxed_ik
    relaxed_ik = RelaxedIK.init_from_config(config_file)

    service = rospy.Service('hlpr_relaxed_ik', IKHandler, get_pose)
    rospy.spin()

if __name__ == '__main__':
    ik_handler_server()

