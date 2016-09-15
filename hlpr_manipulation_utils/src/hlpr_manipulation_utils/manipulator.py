import roslib; roslib.load_manifest('hlpr_manipulation_utils')
from sensor_msgs.msg import JointState
from vector_msgs.msg import JacoCartesianVelocityCmd, LinearActuatorCmd, GripperCmd, GripperStat
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from wpi_jaco_msgs.msg import AngularCommand, CartesianCommand
#from wpi_jaco_msgs.srv import GravComp
from hlpr_manipulation_utils.arm_moveit import *

import rospy
from math import pi, sqrt
from collections import namedtuple
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import actionlib
import time

class Manipulator:
  def __init__(self, arm_prefix = 'right'):
    self.arm = Arm()
    self.gripper = Gripper()
    self.linear_actuator = LinearActuator()

class Gripper:
  def __init__(self, prefix='right'):
    self.pub_grp  = rospy.Publisher('/vector/'+prefix+'_gripper/cmd', GripperCmd, queue_size = 10)
    self.cmd = GripperCmd()
    
    #i have it here but it is not useful
    #rospy.Subscriber('/vector/right_gripper/joint_states', JointState, self.js_cb)
    #self.last_js_update = None
    #self.joint_state = None
    
    rospy.Subscriber('/vector/'+prefix+'_gripper/stat', GripperStat, self.st_cb)
    self.last_st_update = None
    self.gripper_stat = GripperStat()
    

  #def js_cb(self, inState):
  #  self.joint_state = inState.position  
  #  self.last_js_update = rospy.get_time()
    
  def st_cb(self, inStat):
    self.gripperStat = inStat
    self.last_st_update = None
    
  def is_ready(self):
    return self.gripperStat.is_ready
  
  def is_reset(self):
    return self.gripperStat.is_reset

  def is_moving(self):
    return self.gripperStat.is_moving
    
  def object_detected(self):
    return self.gripperStat.obj_detected
    
  def get_pos(self):
    return self.gripperStat.position
    
  def get_commanded_pos(self):
    return self.gripperStat.requested_position

  def get_applied_current(self):
    return self.gripperStat.current

  def set_pos(self, position, speed = 0.02, force = 100, rate = 10, iterations = 5):
    
    self.cmd.position = position
    self.cmd.speed = speed
    self.cmd.force = force
    rrate = rospy.Rate(rate)
    for i in range(0,iterations):
      self.pub_grp.publish(self.cmd)
      rrate.sleep()  
    
  def open(self, speed = 0.02, force = 100):
    self.set_pos(0.085,speed,force)
    
  def close(self, speed = 0.02, force = 100):
    self.set_pos(0,speed,force)

class LinearActuator:
  def __init__(self):
    self.pub_lin  = rospy.Publisher('/vector/linear_actuator_cmd', LinearActuatorCmd, queue_size = 10)
    self.cmd = LinearActuatorCmd()

    #i agree that the naming is weird
    rospy.Subscriber('/vector/joint_states', JointState, self.js_cb)
    self.last_js_update = None
    self.joint_state = None
    
  def js_cb(self, inState):
    self.joint_state = inState.position  
    self.last_js_update = rospy.get_time()


  def set_pos(self, position, vel = 0.):
    self.cmd = LinearActuatorCmd()
    self.cmd.desired_position_m = position
    if not vel == 0:
      print 'What are you thinking? Setting the vel back to 0. If you are sure, change this line in the code'
      vel = 0.

    #probably feed forward velocity
    self.cmd.fdfwd_vel_mps = vel

    self.pub_lin.publish(self.cmd) 

class Arm:
  def __init__(self, arm_prefix = 'right'):
    self.pub_jaco_ang  = rospy.Publisher('/jaco_arm/angular_cmd', AngularCommand, queue_size = 10, latch=True)
    self.pub_jaco_cart = rospy.Publisher('/jaco_arm/cartesian_cmd', CartesianCommand, queue_size = 10, latch=True)
    
    self._arm_prefix = arm_prefix
    self.arm_joint_names = [  self._arm_prefix + "_shoulder_pan_joint",   self._arm_prefix + "_shoulder_lift_joint",   self._arm_prefix + "_elbow_joint", 
                              self._arm_prefix + "_wrist_1_joint",   self._arm_prefix + "_wrist_2_joint",   self._arm_prefix + "_wrist_3_joint"]

    self.joint_states = [0 for i in range(0,len( self.arm_joint_names))]
    
    rospy.Subscriber('/vector/right_arm/joint_states', JointState, self.js_cb)
    self.last_js_update = None
    
    self.smooth_joint_trajectory_client = actionlib.SimpleActionClient('/jaco_arm/joint_velocity_controller/trajectory', FollowJointTrajectoryAction)
    
    #if(self.smooth_joint_trajectory_client.wait_for_server(rospy.Duration(5.0))):
    if(self.smooth_joint_trajectory_client.wait_for_server()):
      self.traj_connection = True
    else:
      self.traj_connection = False

    print self.traj_connection

    self.angular_cmd = AngularCommand()
    self.angular_cmd.armCommand = True
    self.angular_cmd.fingerCommand = False
    self.angular_cmd.repeat = True

    self.cartesian_cmd = CartesianCommand()
    self.cartesian_cmd.armCommand = True
    self.cartesian_cmd.fingerCommand = False
    self.cartesian_cmd.repeat = True
    
    self._init_tuck_poses()

#    if(rospy.wait_for_service('/jaco_arm/grav_comp')):
#      self.gc_connection = True
#    else:
    self.gc_connection = False    
#    self.grav_comp_client = rospy.ServiceProxy('/jaco_arm/grav_comp', GravComp)
 
    self.arm_planner = ArmMoveIt() 

  def _get_arm_joint_values(self, msg):

    # Cycle through the active joints and populate
    # a dictionary for those values
    joint_values = dict()
    for joint_name in self._arm_joint_names:
        # Find that joint name in msg
        idx = msg.name.index(joint_name)
 
        # Populate the joint message in a dictionary
        joint_values[joint_name] = msg.position[idx]
 
    return joint_values
    
  def enableGravComp(self):
    #if(not self.gc_connection):
    #  print 'GravComp Service not available'
    print self.grav_comp_client(True)

  def disableGravComp(self):
    #if(not self.gc_connection):
    #  print 'GravComp Service not available'
    print self.grav_comp_client(False)
      

  def js_cb(self, inState):
    for i in range(0,len(inState.position)):
      self.joint_states[i] = inState.position[i]
     
    self.last_js_update = rospy.get_time()
    
  def get_pos(self):
    return self.joint_states

  def ang_pos_cmd(self, angles):
    if not len(angles) == len(self.arm_joint_names):
      print "Number of desired joint angles does not match the number of available joints"
      return
    self.angular_cmd.position = True
    self.angular_cmd.joints = angles
    self.pub_jaco_ang.publish(self.angular_cmd) 

  def ang_vel_cmd(self, velocities):
    if not len(velocities) == len(self.arm_joint_names):
      print "Number of desired joint velocities does not match the number of available joints"
      return
    self.angular_cmd.position = False
    self.angular_cmd.joints = velocities
    self.pub_jaco_ang.publish(self.angular_cmd) 

  def cart_pos_cmd(self, pose):
    if not len(pose) == 6:
      print "Not enough pose parameters specified"
      return
    self.cartesian_cmd.position = True
    self.cartesian_cmd.arm.linear.x = pose[0]
    self.cartesian_cmd.arm.linear.y = pose[1]
    self.cartesian_cmd.arm.linear.z = pose[2]
        
    self.cartesian_cmd.arm.angular.x = pose[3]
    self.cartesian_cmd.arm.angular.y = pose[4]
    self.cartesian_cmd.arm.angular.z = pose[5]

    self.pub_jaco_cart.publish(self.cartesian_cmd)

  def cart_pos_cmd(self, translation, rotation):
    if not len(translation) == 3:
      print "Not enough translations specified"
      return
    if not len(rotation) == 3:
      print "Not enough rotations specified"
      return
    pose = translation + rotation
    self.cart_pos_cmd(pose)


  def cart_vel_cmd(self, vels):
    if not len(vels) == 6:
      print "Not enough velocities specified"
      return
    self.cartesian_cmd.position = False
    self.cartesian_cmd.arm.linear.x = vels[0]
    self.cartesian_cmd.arm.linear.y = vels[1]
    self.cartesian_cmd.arm.linear.z = vels[2]
        
    self.cartesian_cmd.arm.angular.x = vels[3]
    self.cartesian_cmd.arm.angular.y = vels[4]
    self.cartesian_cmd.arm.angular.z = vels[5]

    self.pub_jaco_cart.publish(self.cartesian_cmd)

  def cart_vel_cmd(self, translation, rotation):
    if not len(translation) == 3:
      print "Not enough translation velocities specified"
      return
    if not len(rotation) == 3:
      print "Not enough rotation velocities specified"
      return
    vels = translation + rotation
    self.cart_pos_cmd(vels)
    
  def ang_cmd_loop(self,angles,rate=10,iterations=5):
    rrate = rospy.Rate(rate)
    for i in range(0,iterations):
      self.ang_pos_cmd(angles)
      rrate.sleep()  
      
  def ang_cmd_wait(self,angles,epsilon=0.05, maxIter=50, rate=10):
    error = epsilon + 1;
    epsilon=5
    iterNum = 0;
    #self.ang_cmd_loop(angles,rate)
    self.ang_pos_cmd(angles)
    rrate = rospy.Rate(rate)
    while error > epsilon and iterNum < maxIter:
      error = vectorDiff(self.joint_states,angles)
      iterNum += 1
      rrate.sleep()
      
    if iterNum == maxIter:
      return False
    return True
    
  #the full handling of vels, accs and effs will come later
  # only the waypoints are needed for wpi jaco! the rest gets thrown away anyway so feel free to skip
  def sendWaypointTrajectory(self, waypoints, durations = 0., vels = 0., accs = 0., effs = 0.):
    if not self.ang_cmd_wait(waypoints[0]):
      print 'Cannot go to the first point in the trajectory'
      return None
#    else: 
#      print 'Went to first'
      
    if not self.traj_connection:
      print 'Action server connection was not established'
      return None
    joint_traj = JointTrajectory()
    joint_traj.joint_names = self.arm_joint_names;
    
    if not durations == 0:
      if not len(durations) == waypoints:
        raise Exception('The number of duration points is not equal to the number of provided waypoints')
    if not vels == 0:
      if not len(vels) == waypoints:
        raise Exception('The number velocity points is not equal to the number of provided waypoints')
    if not accs == 0:
      if not len(accs) == waypoints:
        raise Exception('The number acceleration points is not equal to the number of provided waypoints')
    if not effs == 0:
      if not len(effs) == waypoints:
        raise Exception('The number effort points is not equal to the number of provided waypoints')

    if not effs == 0:    
      if not (vels == 0 and accs == 0):
        raise Exception('Cannot specify efforts with velocities and accelerations at the same time')
    if (not accs == 0) and vels == 0:
      raise Exception('Cannot specify accelerations without velocities')
   
    total_time_from_start = 0.5; 
    for t in range(0, len(waypoints)):
      point = JointTrajectoryPoint()
  
      waypoint = waypoints[t]   
      if not len(waypoint) == len(joint_traj.joint_names):
        raise Exception('The number of provided joint positions is not equal to the number of available joints for index: ' + str(t))
      point.positions = waypoint
    
      if not vels == 0.:
        velocity = vels[t]
        if not len(velocity) == len(joint_traj.joint_names):
          raise Exception('The number of provided joint velocities is not equal to the number of available joints for index: ' + str(t))
        point.velocities = velocity

      if not accs == 0.:
        acceleration = accs[t]
        if not len(acceleration) == len(joint_traj.joint_names):
          raise Exception('The number of provided joint accelerations is not equal to the number of available joints for index: ' + str(t))
        point.accelerations = accelerations
      
      if not effs == 0.:
        effort = effs[t]
        if not len(effort) == len(joint_traj.joint_names):
          raise Exception('The number of provided joint efforts is not equal to the number of available joints for index: ' + str(t))
        point.effort = effort 

      if not durations == 0.:
        point.duration = duration

      # Deal with increasing time for each trajectory point
      point.time_from_start = rospy.Duration(total_time_from_start)
      total_time_from_start = total_time_from_start + 1.0 

      # Set the points
      joint_traj.points.append(point)
     
    traj_goal = FollowJointTrajectoryGoal()
    traj_goal.trajectory = joint_traj

    self.smooth_joint_trajectory_client.send_goal(traj_goal)
    self.smooth_joint_trajectory_client.wait_for_result()
    return self.smooth_joint_trajectory_client.get_result() 

  # Expects waypoints to be in joint space
  def execute_traj_moveit(self, waypoints):
    # Cycle through waypoints
    for point in waypoints:
      plannedTraj = self.arm_planner.plan_jointTargetInput(point)
      if plannedTraj == None or len(plannedTraj.joint_trajectory.points) < 1:
        print "Error: no plan found"
	return -1
      else:
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory = plannedTraj.joint_trajectory
        self.smooth_joint_trajectory_client.send_goal(traj_goal)
        self.smooth_joint_trajectory_client.wait_for_result()   
        self.smooth_joint_trajectory_client.get_result() 
	return 1

  # Expects waypoints to be in end effector space
  def execute_pose_traj_moveit(self, waypoints):
    # Cycle through waypoints
    for point in waypoints:
      plannedTraj = self.arm_planner.plan_poseTargetInput(point)
      if plannedTraj == None or len(plannedTraj.joint_trajectory.points) < 1:
        print "Error: no plan found"
	return -1
      else:
        self.execute_plan_traj(plannedTraj)
	return 1

  def execute_plan_traj(self, plannedTraj):
    traj_goal = FollowJointTrajectoryGoal()
    traj_goal.trajectory = plannedTraj.joint_trajectory
    self.smooth_joint_trajectory_client.send_goal(traj_goal)
    self.smooth_joint_trajectory_client.wait_for_result()   
    self.smooth_joint_trajectory_client.get_result() 


#TODO: figure this out
  def upper_tuck(self, use_moveit=True, vanilla = False):

    if use_moveit:
      # Just last point
      return self.execute_traj_moveit([self.ut_wps[-1]])
    elif vanilla:
      self.sendWaypointTrajectory(self.ut_wps)
      return 1
    else:
      self._ut_with_network()
      return 1

  def upper_untuck(self, use_moveit=True, vanilla = False):
    if use_moveit:
      # Just last point
      self.execute_traj_moveit([self.un_ut_wps[-1]])
    elif vanilla:
      self.sendWaypointTrajectory(self.un_ut_wps)
    else:
      self.untuck()

  def lower_tuck(self, use_moveit=True, vanilla = False):
    if use_moveit:
      # Just last point
      self.execute_traj_moveit([self.lt_wps[-1]])
    elif vanilla:
      self.sendWaypointTrajectory(self.lt_wps)
    else:
      self._lt_with_network()

  def lower_untuck(self, use_moveit=True, vanilla = False):
    if use_moveit:
      # Just last point
      self.execute_traj_moveit([self.un_lt_wps[-1]])
    elif vanilla:
      self.sendWaypointTrajectory(self.un_lt_wps)
    else:
      self.untuck()
    
  def untuck(self, use_moveit=True):
    if use_moveit:
      # Just last point
      self.execute_traj_moveit([self.tuck_network[-1]])
    else:
        self._untuck_with_network()

  def _init_tuck_poses(self):
    self.mid_wp = [-1.57, 3.14, 1.05, -1.57, 1.05, 1.57]
    
    lt_wp0 = [-1.65, 3.68, 1.12, -2.13, 1.48, 2.10]
    lt_wp1 = [-1.49, 4.00, 1.47, -1.74, 1.25, 1.96]
    lt_wp2 = [-1.23, 4.50, 0.95, -2.31, 1.82, 1.96]
    lt_wp3 = [-1.21, 4.76, 0.83, -2.60, 2.56, 1.63]

    self.lt_wps = [lt_wp0, lt_wp1, lt_wp2, lt_wp3]
    self.un_lt_wps = self.lt_wps[::-1]

    ut_wp0 = [-1.60, 2.20, 0.80, -2.20, 1.50, 1.20]
    ut_wp1 = [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90]
    ut_wp2 = [-1.80, 1.80, 1.00, -2.10, 2.50, 0.72]
    ut_wp3 = [-1.90, 1.50, 0.50, -2.00, 3.0, 0.72]

    self.ut_wps = [ut_wp0,ut_wp1,ut_wp2,ut_wp3]
    self.un_ut_wps = self.ut_wps[::-1]
    
    self.tuck_network          = self.un_lt_wps + [self.mid_wp] + self.ut_wps
    self.reversed_tuck_network = self.tuck_network[::-1]
    
  def _find_closest_tuck_wp(self, tuck_wps, max_allowed_dist = 8.0):
    if self.last_js_update is not None:
      if self.last_js_update + 2.0 < rospy.get_time():
        print 'The newest joint state information is too old.'
        return None
    
    minDiff = 1000;    
    minInd = -1
    for i in range(0,len(tuck_wps)):
      diff = vectorDiff(tuck_wps[i],self.joint_states)
      if diff < minDiff:
        minDiff = diff
        minInd = i
        
    if minDiff > max_allowed_dist:
      print 'Current arm configuration ' + str(self.joint_states) + ' is too far from the tuck network: ' + str(minDiff) + ' ' + str(minInd)
      return None
    
    return minInd
    
  def _lt_with_network(self):
    ind = self._find_closest_tuck_wp(self.reversed_tuck_network)
    if ind is not None:
      self.sendWaypointTrajectory(self.reversed_tuck_network[ind:])

  def _ut_with_network(self):
    ind = self._find_closest_tuck_wp(self.tuck_network)
    if ind is not None:
      self.sendWaypointTrajectory(self.tuck_network[ind:])
    
  def _untuck_with_network(self):
    ind = self._find_closest_tuck_wp(self.tuck_network)
    if ind is not None:
      midPoint = 4;
      if ind == midPoint:
        wps = self.tuck_network[midPoint]
      elif ind < midPoint:
        wps = self.tuck_network[ind:midPoint+1]
      else:    
        wps = self.reversed_tuck_network[(len(self.reversed_tuck_network)-ind+1):midPoint+1]
      
      self.sendWaypointTrajectory(wps)
      
def vectorDiff(v1,v2):
 error = 0;
 l = min(len(v1),len(v2))
 for i in range(0,l):
   diff = (v1[i] - v2[i])
   error += diff*diff;
 error = sqrt(error)
 return error
    


