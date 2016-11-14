#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
from math import pi, floor, ceil, fabs

class ArmMoveIt:

  def __init__(self, planning_frame='base_link', default_planner="RRTConnectkConfigDefault"):
    # self.pose = geometry_msgs.msg.PoseStamped()
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    self.group = [moveit_commander.MoveGroupCommander("arm")]

    # Set the planner
    self.planner = default_planner

    # Set the planning pose reference frame
    self.group[0].set_pose_reference_frame(planning_frame)

    # Set continuous joint names
    self.continuous_joints = ['shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    # NOTE: order that moveit currently is configured
    # ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5] # joints that are continous

  def get_IK(self, newPose, root = None):
    ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

    wkPose = geometry_msgs.msg.PoseStamped()
    if root is None:
      wkPose.header.frame_id = self.group[0].get_planning_frame() # name:odom
    else:
      wkPose.header.frame_id = root

    wkPose.header.stamp=rospy.Time.now()
    wkPose.pose=newPose

    msgs_request = moveit_msgs.msg.PositionIKRequest()
    msgs_request.group_name = self.group[0].get_name() # name: arm
    # msgs_request.robot_state = robot.get_current_state()
    msgs_request.pose_stamped = wkPose
    msgs_request.timeout.secs = 2
    msgs_request.avoid_collisions = False

    try:
      jointAngle=compute_ik(msgs_request)
      ans=list(jointAngle.solution.joint_state.position[1:7])
      if jointAngle.error_code.val == -31:
        print 'No IK solution'
        return None
      return ans

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def get_FK(self, root = 'base_link'):

    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

    header = std_msgs.msg.Header()
    header.frame_id = root
    header.stamp = rospy.Time.now()
    fk_link_names = ['right_ee_link']
    robot_state = self.robot.get_current_state()    
    try:
      reply=compute_fk(header,fk_link_names,robot_state)
      return reply.pose_stamped

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def get_FK_wpi(self, joints = None):

    rospy.wait_for_service('/jaco_arm/kinematics/fk')
    compute_fk = rospy.ServiceProxy('/jaco_arm/kinematics/fk', wpi_jaco_msgs.srv.JacoFK)

    if joints is None:
      joints = [pi,pi,pi,pi,pi,pi,pi]    

    try:
      pose=compute_fk(joints)      
      return pose

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e


  def plan_targetInput(self, target, joint_flag):
    '''Generic target planner that what type is specified'''
    try:
      if (joint_flag):
	self.group[0].set_joint_value_target(self._simplify_joints(target))
      else:
	self.group[0].set_pose_target(target)

      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None


  def plan_targetInputWaypoint(self, targets, joint_flag, merged=False):
    '''Generic target planner that what type is specified'''
    ## input: list of pose (geometry_msgs.msg.Pose())
    ## output: plan from current  pose all of the target poses
    ## If merge true - then a single large plan is returned
    ## If merge is false - then several plans in an array are returned

    # Plan each pose individually and stitch together
    try: 
      full_plan = []
      points = []
      current_state = self.robot.get_current_state()
      for target in targets:
          self.group[0].set_start_state(current_state)
	  plan = self.plan_targetInput(target, joint_flag)
          if plan is not None:
              full_plan.append(plan)
	      if merged:
		points = self.merge_points(points, plan.joint_trajectory.points)
	      traj = plan.joint_trajectory
              current_state = self.set_robot_state_pose(traj)
          else:
              print 'No full plan found, see the moveit terminal for the error'
              return None

      if merged:
	plan = full_plan[0]
	plan.joint_trajectory.points = points
	return plan
      else:
	return full_plan
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None


  def set_robot_state_pose(self, traj):
    '''Gets the current robot state pose and sets it to the joint pose'''
    cur_robot_state = self.robot.get_current_state()
    last_point = traj.points[-1].positions
    # convert the joints to array
    joints = [x for x in cur_robot_state.joint_state.position]
    for i in xrange(len(traj.joint_names)):
	# Find index of joint
	joint_name = traj.joint_names[i]
	idx = cur_robot_state.joint_state.name.index(joint_name)
	joints[idx] = last_point[i]

    # Set full joint tuple now
    cur_robot_state.joint_state.position = joints

    return cur_robot_state

  def merge_points(self, points, new_points):
    '''Merge two sets of points and taking into account time'''
    # Check if this is the first set
    if len(points) < 1:
      return new_points

    all_points = points
    # Pull out the last time from current points
    last_point_time = points[-1].time_from_start+rospy.Duration(0.1)
    for point in new_points:
      point.time_from_start = point.time_from_start+last_point_time
      all_points = all_points + [point]
    return all_points

  def _simplify_angle(self, angle):
    # Very simple function that makes sure the angles are between -pi and pi
    if angle > pi:
      while angle > pi:
        angle -= 2*pi
    elif angle < -pi:
      while angle < -pi:
        angle += 2*pi

    return angle

  def _simplify_joints(self, joint_dict):
    # Helper function to convert a dictionary of joint values
    if isinstance(joint_dict, dict):
      simplified_joints = dict()
      for joint in joint_dict:
	# Pull out the name of the joint
	joint_name = '_'.join(joint.split('_')[1::])
	if joint_name in self.continuous_joints:
	  simplified_joints[joint] = self._simplify_angle(joint_dict[joint])
	else:
	  simplified_joints[joint] = joint_dict[joint]
    elif isinstance(joint_dict, list):
      simplified_joints = []
      for i in xrange(len(joint_dict)):
	a = joint_dict[i]
	if i in self.continuous_joints_list:
	  simplified_joints.append(self._simplify_angle(a))
	else:
	  simplified_joints.append(a)
    return simplified_joints

  '''Older functions - left for backwards compatibility'''

  def plan_jointTargetInput(self,target_joint):
    ## input: target joint angles (list) of the robot
    ## output: plan from current joint angles to the target one
    try:
      self.group[0].set_joint_value_target(self._simplify_joints(target_joint))
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None

  def plan_poseTargetInput(self,target_pose):
    ## input: tart pose (geometry_msgs.msg.Pose())
    ## output: plan from current  pose to the target one
    try:
      self.group[0].set_pose_target(target_pose)
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None
 
 
  def box_table_scene(self) :
    
    #Scene : add box 
    # after loading this object/scene, need to do "add" ==> "planning scene"  
    # in the rviz interface if one want to see the box
    
     rospy.sleep(2)
     self.scene.remove_world_object("table_box")
    
     p = geometry_msgs.msg.PoseStamped()
     p.header.frame_id = self.robot.get_planning_frame()
     p.pose.position.x = 1.64
     p.pose.position.y = 0.0
     p.pose.position.z = 0.25
     p.pose.orientation.w = 0.0
     self.scene.add_box("table_box",p,(0.75, 1, 0.5))
    
     rospy.sleep(5)

  def wayPointIK(self, wps, numSteps = None, ik_root = None):
    if numSteps is None:
      numSteps = 3
    jointWps = []

    for i in range(0, len(wps)):
      jointP = self.get_IK(wps[i], ik_root)
      if jointP is None:
        jointWps = None
        break
      jointWps.append(jointP)

    return jointWps

def ask_scene_integration(arm):
  # Ask the user if want to integrate a box scene
  answer= input("""\n Integrate a box as a table from code ? (1 or 0)  
  (if 1: box can't be displaced nor resized by user, if 0: no scene (can always do add from rviz interface) ) \n""")
  
  if answer == 1:
    arm.box_table_scene()
    print "\n Box inserted; to see it ==> rviz interface ==> add button==> planning scene  "
    return
  else:
    print "\n No scene added"
    return  
  
def ask_position(arm,tarPose):
  #Ask the user the values of the target position
   while True:
    try:   
      inputPosition=input(""" \n Target position coord. (format: x,y,z or write -1 to take the robot current position ): """)      
    
      if inputPosition == -1:
        inputPosition=tarPose.position  
        return inputPosition
      
    except (ValueError,IOError,NameError):
      print("\n Please, enter the coordinate in the following format: x,y,z ")
      continue
    else:          
      if len(list(inputPosition)) == 3:
        poseTmp= geometry_msgs.msg.Pose()
        poseTmp.position.x=inputPosition[0]
        poseTmp.position.y=inputPosition[1]
        poseTmp.position.z=inputPosition[2]
        return poseTmp.position
      else:
        print("\n Please, enter the coordinate in the following format: x,y,z ")
        continue
      
        
def ask_orientation(arm,tarPose):
  # Ask the user the values of the target quaternion
  while True:
    try:   
      inputQuat=input(""" \n Target quaternion coordi. (format: qx,qy,qz,qw or write -1 to take the robot current quaternion ):""")
      
      if inputQuat == -1:
        inputQuat=arm.group[0].get_current_pose().pose.orientation                   
        return  inputQuat
        
    except (ValueError,IOError,NameError):
      print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
      continue
    else:
      if len(list(inputQuat)) == 4:
        poseTmp= geometry_msgs.msg.Pose()
        poseTmp.orientation.x=inputQuat[0]
        poseTmp.orientation.y=inputQuat[1]
        poseTmp.orientation.z=inputQuat[2]
        poseTmp.orientation.w=inputQuat[3]
        return poseTmp.orientation    
      else:
        print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
        
def main():
  arm = ArmMoveIt()

  tarPose = geometry_msgs.msg.Pose()

  ## ask if integrate object scene from code or not
  ask_scene_integration(arm)
  
  while not rospy.is_shutdown():
    
    ##   Assigned tarPose the current Pose of the robot 
    tarPose = arm.group[0].get_current_pose().pose
  

    ## ask input from user (COMMENT IF NOT USE AND WANT TO ASSIGN MANUAL VALUE IN CODE)    
    tarPose.position = ask_position(arm,tarPose)   
    tarPose.orientation = ask_orientation(arm,tarPose)  

   ##  Example of Assigned values for new targetPose to robot
#    tarPose.position.x = 0.89
#    tarPose.position.y = 0.00
#    tarPose.position.z = 0.32   
#    tarPose.orientation.x = 0.0     
                  
    print '\n The target coordinate is: %s \n' %tarPose     
    
    ## IK for target position  
    jointTarg = arm.get_IK(tarPose)
    print 'IK calculation step:DONE' 
    
    ## planning with joint target from IK 
    planTraj =  arm.plan_jointTargetInput(jointTarg)
    print 'Planning step with target joint angles:DONE' 
      
    ## planning with pose target
    #print 'Planning step with target pose'   
    #planTraj = arm.plan_poseTargetInput(tarPose)
      
    ## execution of the movement   
    #print 'Execution of the plan' 
    # arm.group[0].execute(planTraj)
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()

