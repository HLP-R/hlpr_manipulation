#ifndef HLPR_MOVEIT_WRAPPER_H_
#define HLPR_MOVEIT_WRAPPER_H_

//CPP
#include <boost/thread/recursive_mutex.hpp>

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/CallIK.h>
#include <rail_manipulation_msgs/CartesianPath.h>
#include <rail_manipulation_msgs/GraspingState.h>
#include <rail_manipulation_msgs/MoveToJointPoseAction.h>
#include <rail_manipulation_msgs/MoveToPoseAction.h>
#include <rail_manipulation_msgs/PrepareGrasp.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <wpi_jaco_msgs/AngularCommand.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#define NUM_JACO_JOINTS 6
#define SCENE_OBJECT_DST_THRESHOLD 0.2

class HlprMoveitWrapper
{

public:

  HlprMoveitWrapper();

  ~HlprMoveitWrapper();

private:
  ros::NodeHandle n, pnh;
  ros::Subscriber armJointStateSubscriber;
  ros::Subscriber recognizedObjectsSubscriber;
  ros::Subscriber followJointTrajectoryResultSubscriber;
  ros::Publisher angularCmdPublisher;
  ros::Publisher trajectoryVisPublisher;
  ros::Publisher graspingStatePublisher;
  ros::Publisher planningScenePublisher;
  ros::ServiceServer cartesianPathServer;
  ros::ServiceServer ikServer;
  ros::ServiceServer attachObjectServer;
  ros::ServiceServer detachObjectServer;
  ros::ServiceServer prepareGraspServer;
  ros::ServiceServer reactivateGraspCollisionServer;
  ros::ServiceClient ikClient;
  ros::ServiceClient clearOctomapClient;
  ros::ServiceClient planningSceneClient;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> armTrajectoryClient;
  actionlib::SimpleActionServer<rail_manipulation_msgs::MoveToPoseAction> moveToPoseServer;
  actionlib::SimpleActionServer<rail_manipulation_msgs::MoveToJointPoseAction> moveToJointPoseServer;

  tf::TransformListener tf;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  moveit::planning_interface::MoveGroupInterface *jacoArmGroup;
  moveit::planning_interface::PlanningSceneInterface *planningSceneInterface;
  planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor;

  boost::recursive_mutex api_mutex;

  sensor_msgs::JointState jointState;
  rail_manipulation_msgs::SegmentedObjectList objectList;  //the last received object list
  std::vector<std::string> attachedObjects;  //the names of the objects (in the planning scene) attached to the robot
  std::vector<std::string> unattachedObjects; //the names of the objects (in the planning scene) not attached to the robot
  std::vector<std::string> gripperNames;
  std::string ignoredObject;

  bool executionFinished; //state of execution, used for monitoring results of asyncExecute

  void moveToPose(const rail_manipulation_msgs::MoveToPoseGoalConstPtr &goal);

  void moveToJointPose(const rail_manipulation_msgs::MoveToJointPoseGoalConstPtr &goal);

  bool cartesianPathCallback(rail_manipulation_msgs::CartesianPath::Request &req, rail_manipulation_msgs::CartesianPath::Response &res);

  bool ikCallback(rail_manipulation_msgs::CallIK::Request &req, rail_manipulation_msgs::CallIK::Response &res);

  moveit_msgs::GetPositionIK::Response callIK(geometry_msgs::PoseStamped pose);

  void armJointStatesCallback(const sensor_msgs::JointState &msg);

  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

  void followJointTrajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult &msg);

  bool attachClosestSceneObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool detachSceneObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool prepareGrasp(rail_manipulation_msgs::PrepareGrasp::Request &req, rail_manipulation_msgs::PrepareGrasp::Response &res);

  bool reactivateGraspCollision(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif
