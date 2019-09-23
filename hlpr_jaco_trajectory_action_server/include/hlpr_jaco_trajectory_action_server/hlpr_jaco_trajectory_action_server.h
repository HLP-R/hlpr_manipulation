#ifndef HLPR_JACO_TRAJECTORY_ACTION_SERVER_H_
#define HLPR_JACO_TRAJECTORY_ACTION_SERVER_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/foreach.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ecl/geometry.hpp>
#include <kinova_msgs/Stop.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/JointVelocity.h>
#include <sensor_msgs/JointState.h>

#define NUM_JACO_JOINTS 7

#define LARGE_ACTUATOR_VELOCITY 0.628 //maximum velocity of large actuator (joints 1-4) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 0.838 //maximum velocity of small actuator (joints 5-7) (rad/s)

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

//gains for trajectory follower
#define KP 225.0
#define KV 10.0
#define ERROR_THRESHOLD .001 //threshold in radians for error at each joint to consider motion a success

class JacoTrajectoryController
{
public:
  JacoTrajectoryController();

  bool startGravityCompService(kinova_msgs::Start::Request &req,
                                kinova_msgs::Start::Response &res);
  bool stopGravityCompService(kinova_msgs::Stop::Request &req,
                                  kinova_msgs::Stop::Response &res);

  bool startForceControlCallback(kinova_msgs::Start::Request &req,
                                   kinova_msgs::Start::Response &res);
  bool stopForceControlCallback(kinova_msgs::Stop::Request &req,
                                  kinova_msgs::Stop::Response &res);

  void jointStateCallback(const sensor_msgs::JointState &msg);

  void executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //Actions
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smoothTrajectoryServer;

  // Messages
  ros::Publisher angularCmdPublisher;
  ros::Subscriber jointStatesSubscriber;
  ros::Publisher angCmdSimPublisher; //!< publisher for simulation argular arm commands

  // Fake gravity comp services for simulation
  ros::ServiceServer start_gravity_comp_;
  ros::ServiceServer stop_gravity_comp_;
  ros::ServiceServer start_force_control_service_;
  ros::ServiceServer stop_force_control_service_;

  boost::recursive_mutex executionMutex;

  // Parameters
  double maxCurvature;
  bool   sim_flag_;
  bool   cubic_flag_;

  sensor_msgs::JointState jointStates;

  std::vector<std::string> jointNames;
};

#endif

