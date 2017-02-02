#ifndef PRIMITIVE_ACTIONS_H_
#define PRIMITIVE_ACTIONS_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rail_manipulation_msgs/PrimitiveAction.h>
#include <rail_manipulation_msgs/CartesianPath.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>

class PrimitiveActions
{
public:
  /**
  * \brief Constructor
  */
  PrimitiveActions();

private:
  ros::NodeHandle n, pnh;
  ros::Publisher angularCmdPublisher;

  ros::ServiceClient eraseTrajectoriesClient;
  ros::ServiceClient cartesianPathClient;
  ros::ServiceClient jacoPosClient;

  actionlib::SimpleActionServer<rail_manipulation_msgs::PrimitiveAction> primitiveServer;

  tf::TransformListener tfListener;

  static const int NUM_JACO_JOINTS = 6;

  void executePrimitive(const rail_manipulation_msgs::PrimitiveGoalConstPtr &goal);
};

#endif
