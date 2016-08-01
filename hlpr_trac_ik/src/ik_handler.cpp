#include "ros/ros.h"
#include "hlpr_trac_ik/IKHandler.h"
#include <trac_ik/trac_ik.hpp>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>
#include <Eigen/Core>

TRAC_IK::TRAC_IK *solver;

bool getPose(hlpr_trac_ik::IKHandler::Request  &req,
         hlpr_trac_ik::IKHandler::Response &res)
{
  int poseLen = req.origin.size();

  //convert seed pose to KDL::JntArray format
  KDL::JntArray prev(poseLen);
  std::string seedStr = "";
  for (int i = 0; i < poseLen; i++) {
    seedStr += std::to_string(req.origin[i]) + " ";
    prev(i) = req.origin[i];
  }

  int len = req.goals.size();
  std::vector<geometry_msgs::Pose> goals = req.goals;
  for(int p = 0; p < len; p++) {
    //convert goal to KDL::Frame format
    std::string goalStr = "";
    KDL::Vector goalPos(goals[p].position.x, goals[p].position.y, goals[p].position.z);
    KDL::Rotation goalRot = KDL::Rotation::Quaternion(goals[p].orientation.x, goals[p].orientation.y, goals[p].orientation.z, goals[p].orientation.w);
    KDL::Frame goalPose(goalRot,goalPos);
    goalStr += std::to_string(goals[p].position.x) + " " + std::to_string(goals[p].position.y) + " " + std::to_string(goals[p].position.z) 
        +  " " + std::to_string(goals[p].orientation.x) + " " + std::to_string(goals[p].orientation.y)
        + " " + std::to_string(goals[p].orientation.z) + " " + std::to_string(goals[p].orientation.w);

    ROS_INFO("request: seed=%s goal=%s", seedStr.c_str(), goalStr.c_str());
 
    //set tolerances and send IK request
    KDL::Vector posTol(req.tolerance[0],req.tolerance[1],req.tolerance[2]);
    float max = std::numeric_limits<float>::max();
    KDL::Vector rotTol(req.tolerance[3],req.tolerance[4],req.tolerance[5]);
    KDL::Twist tol(posTol, rotTol);
    int status = solver->CartToJnt(prev, goalPose, prev, tol);
    if(status < 0)
  	ROS_INFO("IK failed");

    //add IK result to service response
    std::string poseStr = "";
    trajectory_msgs::JointTrajectoryPoint pt;
    for(int i = 0; i < poseLen; i++) {
      double val = prev.data[i];
      pt.positions.push_back(val); 
      poseStr += std::to_string(val) += " ";
    }
    res.poses.push_back(pt);
    ROS_INFO("sending back response: %s", poseStr.c_str());
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hlpr_trac_ik_server");
  ros::NodeHandle n;

  solver = new TRAC_IK::TRAC_IK("linear_actuator_link", "right_ee_link", "/robot_description",0.005,1e-5, TRAC_IK::Distance);

  ros::ServiceServer service = n.advertiseService("hlpr_trac_ik", getPose);
  ROS_INFO("IK wrapper ready.");
  ros::spin();
  delete solver;
  return 0;
}
