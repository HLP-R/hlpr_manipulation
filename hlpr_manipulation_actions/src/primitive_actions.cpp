#include <hlpr_manipulation_actions/primitive_actions.h>

using namespace std;

PrimitiveActions::PrimitiveActions() : pnh("~"),
                                 primitiveServer(n, "hlpr_manipulation/primitive_action", boost::bind(&PrimitiveActions::executePrimitive, this, _1), false)
{
  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);

  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("jaco_arm/erase_trajectories");
  cartesianPathClient = n.serviceClient<rail_manipulation_msgs::CartesianPath>("hlpr_moveit_wrapper/cartesian_path");
  jacoPosClient = n.serviceClient<wpi_jaco_msgs::GetAngularPosition>("jaco_arm/get_angular_position");

  primitiveServer.start();
}

void PrimitiveActions::executePrimitive(const rail_manipulation_msgs::PrimitiveGoalConstPtr &goal)
{
  ROS_INFO("Received primitive action request.");
  rail_manipulation_msgs::PrimitiveFeedback feedback;
  rail_manipulation_msgs::PrimitiveResult result;
  stringstream ss;

  if (goal->primitive_type == rail_manipulation_msgs::PrimitiveGoal::TRANSLATION)
  {
    ss << "Moving the arm ";
    string direction;
    switch (goal->axis)
    {
      case (rail_manipulation_msgs::PrimitiveGoal::X_AXIS):
        if (goal->distance >= 0)
          direction = "right";
        else
          direction = "left";
        break;
      case (rail_manipulation_msgs::PrimitiveGoal::Y_AXIS):
        if (goal->distance >= 0)
          direction = "forward";
        else
          direction = "back";
        break;
      case (rail_manipulation_msgs::PrimitiveGoal::Z_AXIS):
        if (goal->distance >= 0)
          direction = "up";
        else
          direction = "down";
        break;
    }
    ss << direction << "...";

    feedback.feedback = ss.str();
    primitiveServer.publishFeedback(feedback);

    tf::StampedTransform startTransform;
    tfListener.lookupTransform("base_link", "right_ee_link", ros::Time(0), startTransform);
    geometry_msgs::PoseStamped startPose;
    startPose.header.frame_id = "base_link";
    startPose.header.stamp = ros::Time::now();
    startPose.pose.position.x = startTransform.getOrigin().x();
    startPose.pose.position.y = startTransform.getOrigin().y();
    startPose.pose.position.z = startTransform.getOrigin().z();
    tf::quaternionTFToMsg(startTransform.getRotation(), startPose.pose.orientation);

    geometry_msgs::PoseStamped endPose;
    endPose.header = startPose.header;
    endPose.pose = startPose.pose;
    switch (goal->axis)
    {
      case (rail_manipulation_msgs::PrimitiveGoal::X_AXIS):
        endPose.pose.position.x += goal->distance;
        break;
      case (rail_manipulation_msgs::PrimitiveGoal::Y_AXIS):
        endPose.pose.position.y += goal->distance;
        break;
      case (rail_manipulation_msgs::PrimitiveGoal::Z_AXIS):
        endPose.pose.position.z += goal->distance;
        break;
    }

    ROS_INFO("Sending goal to Cartesian Path service");
    rail_manipulation_msgs::CartesianPath cartesianPath;
    cartesianPath.request.avoidCollisions = false;
    cartesianPath.request.waypoints.push_back(endPose);
    if (!cartesianPathClient.call(cartesianPath))
    {
      feedback.feedback = "Couldn't move arm, an error has occurred.";
      primitiveServer.publishFeedback(feedback);

      ROS_INFO("Cartesian Path service call failed.");
      primitiveServer.setAborted();
      return;
    }

    result.completion = cartesianPath.response.completion;
    ROS_INFO("Path completion: %f", result.completion);

    if (result.completion <= 0.5)
    {
      ss.str("");
      ss << "The arm couldn't fully move " << direction << ". Try another command.";
      feedback.feedback = ss.str();
      primitiveServer.publishFeedback(feedback);
    }
    else
    {
      ss.str("");
      ss << "Finished moving the arm " << direction << ".";
      feedback.feedback = ss.str();
      primitiveServer.publishFeedback(feedback);
    }

    primitiveServer.setSucceeded(result);
    return;
  }
  else if (goal->primitive_type == rail_manipulation_msgs::PrimitiveGoal::ROTATION)
  {
    ss << "Rotating the gripper ";
    string direction;
    if (goal->distance >= 0)
      direction = "counterclockwise";
    else
      direction = "clockwise";
    ss << direction << "...";

    feedback.feedback = ss.str();
    primitiveServer.publishFeedback(feedback);

    wpi_jaco_msgs::GetAngularPosition getAngularPosition;
    if (!jacoPosClient.call(getAngularPosition))
    {
      feedback.feedback = "Couldn't rotate the gripper, an error has occurred.";
      primitiveServer.publishFeedback(feedback);

      ROS_INFO("Jaco Angular Position service call failed.");
      primitiveServer.setAborted();
      return;
    }
    wpi_jaco_msgs::AngularCommand angularCmd;
    angularCmd.armCommand = true;
    angularCmd.fingerCommand = false;
    angularCmd.position = true;
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
    {
      angularCmd.joints.push_back(getAngularPosition.response.pos[i]);
    }
    int wristJoint = NUM_JACO_JOINTS - 1;
    angularCmd.joints[wristJoint] += goal->distance;

    angularCmdPublisher.publish(angularCmd);
    ros::Duration(0.5).sleep(); //wait for command to go through

    ros::Rate loopRate(10);
    bool commandFinished = false;
    double prevJointPos = getAngularPosition.response.pos[wristJoint];
    double startJointPos = getAngularPosition.response.pos[wristJoint];
    ros::Duration(0.5).sleep(); //short wait to allow arm to start moving
    while (!commandFinished)
    {
      loopRate.sleep();
      if (!jacoPosClient.call(getAngularPosition))
      {
        ROS_INFO("Jaco Angular Position service call failed.");
        commandFinished = true;
      }
      if (fabs(getAngularPosition.response.pos[wristJoint] - prevJointPos) < 0.02)
      {
        commandFinished = true;
      }
      prevJointPos = getAngularPosition.response.pos[wristJoint];
    }

    //publish stop command
    angularCmd.position = false;
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
    {
      angularCmd.joints.push_back(0.0);
    }
    angularCmdPublisher.publish(angularCmd);

    result.completion = min(fabs((prevJointPos - startJointPos)/goal->distance), 1.0);

    if (result.completion <= 0.5)
    {
      ss.str("");
      ss << "The gripper couldn't fully rotate " << direction << ". Try another command.";
      feedback.feedback = ss.str();
      primitiveServer.publishFeedback(feedback);
    }
    else
    {
      ss.str("");
      ss << "Finished rotating the gripper " << direction << ".";
      feedback.feedback = ss.str();
      primitiveServer.publishFeedback(feedback);
    }

    primitiveServer.setSucceeded(result);
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "primitive_actions");

  PrimitiveActions pa;

  ros::spin();
}
