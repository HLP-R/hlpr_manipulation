#include <hlpr_jaco_trajectory_action_server/hlpr_jaco_trajectory_action_server.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <iostream>

using namespace std;

JacoTrajectoryController::JacoTrajectoryController() : pnh("~"),
  smoothTrajectoryServer(pnh, "trajectory", boost::bind(&JacoTrajectoryController::executeSmoothTrajectory, this, _1), false)
{
  ROS_INFO("Starting HLP-R Custom Jaco Joint Trajectory Action Server");

  pnh.param("max_curvature", maxCurvature, 100.0);
  pnh.param("sim", sim_flag_, false);

  jointNames.clear();
  jointNames.push_back("j2s7s300_joint_1");
  jointNames.push_back("j2s7s300_joint_2");
  jointNames.push_back("j2s7s300_joint_3");
  jointNames.push_back("j2s7s300_joint_4");
  jointNames.push_back("j2s7s300_joint_5");
  jointNames.push_back("j2s7s300_joint_6");
  jointNames.push_back("j2s7s300_joint_7");

  // Setting up simulation vs. real robot
  if(!sim_flag_)
  {
    ROS_INFO("Using real robot arm.");

    // Connect to the low-level angular driver from kinova-ros
    angularCmdPublisher = n.advertise<kinova_msgs::JointVelocity>("j2s7s300_driver/in/joint_velocity", 1);
  }
  else
  {
    ROS_INFO("Using simulation robot arm.");

    // Setup a fake gravity comp service (torque control)
    start_gravity_comp_ = n.advertiseService(
                "j2s7s300_driver/in/start_gravity_comp", &JacoTrajectoryController::startGravityCompService, this);
    stop_gravity_comp_ = n.advertiseService(
                "j2s7s300_driver/in/stop_gravity_comp", &JacoTrajectoryController::stopGravityCompService, this);

    // Setup a fake admittance service (Force control)
    start_force_control_service_ = n.advertiseService("j2s7s300_driver/in/start_force_control", &JacoTrajectoryController::startForceControlCallback, this);
    stop_force_control_service_ = n.advertiseService("j2s7s300_driver/in/stop_force_control", &JacoTrajectoryController::stopForceControlCallback, this);

    // Connect to the gazebo low-level ros controller
    angCmdSimPublisher = n.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/command", 1);
  }

  // Subscribes to the joint states of the robot
  jointStatesSubscriber = n.subscribe("joint_states", 1, &JacoTrajectoryController::jointStateCallback, this);


  // Start the trajectory server
  smoothTrajectoryServer.start();
}

/** Fake Gravity Comp Services for Simulation **/
bool JacoTrajectoryController::startGravityCompService(kinova_msgs::Start::Request &req,
                                             kinova_msgs::Start::Response &res)
{
    ROS_INFO("Simulation 'enabled' grav comp. Note: nothing actually happens");
    res.start_result = "Start gravity compensation requested.";
    return true;
}

/** Fake Gravity Comp Services for Simulation **/
bool JacoTrajectoryController::stopGravityCompService(kinova_msgs::Stop::Request &req,
                                             kinova_msgs::Stop::Response &res)
{
    ROS_INFO("Simulation 'disabled' grav comp. Note: nothing actually happens");
    res.stop_result = "Stop gravity compensation requested.";
    return true;
}

bool JacoTrajectoryController::startForceControlCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res)
{
    ROS_INFO("Simulation 'enabled' admittance mode. Note: nothing actually happens");
    res.start_result = "Start force control requested.";
    return true;
}

bool JacoTrajectoryController::stopForceControlCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res)
{
    ROS_INFO("Simulation 'disabled' admittance mode. Note: nothing actually happens");
    res.stop_result = "Stop force control requested.";
    return true;
}

void JacoTrajectoryController::jointStateCallback(const sensor_msgs::JointState &msg)
{
  // Get the names of all the joints
  std::vector<std::string> pub_joint_names = msg.name;

  // Create a new message
  sensor_msgs::JointState arm_msg;
  std::vector<double> position, velocity, effort;
  std::vector<std::string> names;
  position.resize(NUM_JACO_JOINTS);
  velocity.resize(NUM_JACO_JOINTS);
  effort.resize(NUM_JACO_JOINTS);
  names.resize(NUM_JACO_JOINTS);
  arm_msg.position = position;
  arm_msg.velocity = velocity;
  arm_msg.effort = effort;
  arm_msg.name = names;

  // Cycle through the number of JACO joints
  for (int joint_id = 0; joint_id < NUM_JACO_JOINTS; joint_id++){

    // Find the location of the joint
    string joint_name = jointNames[joint_id];
    int msg_loc = distance(pub_joint_names.begin(), find(pub_joint_names.begin(), pub_joint_names.end(), joint_name));

    // Pull out joint loc and store
    arm_msg.position[joint_id] = msg.position[msg_loc];
    arm_msg.name[joint_id] = msg.name[msg_loc];
    arm_msg.velocity[joint_id] = msg.velocity[msg_loc];
    arm_msg.effort[joint_id] = msg.effort[msg_loc];
  }

  jointStates = arm_msg;
  //cout << "Current names: " << jointStates.name[0] << ", " << jointStates.name[1] << ", " << jointStates.name[2] << ", " << jointStates.name[3] << ", " << jointStates.name[4] << ", " << jointStates.name[5] << jointStates.name[6] << endl;
  //cout << "Current values: " << jointStates.position[0] << ", " << jointStates.position[1] << ", " << jointStates.position[2] << ", " << jointStates.position[3] << ", " << jointStates.position[4] << ", " << jointStates.position[5] << jointStates.position[6] << endl;
}

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 *
 *  also known as "smallest delta" or "shortest way around the circle"
 */
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

bool JacoTrajectoryController::jointVelocityCheck(const int numPoints, ecl::Array<double> &timePoints, std::vector<ecl::Array<double>> const &jointPoints)
{
  // Gather timing corrections for trajectory segments that violate max velocity
  float correctedTime[numPoints] = {};
  for (unsigned int i = 1; i < numPoints; i++)
  {
    // ROS_INFO_STREAM("time point " << i);
    float maxTime = 0.0;
    float vel = 0.0;

    float plannedTime = timePoints[i] - timePoints[i - 1];
    bool validTime = plannedTime > 0;

    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      float time = fabs(jointPoints[j][i] - jointPoints[j][i - 1]);
      if (plannedTime > 0)
        vel = fabs(jointPoints[j][i] - jointPoints[j][i - 1]) / plannedTime;

      if (j <= 3)
      {
        time /= LARGE_ACTUATOR_VELOCITY_LIMIT;
        if (plannedTime > 0 && vel > LARGE_ACTUATOR_VELOCITY_LIMIT)
          validTime = false;
      }
      else
      {
        time /= SMALL_ACTUATOR_VELOCITY_LIMIT;
        if (plannedTime > 0 && vel > SMALL_ACTUATOR_VELOCITY_LIMIT)
          validTime = false;
      }
      // ROS_INFO_STREAM("joint " << j << ", planned time = " << plannedTime << ", velocity = " << vel << ", valid=" << validTime);

      if (time > maxTime)
        maxTime = time;
    }

    if (!validTime)
      correctedTime[i] = maxTime;
  }

  // Apply timing corrections
  for (unsigned int i = 1; i < numPoints; i++)
  {
    correctedTime[i] += correctedTime[i - 1];
    timePoints[i] += correctedTime[i];
  }

  bool jointVelocitiesCorrected = (correctedTime[numPoints - 1] > 0);

      // Print warning if time corrections were applied
      if (jointVelocitiesCorrected)
  {
    ROS_WARN("Timing of joint trajectory violates Kinova controller max velocities");
    ROS_WARN("This should not occur under normal circumstances.");
    ROS_WARN("Check your MoveIt joint velocity limits (likely in joint_limits.yaml).");
    ROS_WARN_STREAM("The trajectory will be executed using re-calculated timings based on the " <<
             "actual joint velocity limits. This could cause tracking errors and/or collisions.");

    // if we are going to use recomputed time, the trajectory will probably take longer
    // than it was originally planned for. MoveIt has a feature, "execution duration
    // monitoring", where it will automatically end a trajectory if it runs for longer
    // than it was planned to.

    // Normally, execution duration monitoring is fine. But in this case, it could
    // cause the now-lengthened trajectory to be stopped (preempted) before it's
    // completed.

    // Here, we check to see if execution duration monitoring is turned on. If it is,
    // we warn the user of that as well.
    if (ros::service::exists("/move_group/trajectory_execution/set_parameters", false))
    {
      dynamic_reconfigure::ReconfigureRequest req;
      dynamic_reconfigure::ReconfigureResponse resp;

      ros::service::call("/move_group/trajectory_execution/set_parameters", req, resp);

      for (auto const &it : resp.config.bools)
        if (it.name == "execution_duration_monitoring" && it.value)
          ROS_WARN("Warning: Execution duration monitoring turned on. This may cause trajectory to be preempted before completion.");
    }
  }

  return jointVelocitiesCorrected;
}

void JacoTrajectoryController::stopArm() {

  kinova_msgs::JointVelocity trajectoryPoint;
  trajectoryPoint.joint1 = 0.0;
  trajectoryPoint.joint2 = 0.0;
  trajectoryPoint.joint3 = 0.0;
  trajectoryPoint.joint4 = 0.0;
  trajectoryPoint.joint5 = 0.0;
  trajectoryPoint.joint6 = 0.0;
  trajectoryPoint.joint7 = 0.0;

  this->angularCmdPublisher.publish(trajectoryPoint);
}

void JacoTrajectoryController::executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  int numPoints = goal->trajectory.points.size();
  vector< ecl::Array<double> > jointPoints;
  jointPoints.resize(NUM_JACO_JOINTS);
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    jointPoints[i].resize(numPoints);
  }

  ecl::Array<double> timePoints(numPoints);

  // get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {
    bool includedJoints[NUM_JACO_JOINTS] = { };
    timePoints[i] = goal->trajectory.points[i].time_from_start.toSec();

    for (unsigned int trajectoryIndex = 0; trajectoryIndex < goal->trajectory.joint_names.size(); trajectoryIndex++)
    {
      string jointName = goal->trajectory.joint_names[trajectoryIndex];
      int jointIndex = distance(jointNames.begin(), find(jointNames.begin(), jointNames.end(), jointName));
      if (jointIndex >= 0 && jointIndex < NUM_JACO_JOINTS)
      {
        jointPoints[jointIndex][i] = goal->trajectory.points.at(i).positions.at(trajectoryIndex);
        includedJoints[jointIndex] = true;
      }
    }

    // Fill non-included joints with current joint state
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
      if (!includedJoints[j])
        jointPoints[j][i] = jointStates.position[j];
  }

  bool jointVelocitiesCorrected = this->jointVelocityCheck(numPoints, timePoints, jointPoints);

  // Spline the given points to smooth the trajectory
  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(NUM_JACO_JOINTS);

  // Setup cubic storage in case
  vector<ecl::CubicSpline> cubic_splines;
  cubic_splines.resize(NUM_JACO_JOINTS);
  cubic_flag_=false;
  try
  {
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], maxCurvature);
      splines.at(i) = tempSpline;
    }
  }
  catch (...) // This catches ALL exceptions
  {
    cubic_flag_= true;
    ROS_WARN("WARNING: Performing cubic spline rather than smooth linear because of crash");
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      ecl::CubicSpline tempSpline = ecl::CubicSpline::Natural(timePoints, jointPoints[i]);
      cubic_splines.at(i) = tempSpline;
    }
  }

  //control loop
  bool trajectoryComplete = false;
  double startTime = ros::Time::now().toSec();
  double t = 0;
  float error[NUM_JACO_JOINTS];
  float totalError;
  float prevError[NUM_JACO_JOINTS] = {0};
  float integratedError[NUM_JACO_JOINTS] = {0};
  float currentPoint;
  double current_joint_pos[NUM_JACO_JOINTS];
  kinova_msgs::JointVelocity trajectoryPoint;
  ros::Rate rate(100);
  bool reachedFinalPoint;
  ros::Time finalPointTime;

  // Check if we send the trajectory to simulation vs. real robot
  // if sim just send position trajectory and not run the PID loop
  if (sim_flag_)
  {
    ROS_INFO("WARNING: Simulation velocity trajectory is executed as a position trajectory");

    // Setup msg JointTrajectory for std gazebo ros controller
    // Populate JointTrajectory with the points in the current goal
    trajectory_msgs::JointTrajectory jtm;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtm.joint_names = goal->trajectory.joint_names;
    jtm.points = goal->trajectory.points;

    // Publish out trajaectory points listed in the goal
    angCmdSimPublisher.publish(jtm);

    // Wait a second for the arm to move a bit
    ros::Duration(0.1).sleep();

    // Check the total error to determine if the trajectory finished
    totalError = 1.0;
    float prevError = 0.0;
    bool jointError = true;
    while (abs(totalError - prevError) > 0.001 && jointError)
    {
        prevError = totalError;

        // Copy from joint_state publisher to current joints
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        // Compute total error of all joints away from last position
        totalError = 0;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          error[i] = nearest_equivalent(simplify_angle(goal->trajectory.points[numPoints-1].positions[i]),currentPoint) - currentPoint;
          totalError += fabs(error[i]);
          jointError = jointError || error[i] > ERROR_THRESHOLD;
        }

    // Rate to check if error has changed
    ros::Duration(0.1).sleep();
    }

    // Tell the server we finished the trajectory
    ROS_INFO("Simulated trajectory execution complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);

  }
  else
  {

    // Sending to the real robot
    while (!trajectoryComplete)
    {
      //check for preempt requests from clients
      if (smoothTrajectoryServer.isPreemptRequested())
      {
        this->stopArm();
        smoothTrajectoryServer.setPreempted();
        ROS_INFO("Smooth trajectory server preempted by client");
        return;
      }

      //get time for trajectory
      t = ros::Time::now().toSec() - startTime;
      if (t > timePoints.at(timePoints.size() - 1))
      {
        //use final trajectory point as the goal to calculate error until the error
        //is small enough to be considered successful

        if (!reachedFinalPoint)
        {
          reachedFinalPoint = true;
          finalPointTime = ros::Time::now();
        }

        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        bool jointError = false;
        double maxError = 0;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);

          // Check if we're using a cubic or a linear spline
          double splineValue;
          if (cubic_flag_)
          {
            splineValue = (cubic_splines.at(i))(timePoints.at(timePoints.size() - 1));
          }
          else
          {
            splineValue = (splines.at(i))(timePoints.at(timePoints.size() - 1));
          }
          // Now generate the value
          error[i] = nearest_equivalent(simplify_angle(splineValue),
                                        currentPoint) - currentPoint;
          jointError = jointError || fabs(error[i]) > ERROR_THRESHOLD;
        }

        if (!jointError || ros::Time::now() - finalPointTime >= ros::Duration(3.0))
        {
          this->stopArm();
          trajectoryComplete = true;
          ROS_INFO("Trajectory complete!");
          ROS_INFO_STREAM("Steady-State Error: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5]);
          break;
        }
      }
      else
      {
        //calculate error
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          // Check if we're using a cubic or a linear spline
          double splineValue;
          if (cubic_flag_)
          {
            splineValue = (cubic_splines.at(i))(t);
          }
          else
          {
            splineValue = (splines.at(i))(t);
          }
          error[i] = nearest_equivalent(simplify_angle(splineValue), currentPoint) - currentPoint;
        }
      }

      // integrate the error
      for (int i = 0; i < NUM_JACO_JOINTS; ++i)
      {
        integratedError[i] += error[i];
      }

      //calculate control input
      //populate the velocity command
      trajectoryPoint.joint1 = (KP * error[0] + KV * (error[0] - prevError[0] + KI * (integratedError[0])) * RAD_TO_DEG);
      trajectoryPoint.joint2 = (KP * error[1] + KV * (error[1] - prevError[1] + KI * (integratedError[1])) * RAD_TO_DEG);
      trajectoryPoint.joint3 = (KP * error[2] + KV * (error[2] - prevError[2] + KI * (integratedError[2])) * RAD_TO_DEG);
      trajectoryPoint.joint4 = (KP * error[3] + KV * (error[3] - prevError[3] + KI * (integratedError[3])) * RAD_TO_DEG);
      trajectoryPoint.joint5 = (KP * error[4] + KV * (error[4] - prevError[4] + KI * (integratedError[4])) * RAD_TO_DEG);
      trajectoryPoint.joint6 = (KP * error[5] + KV * (error[5] - prevError[5] + KI * (integratedError[5])) * RAD_TO_DEG);
      trajectoryPoint.joint7 = (KP * error[6] + KV * (error[6] - prevError[6] + KI * (integratedError[6])) * RAD_TO_DEG);

      //send the velocity command
      angularCmdPublisher.publish(trajectoryPoint);

      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        prevError[i] = error[i];
      }

      rate.sleep();
      ros::spinOnce();
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hlpr_jaco_trajectory_action_server");

  JacoTrajectoryController jtc;
  ros::spin();
}

