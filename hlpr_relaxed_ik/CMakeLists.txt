cmake_minimum_required(VERSION 2.8.3)
project(hlpr_relaxed_ik)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  relaxed_ik
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  sensor_msgs
)

add_service_files(
  FILES
  IKHandler.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  trajectory_msgs
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime geometry_msgs relaxed_ik roscpp rospy std_msgs trajectory_msgs sensor_msgs
)

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

