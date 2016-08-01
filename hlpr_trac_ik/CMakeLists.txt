cmake_minimum_required(VERSION 2.8.3)
project(hlpr_trac_ik)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs geometry_msgs trajectory_msgs trac_ik_lib cmake_modules)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")

add_service_files(
  FILES
  IKHandler.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs trajectory_msgs trac_ik_lib
  DEPENDS Boost orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(hlpr_trac_ik src/ik_handler.cpp)
target_link_libraries(hlpr_trac_ik ${catkin_LIBRARIES})
add_dependencies(hlpr_trac_ik hlpr_trac_ik_generate_messages_cpp)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS hlpr_trac_ik
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

