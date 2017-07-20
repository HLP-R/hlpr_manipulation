cmake_minimum_required(VERSION 2.8.3)
project(hlpr_manipulation_actions)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    geometry_msgs
    roscpp
    moveit_ros_planning_interface
    pcl_conversions
    pcl_ros
    rail_manipulation_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf2_ros
    wpi_jaco_msgs
    )

find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(hlpr_moveit_wrapper src/hlpr_moveit_wrapper.cpp)
add_executable(common_actions src/common_actions.cpp)
add_executable(primitive_actions src/primitive_actions.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hlpr_moveit_wrapper ${catkin_LIBRARIES})
target_link_libraries(common_actions ${catkin_LIBRARIES})
target_link_libraries(primitive_actions ${catkin_LIBRARIES})

add_dependencies(hlpr_moveit_wrapper rail_manipulation_msgs_gencpp)
add_dependencies(common_actions rail_manipulation_msgs_gencpp)
add_dependencies(primitive_actions rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS hlpr_moveit_wrapper common_actions primitive_actions
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
