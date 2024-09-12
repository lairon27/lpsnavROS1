cmake_minimum_required(VERSION 3.0.2)
project(lpsnav_simulation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/environment_node.py
  src/agent_lpsnav.py
  src/sfm_agent.py
  src/utils.py
  src/helper_functions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch files)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

