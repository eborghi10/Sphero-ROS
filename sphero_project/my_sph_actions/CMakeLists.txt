cmake_minimum_required(VERSION 2.8.3)
project(my_sph_actions)

## Find catkin macros and libraries
## Is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
  nav_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  sphero_action_msg.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## CATKIN_DEPENDS: catkin_packages dependent projects also need

catkin_package(
  CATKIN_DEPENDS rospy nav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
