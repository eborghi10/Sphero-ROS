cmake_minimum_required(VERSION 2.8.3)
project(sphero_gazebo)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin package

find_package(catkin REQUIRED COMPONENTS
  xacro
  angles
  control_toolbox
  gazebo_ros
  robot_controllers
  robot_controllers_interface
  sphero_node
)

find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
INCLUDE_DIRS include
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
    sphero_node
  DEPENDS
    gazebo_ros
  LIBRARIES
	gazebo_sphero_controller
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  /opt/ros/latest/lib
  /opt/ros/kinetic/lib
  ${GAZEBO_LIBRARY_DIRS}
)


## Declare a C++ library
add_library(gazebo_sphero_controller src/gazebo_sphero_controller.cpp)
target_link_libraries(
	gazebo_sphero_controller 
	gazebo_ros_utils 
	${GAZEBO_LIBRARIES} 
	${catkin_LIBRARIES} 
	${Boost_LIBRARIES}
)
