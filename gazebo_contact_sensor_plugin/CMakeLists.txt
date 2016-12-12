cmake_minimum_required(VERSION 2.8.3)
project(gazebo_contact_sensor_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  sphero_node
)

find_package (gazebo REQUIRED)
find_package (sphero_node REQUIRED)

catkin_package(
  DEPENDS
  	gazebo_ros
  INCLUDE_DIRS 
  	include
  CATKIN_DEPENDS 
  	gazebo_plugins 
  	gazebo_ros 
  	roscpp 
  	sphero_node
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
                     ${SDFormat_INCLUDE_DIRS}
)


#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/ContactPlugin.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )