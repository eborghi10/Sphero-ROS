cmake_minimum_required(VERSION 2.8.3)
project(sphero_node)

find_package(catkin REQUIRED message_generation  std_msgs)

add_message_files(FILES
                  SpheroCollision.msg
                 )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
