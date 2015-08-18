cmake_minimum_required(VERSION 2.8.3)
project(nubot_common)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
)


add_message_files(DIRECTORY msgs FILES 
VelCmd.msg
)

add_service_files(DIRECTORY srv FILES BallHandle.srv Shoot.srv)

generate_messages(DEPENDENCIES std_msgs)

set(nubot_common_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/core/include 
)
include_directories(${nubot_common_includes})
catkin_package(
  INCLUDE_DIRS ${nubot_common_includes}
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/core/include
)
