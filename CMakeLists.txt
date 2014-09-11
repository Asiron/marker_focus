cmake_minimum_required(VERSION 2.8.3)
project(marker_focus)

find_package(catkin REQUIRED COMPONENTS
  nao_msgs
  roscpp
  rospy
  std_msgs
  message_filters
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)