cmake_minimum_required(VERSION 2.8.3)
project(mx_demo)

find_package(catkin REQUIRED COMPONENTS
  mx_controllers
  roscpp
  rospy
  std_msg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

