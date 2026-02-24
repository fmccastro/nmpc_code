cmake_minimum_required(VERSION 3.0.2)
project(nmpc_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  gazebo_dev
  geometry_msgs
  gazebo_msgs
  message_generation
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES contact_steering_plugin
  CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(contact_steering_plugin plugins/contact_steering.cpp)

target_link_libraries(contact_steering_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})