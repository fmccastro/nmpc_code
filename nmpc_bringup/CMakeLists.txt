cmake_minimum_required(VERSION 3.0.2)
project(nmpc_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  nmpc_description
  leo_description
  urdf
  controller_manager
  message_generation
)

add_message_files(
  FILES
  pose3D.msg
  pose3DStamped.msg
  velocity3D.msg
  wheelTrueVelocitiesBodyFrame.msg
  referencePath.msg
  contactForces.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime nmpc_description leo_description
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)