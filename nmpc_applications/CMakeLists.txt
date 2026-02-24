cmake_minimum_required(VERSION 3.0.2)
project(nmpc_applications)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  geometry_msgs
  gazebo_msgs
  message_generation
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES contacts_plugin
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories($(Boost_INCLUDE_DIR) ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Declare a C++ library
add_library(contacts_plugin plugins/contacts.cc)
add_library(set_force_links_plugin plugins/setForce2Links.cc)
add_library(get_force_links_plugin plugins/getForce2Links.cc)
add_library(set_torque_joints_plugin plugins/setTorque2Joints.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(contacts_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(get_force_links_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(set_force_links_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(set_torque_joints_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})