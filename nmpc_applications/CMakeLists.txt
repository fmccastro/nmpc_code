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
  #LIBRARIES nmpc_applications
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories($(Boost_INCLUDE_DIR) ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} plugins/contacts.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})