cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(sitl_gazebo)

find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  include
)

add_library(gazebo_motor_model_ros SHARED
  src/gazebo_motor_model_ros.cpp
)

ament_target_dependencies(gazebo_motor_model_ros
  rclcp
  ${GAZEBO_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
)

install(TARGETS gazebo_motor_model_ros
  LIBRARY DESTINATION lib
)
