cmake_minimum_required(VERSION 2.8.3)
project(cola2_sim)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  rospy
  tf
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  dynamic_reconfigure
  gazebo_msgs
  message_generation
  message_runtime

  # COLA2 Dependencies
  cola2_lib
  cola2_msgs
)

# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
               sensor_msgs
               nav_msgs
               geometry_msgs
               gazebo_msgs
               cola2_msgs
)

# Declare things to be passed to dependent projects
catkin_package()
