cmake_minimum_required(VERSION 3.0.2)
project(cola2_safety)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure

  # COLA2 Dependencies
  cola2_lib
  cola2_msgs
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/SafeDepthAltitude.cfg
  cfg/VirtualCageInfo.cfg
)

# Declare things to be passed to dependent projects
catkin_package()
