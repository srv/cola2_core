cmake_minimum_required(VERSION 2.8.3)
project(cola2_log)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  diagnostic_msgs

  # COLA2 Dependencies
  cola2_lib
)

# Declare things to be passed to dependent projects
catkin_package()
