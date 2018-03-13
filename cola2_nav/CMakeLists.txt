cmake_minimum_required(VERSION 2.8.3)
project(cola2_nav)

add_compile_options(-std=c++11 -Wall -Wextra)

# QtCreator add all files
file(GLOB_RECURSE EXTRA_FILES src/*)
file(GLOB_RECURSE EXTRA_INCLUDES include/${PROJECT_NAME}/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES} ${EXTRA_INCLUDES})

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  roscpp
  tf
  tf_conversions
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  nav_msgs

  # COLA2 Dependencies
  cola2_lib
  cola2_msgs
)

# System dependencies
find_package(Eigen3 REQUIRED)

# TODO: why is this required?
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cola2_nav
  CATKIN_DEPENDS roscpp
                 tf
                 tf_conversions
                 std_msgs
                 std_srvs
                 sensor_msgs
                 geometry_msgs
                 visualization_msgs
                 diagnostic_msgs
                 nav_msgs
                 cola2_lib
                 cola2_msgs
  DEPENDS Eigen3
)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(cola2_nav
  src/${PROJECT_NAME}/ekf_base.cpp
  src/${PROJECT_NAME}/ekf_base_landmarks.cpp
  src/${PROJECT_NAME}/ekf_base_landmarks_ros.cpp
  src/${PROJECT_NAME}/ekf_position_velocity_landmarks.cpp
  src/${PROJECT_NAME}/transformations.cpp
)
target_link_libraries(cola2_nav ${catkin_LIBRARIES})
add_dependencies(cola2_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
