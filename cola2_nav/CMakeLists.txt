cmake_minimum_required(VERSION 2.8.3)
project(cola2_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra)

# QtCreator add all files
file(GLOB_RECURSE EXTRA_FILES src/*)
file(GLOB_RECURSE EXTRA_INCLUDES include/${PROJECT_NAME}/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES} ${EXTRA_INCLUDES})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                cola2_msgs
                cola2_lib
                diagnostic_msgs
                geometry_msgs
                nav_msgs
                roscpp
                rospy
                sensor_msgs
                tf
                tf_conversions
                visualization_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cola2_nav
   CATKIN_DEPENDS cola2_msgs
                  cola2_lib
                  diagnostic_msgs
                  geometry_msgs
                  nav_msgs
                  roscpp
                  sensor_msgs
                  tf
                  tf_conversions
                  visualization_msgs
   DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

## Declare a cpp library
add_library(cola2_nav
   src/${PROJECT_NAME}/ekf_base.cpp
   src/${PROJECT_NAME}/ekf_base_landmarks.cpp
   src/${PROJECT_NAME}/ekf_base_landmarks_ros.cpp
   src/${PROJECT_NAME}/ekf_position_velocity_landmarks.cpp
   src/${PROJECT_NAME}/transformations.cpp
)
target_link_libraries(cola2_nav ${catkin_LIBRARIES})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(cola2_nav cola2_lib_gencpp cola2_msgs_generate_messages_cpp)
