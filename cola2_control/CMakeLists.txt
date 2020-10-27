cmake_minimum_required(VERSION 3.0.2)
project(cola2_control)

add_compile_options(-std=c++11 -Wall -Wextra)

# QtCreator add all files
file(GLOB_RECURSE EXTRA_FILES src/*)
file(GLOB_RECURSE EXTRA_INCLUDES include/${PROJECT_NAME}/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES} ${EXTRA_INCLUDES})

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  cmake_modules
  roscpp
  rosparam
  rospy
  roslib
  tf
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  actionlib
  dynamic_reconfigure

  # COLA2 Dependencies
  cola2_lib
  cola2_msgs
)

# System dependencies
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED)

# This package has a setup.py. This macro ensures modules and global scripts
# declared therein get installed
catkin_python_setup()


generate_dynamic_reconfigure_options(
        cfg/controller_only_thrusters.cfg
)

# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controllers mission_utils low_level_controllers ros_control_base
  # TODO: go and check what is in the headers
  CATKIN_DEPENDS roscpp
                 roslib
                 std_msgs
                 sensor_msgs
                 actionlib
                 cola2_lib
                 cola2_msgs
  DEPENDS TinyXML Boost
)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(controllers SHARED
  src/${PROJECT_NAME}/controllers/anchor.cpp
  src/${PROJECT_NAME}/controllers/goto.cpp
  src/${PROJECT_NAME}/controllers/holonomic_goto.cpp
  src/${PROJECT_NAME}/controllers/los_cte.cpp
)
target_link_libraries(controllers ${catkin_LIBRARIES})
add_dependencies(controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(mission_utils SHARED
  src/${PROJECT_NAME}/mission_utils/mission_maneuver.cpp
  src/${PROJECT_NAME}/mission_utils/mission_position.cpp
  src/${PROJECT_NAME}/mission_utils/mission_tolerance.cpp
  src/${PROJECT_NAME}/mission_utils/mission_waypoint.cpp
  src/${PROJECT_NAME}/mission_utils/mission_section.cpp
  src/${PROJECT_NAME}/mission_utils/mission_park.cpp
  src/${PROJECT_NAME}/mission_utils/mission_configuration.cpp
  src/${PROJECT_NAME}/mission_utils/mission_action.cpp
  src/${PROJECT_NAME}/mission_utils/mission_step.cpp
  src/${PROJECT_NAME}/mission_utils/mission.cpp
)
target_link_libraries(mission_utils ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
add_dependencies(mission_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(low_level_controllers SHARED
  src/${PROJECT_NAME}/low_level_controllers/auv_controller_base.cpp
  src/${PROJECT_NAME}/low_level_controllers/controller_base.cpp
  src/${PROJECT_NAME}/low_level_controllers/merge.cpp
  src/${PROJECT_NAME}/low_level_controllers/ndof_controller.cpp
  src/${PROJECT_NAME}/low_level_controllers/pid.cpp
  src/${PROJECT_NAME}/low_level_controllers/poly.cpp
  src/${PROJECT_NAME}/low_level_controllers/request.cpp
)
target_link_libraries(low_level_controllers ${catkin_LIBRARIES})
add_dependencies(low_level_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(ros_control_base SHARED
  src/${PROJECT_NAME}/ros_controller/auv_ros_controller.cpp
)
target_link_libraries(ros_control_base ${catkin_LIBRARIES} low_level_controllers)
add_dependencies(ros_control_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(only_thrusters_controller_lib SHARED
  src/${PROJECT_NAME}/low_level_controllers/only_thrusters_controller.cpp
  src/${PROJECT_NAME}/low_level_controllers/only_thruster_allocator.cpp
)
target_link_libraries(only_thrusters_controller_lib ${catkin_LIBRARIES} low_level_controllers)
add_dependencies(only_thrusters_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Add executables
add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pilot src/pilot_node.cpp)
target_link_libraries(pilot ${catkin_LIBRARIES} ${Boost_LIBRARIES} controllers low_level_controllers)
add_dependencies(pilot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(captain src/captain_node.cpp)
target_link_libraries(captain ${catkin_LIBRARIES} ${Boost_LIBRARIES} mission_utils)
add_dependencies(captain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(external_mission_example src/external_mission_example_node.cpp)
target_link_libraries(external_mission_example ${catkin_LIBRARIES} ${Boost_LIBRARIES} mission_utils)
add_dependencies(external_mission_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_only_thrusters src/only_thrusters_controller_node.cpp)
target_link_libraries(controller_only_thrusters ${catkin_LIBRARIES} controllers low_level_controllers ros_control_base only_thrusters_controller_lib)
add_dependencies(controller_only_thrusters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Mark executable scripts (Python etc.) for installation. In contrast to
# setup.py, you can choose the destination
install(PROGRAMS
  src/${PROJECT_NAME}/joystickbase.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
