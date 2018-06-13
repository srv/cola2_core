
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_TYPES__
#define __CONTROLLER_TYPES__

#include <vector>
#include <string>


namespace control
{
  typedef struct
  {
    double x;
    double y;
    double z;
  } Point;

  typedef struct
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  } Vector6d;

  typedef struct
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double altitude;
  } Nav;

  typedef struct
  {
    double roll;
    double pitch;
    double yaw;
  } RPY;

  typedef struct
  {
    double north;
    double east;
    double depth;
  } NED;

  typedef struct
  {
    std::vector<control::Point> points_list;
  } PointsList;

  typedef struct
  {
    std::string requester;
    unsigned int priority;
    bool altitude_mode;
    control::NED position;
    double altitude;
    control::RPY orientation;
    control::Point position_tolerance;
    control::RPY orientation_tolerance;
    control::Point linear_velocity;
    control::RPY angular_velocity;
    unsigned int controller_type;
    unsigned int timeout;
    control::Vector6d disable_axis;
    bool keep_position;
  } Waypoint;

  typedef struct
  {
    control::Point initial_position;
    control::Point final_position;
    bool altitude_mode;
    bool disable_z;
    control::Point tolerance;
    double surge_velocity;
  } Section;

  typedef struct
  {
    control::NED position;
    control::RPY orientation;
    control::Vector6d disable_axis;
    double altitude;
    bool altitude_mode;
  } Pose;

  typedef struct
  {
    control::Point linear;
    control::Point angular;
    control::Vector6d disable_axis;
  } Velocity;

  typedef struct
  {
    control::Pose pose;
    control::Velocity velocity;
  } State;

  typedef struct
  {
    double desired_surge;
    double desired_depth;
    double desired_yaw;

    double cross_track_error;
    double depth_error;
    double yaw_error;
    double distance_to_end;

    bool success;
  } Feedback;
}

#endif /* __CONTROLLER_TYPES__ */
