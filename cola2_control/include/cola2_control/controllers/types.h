
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
#include <limits>

namespace control
{
  // Default values
  const double       d_double(std::numeric_limits<double>::quiet_NaN());
  const unsigned int d_uint(std::numeric_limits<unsigned int>::quiet_NaN());
  const bool         d_bool(false);
  const std::string  d_string("Not init");

  class Point
  {
   public:
    double x;
    double y;
    double z;
    Point(): x(d_double), y(d_double), z(d_double) {}
  };

  class Vector6d
  {
   public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    Vector6d(): x(d_double), y(d_double), z(d_double),
                roll(d_double), pitch(d_double),
                yaw(d_double) {}
  };

  class Nav
  {
   public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double altitude;
    Nav(): x(d_double), y(d_double), z(d_double),
           roll(d_double), pitch(d_double), yaw(d_double),
           altitude(d_double) {}
  };

  class RPY
  {
   public:
    double roll;
    double pitch;
    double yaw;
    RPY(): roll(d_double), pitch(d_double), yaw(d_double) {}
  };

  class NED
  {
   public:
    double north;
    double east;
    double depth;
    NED(): north(d_double), east(d_double), depth(d_double) {}
  };

  struct PointsList
  {
    std::vector<control::Point> points_list;
  };

  class Waypoint
  {
   public:
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
    double timeout;
    control::Vector6d disable_axis;
    bool keep_position;
    Waypoint(): requester(d_string), priority(d_uint),
                altitude_mode(d_bool), altitude(d_double),
                controller_type(d_uint), timeout(d_double),
                keep_position(d_bool) {}
  };

  class Section
  {
   public:
    control::Point initial_position;
    control::Point final_position;
    bool altitude_mode;
    bool disable_z;
    control::Point tolerance;
    double surge_velocity;
    double timeout;
    Section(): altitude_mode(d_bool), disable_z(d_bool),
               surge_velocity(d_double), timeout(d_double) {}
  };

  class Pose
  {
   public:
    control::NED position;
    control::RPY orientation;
    control::Vector6d disable_axis;
    double altitude;
    bool altitude_mode;
    Pose(): altitude(d_double), altitude_mode(d_bool) {}
  };

  struct Velocity
  {
    control::Point linear;
    control::Point angular;
    control::Vector6d disable_axis;
  };

  struct State
  {
    control::Pose pose;
    control::Velocity velocity;
  };

  class Feedback
  {
   public:
    double desired_surge;
    double desired_depth;
    double desired_yaw;
    double cross_track_error;
    double depth_error;
    double yaw_error;
    double distance_to_end;
    bool success;
    Feedback(): desired_surge(d_double), desired_depth(d_double),
                desired_yaw(d_double), cross_track_error(d_double),
                depth_error(d_double), yaw_error(d_double),
                distance_to_end(d_double), success(d_bool) {}
  };
}

#endif /* __CONTROLLER_TYPES__ */
