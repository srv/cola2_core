/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/controllers/los_cte.h>
#include <cola2_control/controllers/types.h>

// Constructor
LosCteController::LosCteController(LosCteControllerConfig config) : config_(config)
{
}

void LosCteController::setConfig(const LosCteControllerConfig& config)
{
  config_ = config;
}

// Compute Method
void LosCteController::compute(const control::State& current_state, const control::Section& section,
                               control::State& controller_output, control::Feedback& feedback,
                               control::PointsList& marker)
{
  // If the user defines a surge smaller than min_surge
  // the min_surge is not used
  bool use_min_surge = true;

  // Set all axis as disabled by default
  controller_output.pose.disable_axis.x     = true;
  controller_output.pose.disable_axis.y     = true;
  controller_output.pose.disable_axis.z     = true;
  controller_output.pose.disable_axis.roll  = true;
  controller_output.pose.disable_axis.pitch = true;
  controller_output.pose.disable_axis.yaw   = true;
  controller_output.velocity.disable_axis.x     = true;
  controller_output.velocity.disable_axis.y     = true;
  controller_output.velocity.disable_axis.z     = true;
  controller_output.velocity.disable_axis.roll  = true;
  controller_output.velocity.disable_axis.pitch = true;
  controller_output.velocity.disable_axis.yaw   = true;

  // Set variables to zero
  controller_output.pose.position.north    = 0.0;
  controller_output.pose.position.east     = 0.0;
  controller_output.pose.position.depth    = 0.0;
  controller_output.pose.orientation.roll  = 0.0;
  controller_output.pose.orientation.pitch = 0.0;
  controller_output.pose.orientation.yaw   = 0.0;
  controller_output.velocity.linear.x  = 0.0;
  controller_output.velocity.linear.y  = 0.0;
  controller_output.velocity.linear.z  = 0.0;
  controller_output.velocity.angular.x = 0.0;
  controller_output.velocity.angular.y = 0.0;
  controller_output.velocity.angular.z = 0.0;

  // Compute desired surge and yaw
  double surge = std::min(section.surge_velocity, config_.max_surge_velocity);
  double desired_yaw;

  // Distance to current way-point
  double dist_final = std::sqrt(std::pow(section.final_position.x - current_state.pose.position.north, 2) +
                                std::pow(section.final_position.y - current_state.pose.position.east, 2));

  // Distance to previous way-point
  double dist_origin = std::sqrt(std::pow(section.initial_position.x - current_state.pose.position.north, 2) +
                                 std::pow(section.initial_position.y - current_state.pose.position.east, 2));

  // Distance to previous way-point
  double dist_waypoints = std::sqrt(std::pow(section.initial_position.x - section.final_position.x, 2) +
                                    std::pow(section.initial_position.y - section.final_position.y, 2));

  // Compute cross-track error
  // Angle of path
  double beta = std::atan2(section.final_position.y - section.initial_position.y,
                           section.final_position.x - section.initial_position.x);  // Fossen LOS, Sec. 10.3.2

  double sbeta = std::sin(beta);
  double cbeta = std::cos(beta);

  // Along-track distance (s) and cross-track error (e) (rotation)
  double s = (current_state.pose.position.north - section.initial_position.x) * cbeta +
             (current_state.pose.position.east - section.initial_position.y) * sbeta;

  double e = -(current_state.pose.position.north - section.initial_position.x) * sbeta +
             (current_state.pose.position.east - section.initial_position.y) * cbeta;
  feedback.cross_track_error = e;

  // Orthogonal projection
  double x_proj = section.initial_position.x + (s + config_.delta) * cbeta;
  double y_proj = section.initial_position.y + (s + config_.delta) * sbeta;

  // Compute LOS vector
  double LOSX;
  double LOSY;
  if (s < dist_waypoints)
  {
    LOSX = x_proj - current_state.pose.position.north;
    LOSY = y_proj - current_state.pose.position.east;
  }
  else
  {
    LOSX = x_proj - current_state.pose.position.north;
    LOSY = y_proj - current_state.pose.position.east;
  }

  // Compute surge
  // Take the smaller one
  double dist_angle = std::atan2(1 - config_.min_velocity_ratio, config_.distance_to_max_velocity);
  double distance = std::min(dist_final, dist_origin);
  if (distance < config_.distance_to_max_velocity)
  {
    double ratio = config_.min_velocity_ratio + std::tan(dist_angle) * distance;
    surge = surge * ratio;
    if (use_min_surge && surge < config_.min_surge_velocity)
      surge = config_.min_surge_velocity;
  }

  // Compute yaw
  desired_yaw = std::atan2(LOSY, LOSX);

  // define current z according to altitude_mode
  double current_z = current_state.pose.position.depth;
  if (section.altitude_mode)
    current_z = current_state.pose.altitude;

  // Check if final position X, Y is reached
  feedback.success = false;

  if (std::fabs(current_state.pose.position.north - section.final_position.x) < section.tolerance.x &&
      std::fabs(current_state.pose.position.east - section.final_position.y) < section.tolerance.y)
  {
    // If Z axis is disabled success is true...
    if (section.disable_z)
    {
      feedback.success = true;
    }
    // ...otherwise check Z value
    else if (std::fabs(current_z - section.final_position.z) < section.tolerance.z)
    {
      feedback.success = true;
    }
  }

  // Check if the limit line has been crossed. Where the limit line is
  // the line perpendicular to the desired path that pass through the
  // current final_positionwaypoint
  double inc_y = section.final_position.y - section.initial_position.y;
  double m_l = 999999.9;
  if (inc_y != 0.0)
  {
    m_l = -(section.final_position.x - section.initial_position.x) / inc_y;
  }
  double c_l = -m_l * section.final_position.x + section.final_position.y;
  double current_d =
      (m_l * current_state.pose.position.north - current_state.pose.position.east + c_l) / std::sqrt(m_l * m_l + 1);
  double sign = (m_l * section.initial_position.x - section.initial_position.y + c_l) / std::sqrt(m_l * m_l + 1);
  if (sign * current_d < 0.0)
  {
    std::cout << "LOSCTE: beyond the final section waypoint!" << std::endl;

    // The vehicle has crossed the line
    if (section.disable_z)
    {
      // If z is uncontrolled
      feedback.success = true;
    }
    else
    {
      if (std::fabs(current_z - section.final_position.z) < section.tolerance.z)
      {
        // If Z is ok, success = True
        feedback.success = true;
      }
      else
      {
        // Otherwise X and Yaw = 0.0 and success = false
        // wait for z to be True
        desired_yaw = current_state.pose.orientation.yaw;
        surge = 0.0;
        feedback.success = false;
      }
    }
  }

  // There was a problem when final and initial points in the section
  // are the same. This particular case can make the vehicle's drift
  if ((section.initial_position.x == section.final_position.x) &&
      (section.initial_position.y == section.final_position.y))
  {
    std::cout << "LOSCTE: initial and final waypoint are the same!" << std::endl;
    surge = 0.0;
    desired_yaw = current_state.pose.orientation.yaw;
    if (std::fabs(current_z - section.final_position.z) < section.tolerance.z)
    {
      feedback.success = true;
    }
  }

  // Set up controller's output and feedback
  // Set desired yaw
  controller_output.pose.orientation.yaw = desired_yaw;
  controller_output.pose.disable_axis.yaw = false;
  feedback.desired_yaw = desired_yaw;

  // Set desired Z
  double desired_depth = section.final_position.z;

  controller_output.pose.altitude_mode = section.altitude_mode;
  controller_output.pose.altitude = section.final_position.z;
  controller_output.pose.position.depth = desired_depth;
  if (!section.disable_z)
  {
    controller_output.pose.disable_axis.z = false;
  }
  feedback.desired_depth = desired_depth;

  // Set Surge velocity
  controller_output.velocity.linear.x = surge;
  controller_output.velocity.disable_axis.x = false;
  feedback.desired_surge = surge;

  // Fill additional feedback vars
  feedback.distance_to_end = dist_final;


  // Fill marker
  control::Point initial_point;
  control::Point final_point;

  initial_point.x = section.initial_position.x;
  initial_point.y = section.initial_position.y;
  initial_point.z = section.initial_position.z;
  marker.points_list.push_back(initial_point);
  final_point.x = section.final_position.x;
  final_point.y = section.final_position.y;
  final_point.z = section.final_position.z;
  marker.points_list.push_back(final_point);
}
