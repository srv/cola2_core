/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/controllers/goto.h>

// Constructor
GotoController::GotoController(GotoControllerConfig config) : config_(config)
{
}

void GotoController::setConfig(const GotoControllerConfig& config)
{
  config_ = config;
}

// Compute Method
void GotoController::compute(const control::State& current_state, const control::Waypoint& waypoint,
                             control::State& controller_output, control::Feedback& feedback,
                             control::PointsList& marker)
{
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

  // Compute YAW error
  double inc_x = waypoint.position.north - current_state.pose.position.north;
  double inc_y = waypoint.position.east - current_state.pose.position.east;
  double desired_yaw = std::atan2(inc_y, inc_x);
  double yaw_error = cola2::utils::wrapAngle(desired_yaw - current_state.pose.orientation.yaw);
  double distance = std::sqrt(std::pow(inc_x, 2.0) + std::pow(inc_y, 2.0));

  // Compute SURGE error
  // If current distance to wp > 1/vel_x_k and angle_error < max_error
  // then move to max vel
  double surge = 0.0;
  if (std::fabs(yaw_error) < config_.max_angle_error)
  {
    surge = std::sqrt(std::pow(inc_x, 2) + std::pow(inc_y, 2)) * config_.surge_proportional_gain;
    surge = cola2::utils::saturate(surge, 1.0);
  }

  // Adjust Surge response depending on Yaw error
  surge = surge * (1.0 - (std::fabs(yaw_error) / config_.max_angle_error));

  // Move from 25% to 100% of surge max velocity, not less
  if (surge > 0.0 && surge < 0.25)
  {
    surge = 0.25;
  }

  double velocity = waypoint.linear_velocity.x;
  if (velocity == 0.0 || velocity > config_.max_surge * 3.0)
  {
    velocity = config_.max_surge;
  }
  surge = surge * velocity;

  // Take desired and current Z
  double desired_z = waypoint.position.depth;
  double current_z = current_state.pose.position.depth;
  if (waypoint.altitude_mode)
  {
    desired_z = waypoint.altitude;
    current_z = current_state.pose.altitude;
  }

  // If necessary, check if final position X, Y is reached
  feedback.success = true;
  if (!waypoint.disable_axis.x)
  {  // X-Y tolerance must be checked
    if (std::fabs(current_state.pose.position.north - waypoint.position.north) > waypoint.position_tolerance.x ||
        std::fabs(current_state.pose.position.east - waypoint.position.east) > waypoint.position_tolerance.y)
    {
      feedback.success = false;
    }
  }
  // If necessary, check if final position Z is reached
  if (!waypoint.disable_axis.z && std::fabs(current_z - desired_z) > waypoint.position_tolerance.z)
  {
    feedback.success = false;
  }

  // Set up controller's output and feedback
  // Set desired Z
  if (!waypoint.disable_axis.z)
  {
    controller_output.pose.altitude_mode = waypoint.altitude_mode;
    controller_output.pose.altitude = waypoint.altitude;
    controller_output.pose.position.depth = waypoint.position.depth;
    controller_output.pose.disable_axis.z = false;
    feedback.desired_depth = desired_z;
  }

  // If X-Y motion is enabled
  if (!waypoint.disable_axis.x)
  {
    // Set Surge velocity
    controller_output.velocity.linear.x = surge;
    controller_output.velocity.disable_axis.x = false;
    feedback.desired_surge = surge;
    controller_output.pose.orientation.yaw = desired_yaw;
    controller_output.pose.disable_axis.yaw = false;
    feedback.desired_yaw = desired_yaw;
  }

  // Set yaw if yaw is enabled and X is disabled
  if (waypoint.disable_axis.x && !waypoint.disable_axis.yaw)
  {
    controller_output.pose.orientation.yaw = waypoint.orientation.yaw;
    controller_output.pose.disable_axis.yaw = false;
    feedback.desired_yaw = waypoint.orientation.yaw;
  }

  // Fill additional feedback vars
  feedback.distance_to_end = distance;

  // Fill marker
  control::Point initial_point;
  control::Point final_point;

  initial_point.x = current_state.pose.position.north;
  initial_point.y = current_state.pose.position.east;
  initial_point.z = current_state.pose.position.depth;
  marker.points_list.push_back(initial_point);
  final_point.x = waypoint.position.north;
  final_point.y = waypoint.position.east;
  if (waypoint.altitude_mode)
    final_point.z = current_state.pose.position.depth + (current_state.pose.altitude - waypoint.altitude);
  else
    final_point.z = waypoint.position.depth;

  marker.points_list.push_back(final_point);
}
