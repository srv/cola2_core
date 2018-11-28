/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/controllers/anchor.h>

// Constructor
AnchorController::AnchorController(AnchorControllerConfig config) : config_(config)
{
}

void AnchorController::setConfig(const AnchorControllerConfig& config)
{
  config_ = config;
}

// Compute Method
void AnchorController::compute(const control::State& current_state, const control::Waypoint& waypoint,
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

  // Compute distance to the waypoint
  double robot_distance_2D = std::sqrt(std::pow(waypoint.position.north - current_state.pose.position.north, 2) +
                                       std::pow(waypoint.position.east - current_state.pose.position.east, 2));

  // Yaw
  double desired_yaw = std::atan2(waypoint.position.east - current_state.pose.position.east,
                                  waypoint.position.north - current_state.pose.position.north);

  // Surge
  double error = config_.radius - robot_distance_2D;
  double desired_surge = -config_.kp * error;
  if (desired_surge > config_.max_surge)
  {
    desired_surge = config_.max_surge;
  }
  if (desired_surge < config_.min_surge)
  {
    desired_surge = config_.min_surge;
  }
  if (std::fabs(cola2::utils::wrapAngle(desired_yaw - current_state.pose.orientation.yaw)) > config_.max_angle_error)
  {
    desired_surge = 0.0;
  }

  // Set up controller's output and feedback:
  // Set z...
  controller_output.pose.altitude_mode = waypoint.altitude_mode;
  controller_output.pose.altitude = waypoint.altitude;
  controller_output.pose.position.depth = waypoint.position.depth;
  controller_output.pose.disable_axis.z = false;
  feedback.desired_depth = waypoint.position.depth;
  if (waypoint.altitude_mode)
    feedback.desired_depth = waypoint.altitude;
  // Set surge...
  controller_output.velocity.linear.x = desired_surge;
  controller_output.velocity.disable_axis.x = false;
  feedback.desired_surge = desired_surge;
  // Set yaw...
  controller_output.pose.orientation.yaw = desired_yaw;
  controller_output.pose.disable_axis.yaw = false;
  feedback.desired_yaw = desired_yaw;

  // Fill additional feedback vars
  feedback.distance_to_end = robot_distance_2D;

  // Fill marker
  control::Point initial_point;
  control::Point final_point;

  // Set marker points
  initial_point.x = current_state.pose.position.north;
  initial_point.y = current_state.pose.position.east;
  initial_point.z = current_state.pose.position.depth;
  marker.points_list.push_back(initial_point);
  final_point.x = waypoint.position.north;
  final_point.y = waypoint.position.east;
  final_point.z = waypoint.position.depth;
  if (waypoint.altitude_mode)
    final_point.z = current_state.pose.position.depth + (current_state.pose.altitude - waypoint.altitude);
  marker.points_list.push_back(final_point);

  // Set success to false so that the pilot does not stop keeping position
  // until the timeout expires
  feedback.success = false;
}
