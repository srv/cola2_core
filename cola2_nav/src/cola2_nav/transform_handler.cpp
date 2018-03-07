/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/transform_handler.h"

TransformHandler::TransformHandler()
{
  frame_vehicle_ = cola2::rosutils::getNamespace() + std::string("/base_link");
}

bool TransformHandler::getTransform(const std::string& frame, Eigen::Affine3d& transform)
{
  // Return identity when frame is vehicle frame
  if (std::strcmp(frame.c_str(), frame_vehicle_.c_str()) == 0)
  {
    transform = Eigen::Affine3d::Identity();
    return true;
  }
  // Look for the transform
  try
  {
    // Found in map
    transform = transforms_.at(frame);
    return true;
  }
  catch (std::out_of_range)
  {
    // Need to query it
    if (getDynamicTransform(frame, transform))
    {
      transforms_[frame] = transform;
      ROS_INFO_STREAM("Transform Handler added '" << frame << '\n');
      ROS_INFO_STREAM("trans: " << transform.translation().transpose() << '\n');
      ROS_INFO_STREAM("rpy:   " << cola2::utils::rotation2euler(transform.rotation()).transpose() << '\n');
      return true;
    }
  }
  return false;
}

bool TransformHandler::getDynamicTransform(const std::string& frame, Eigen::Affine3d& transform)
{
  // Return identity when frame is vehicle frame
  if (std::strcmp(frame.c_str(), frame_vehicle_.c_str()) == 0)
  {
    transform = Eigen::Affine3d::Identity();
    return true;
  }
  // Look for the transform
  try
  {
    tf::StampedTransform tf_trans;
    tf_listener_.waitForTransform(frame_vehicle_, frame, ros::Time(0), ros::Duration(0.5));
    tf_listener_.lookupTransform(frame_vehicle_, frame, ros::Time(0), tf_trans);
    transform = tfTransformToEigen(tf_trans);
    return true;
  }
  catch (tf::TransformException ex)
  {
    ROS_FATAL_STREAM("Unable to get dynamic transform: " << frame << '\n');
  }
  return false;
}

Eigen::Affine3d TransformHandler::tfTransformToEigen(const tf::StampedTransform& trans) const
{
  const Eigen::Quaterniond quat(trans.getRotation().getW(), trans.getRotation().getX(), trans.getRotation().getY(),
                                trans.getRotation().getZ());
  Eigen::Affine3d affine(quat.toRotationMatrix());
  affine.translation() = Eigen::Vector3d(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
  return affine;
}
