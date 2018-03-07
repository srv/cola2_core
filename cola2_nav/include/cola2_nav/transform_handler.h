/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_TRANSFORM_HANDLER_H_
#define COLA2_NAV_TRANSFORM_HANDLER_H_

#include <cola2_lib/rosutils/this_node.h>
#include <cola2_lib/utils/angles.h>
#include <tf/transform_listener.h>  // TODO: move to TF2
#include <Eigen/Dense>
#include <map>
#include <string>

/**
 * \brief The TransformHandler class queries and saves transforms with origin at the vehicle frame.
 */
class TransformHandler
{
private:
  std::map<std::string, Eigen::Affine3d> transforms_;  //!< transforms from the robot to the sensors
  tf::TransformListener tf_listener_;                  //!< transform listener
  std::string frame_vehicle_;                          //!< vehicle frame

  /**
   * \brief Convert from ROS TFs to Eigen::Affine3d.
   */
  Eigen::Affine3d tfTransformToEigen(const tf::StampedTransform& trans) const;

public:
  /**
   * \brief Constructor that queries the namespace to know vehicle frame.
   */
  TransformHandler();
  /**
   * \brief Get a static transform from the map or query the listener and save.
   */
  bool getTransform(const std::string& frame, Eigen::Affine3d& transform);
  /**
   * \brief Get a transform by querying the listener.
   */
  bool getDynamicTransform(const std::string& frame, Eigen::Affine3d& transform);
};

#endif  // COLA2_NAV_TRANSFORM_HANDLER_H_
