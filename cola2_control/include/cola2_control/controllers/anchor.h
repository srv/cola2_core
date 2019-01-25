
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_ANCHOR__
#define __CONTROLLER_ANCHOR__

#include <cola2_control/controllers/types.h>
#include <cola2_lib/utils/angles.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

typedef struct
{
  double kp;
  double radius;
  double min_surge;
  double max_surge;
  double max_angle_error;
} AnchorControllerConfig;

/**
 * \brief COLA2 Anchor controller.
 * To keep a non holonomic vehicle in a point.
 */
class AnchorController
{
private:
  AnchorControllerConfig config_;

public:
  /**
   * Constructor. Requires an AnchorControllerConfig structure.
   */
  AnchorController(AnchorControllerConfig);

  /**
   * Given the current control::State and the desired control::Waypoint
   * computes the action to keep the vehicle anchored.
   */
  void compute(const control::State&, const control::Waypoint&, control::State&, control::Feedback&,
               control::PointsList&);

  /**
   * Set configuration by means of a AnchorControllerConfig struct.
   */
  void setConfig(const AnchorControllerConfig&);
};

#endif /* __CONTROLLER_ANCHOR__ */
