/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_GOTO__
#define __CONTROLLER_GOTO__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cola2_control/controllers/types.h>
#include <cola2_lib/utils/angles.h>
#include <cola2_lib/utils/saturate.h>

typedef struct
{
  double surge_proportional_gain;
  double max_angle_error;
  double max_surge;
} GotoControllerConfig;

/**
 * \brief GotoController class.
 * Computes the surge, heave and yaw motion to reach a waypoint.
 */
class GotoController
{

private:
  GotoControllerConfig config_;

public:
  /**
   * Constructor. Requires an GotoControllerConfig structure.
   */
  GotoController(GotoControllerConfig);

  /**
   * Given the current control::State and the desired control::Waypoint
   * computes the action to reach a waypoint.
   */
  void compute(const control::State&, const control::Waypoint&, control::State&, control::Feedback&,
               control::PointsList&);

  /**
   * Set configuration by means of a AnchorControllerConfig struct.
   */
  void setConfig(const GotoControllerConfig&);

};

#endif /* __CONTROLLER_GOTO__ */