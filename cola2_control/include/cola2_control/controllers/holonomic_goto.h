/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_HOLONOMIC_GOTO__
#define __CONTROLLER_HOLONOMIC_GOTO__

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cola2_control/controllers/types.h>
#include <cola2_lib/utils/angles.h>

typedef struct
{
  double no_params;
} HolonomicGotoControllerConfig;

/**
 * \brief HolonomicGotoController class.
 * Computes the surge, sway, heave and yaw motion to reach a waypoint.
 */
class HolonomicGotoController
{
private:
  HolonomicGotoControllerConfig config_;

public:
  /**
   * Constructor. Requires an HolonomicGotoControllerConfig structure.
   */
  HolonomicGotoController(HolonomicGotoControllerConfig);

  /**
   * Given the current control::State and the desired control::Waypoint
   * computes the action to reach a waypoint.
   */
  void compute(const control::State&, const control::Waypoint&, control::State&, control::Feedback&,
               control::PointsList&);

  /**
   * Set configuration by means of a HolonomicGotoControllerConfig struct.
   */
  void setConfig(const HolonomicGotoControllerConfig&);
};

#endif /* __CONTROLLER_HOLONOMIC_GOTO__ */
