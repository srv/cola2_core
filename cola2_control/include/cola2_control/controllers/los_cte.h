
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __CONTROLLER_LOS_CTE__
#define __CONTROLLER_LOS_CTE__

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
  double min_surge_velocity;
  double max_surge_velocity;
  double min_velocity_ratio;  // from 0 to 1
  double delta;
  double distance_to_max_velocity;
} LosCteControllerConfig;

/**
 * \brief LosCteController class.
 * Computes the surge, heave and yaw motion to follow a control::Section using a
 * Line-of-Sight with Cross-Tracking-Error controller.
 */
class LosCteController
{
private:
  LosCteControllerConfig config_;

public:
  /**
   * Constructor. Requires an LosCteControllerConfig structure.
   */
  LosCteController(LosCteControllerConfig);

  /**
   * Given the current control::State and the desired control::Section
   * computes the action to reach a waypoint.
   */
  void compute(const control::State&, const control::Section&, control::State&, control::Feedback&,
               control::PointsList&);
  /**
   * Set configuration by means of a HolonomicGotoControllerConfig struct.
   */
  void setConfig(const LosCteControllerConfig&);
};

#endif /* __CONTROLLER_LOS_CTE__ */
