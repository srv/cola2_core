
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_

#include <assert.h>
#include <map>
#include <string>
#include <vector>

#include <cola2_control/low_level_controllers/controller_base.h>
#include <cola2_control/low_level_controllers/request.h>

class NDofController
{
private:
  std::vector<IController*> controllers_;
  unsigned int n_dof_;

public:
  NDofController(const unsigned int n_dof = 6);

  void addController(IController* controller);

  void setControllerParams(std::vector<std::map<std::string, double> > params);

  void reset();

  std::vector<double> compute(double time_in_sec, Request req, std::vector<double> feedback);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_
