
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __ONLYTHRUSTERALLOCATOR_CLASS__
#define __ONLYTHRUSTERALLOCATOR_CLASS__

#include <cola2_control/low_level_controllers/poly.h>
#include <cola2_control/low_level_controllers/request.h>
#include <cola2_lib/utils/saturate.h>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <map>
#include <sstream>

class OnlyThrusterAllocator
{
private:
  unsigned int n_thrusters_;
  double max_force_thruster_positive_;
  double max_force_thruster_negative_;
  double thruster_distance_yaw_;
  Eigen::MatrixXd tcm_inv_;
  Poly poly_positive_;
  Poly poly_negative_;
  bool is_init_;

  /**
   * Compute newtons to setpoints for each thruster
   * @param wrench current force
   * @return setpoint for each thruster
   */
  Eigen::VectorXd forceToSetpoint(Eigen::VectorXd thruster_forces);

  /**
   * Surge and yaw are controlled by the same thrusters. They must be merged with this function that prioritizes
   * yaw over surge if both setpoints can not be achieved.
   * @param surge force
   * @param yaw torque
   */
  void mergeSurgeYaw(double&, double& yaw);

public:
  /**
   * Class constructor
   * @param n_thrusters number of thrusters in the vehicle
   */
  OnlyThrusterAllocator(unsigned int n_thrusters);

  ~OnlyThrusterAllocator();

  void setParams(const double max_force_thruster_positive, const double max_force_thruster_negative,
                 const double thruster_distance_yaw, const std::vector<double> thruster_poly_positive,
                 const std::vector<double> thruster_poly_negative, const std::vector<double> tcm_values);

  /**
   * Computes the setpoint for each thrusters taking into account the force + torque (wrench) to be
   * achieved by the vehicle
   * @param wrench Desired force + troque (6DoFs)
   * @return setpoint for each thruster
   */
  Eigen::VectorXd compute(Request wrench);
};

#endif  // __ONLYTHRUSTERALLOCATOR_CLASS__
