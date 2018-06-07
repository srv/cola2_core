
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/only_thruster_allocator.h>

OnlyThrusterAllocator::OnlyThrusterAllocator(unsigned int n_thrusters) :
    n_thrusters_(n_thrusters),
    poly_positive_("thruster_allocator_"),
    poly_negative_("thruster_allocator_"),
    is_init_(false)
{
  // Load params
  max_force_thruster_positive_ = 90.0;
  max_force_thruster_negative_ = 100.0;
  thruster_distance_yaw_ = 0.51;
  Eigen::MatrixXd tcm(6, n_thrusters_);
}

OnlyThrusterAllocator::~OnlyThrusterAllocator()
{
}

void OnlyThrusterAllocator::setParams(const double max_force_thruster_positive,
                                      const double max_force_thruster_negative,
                                      const double thruster_distance_yaw,
                                      const std::vector<double> thruster_poly_positive,
                                      const std::vector<double> thruster_poly_negative,
                                      const std::vector<double> tcm_values)
{
  std::cout << "OnlyThrusterAllocator set params\n";

  max_force_thruster_positive_ = max_force_thruster_positive;
  max_force_thruster_negative_ = max_force_thruster_negative;
  thruster_distance_yaw_ = thruster_distance_yaw;

  // Init thruster linearization polynomiums
  std::map<std::string, double> params_positive;
  params_positive["n_dof"] = thruster_poly_positive.size();
  for (unsigned int i = 0; i < thruster_poly_positive.size(); i++)
  {
    std::ostringstream s;
    s << i;
    const std::string i_as_string(s.str());
    params_positive[i_as_string] = thruster_poly_positive[i];
  }
  poly_positive_.setParameters(params_positive);

  std::map<std::string, double> params_negative;
  params_negative["n_dof"] = thruster_poly_negative.size();
  for (unsigned int i = 0; i < thruster_poly_negative.size(); i++)
  {
    std::ostringstream s;
    s << i;
    const std::string i_as_string(s.str());
    params_negative[i_as_string] = thruster_poly_negative[i];
  }
  poly_negative_.setParameters(params_negative);

  // Init TCM inverse
  std::cout << "tcm_values.size(): " << tcm_values.size() << "\n";
  assert(tcm_values.size() == 6 * n_thrusters_);
  Eigen::MatrixXd tcm(6, n_thrusters_);
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < n_thrusters_; j++)
    {
      tcm(i, j) = tcm_values.at(i * n_thrusters_ + j);
    }
  }

  std::cout << "TCM:\n" << tcm << "\n";
  tcm_inv_ = (tcm.transpose() * tcm).inverse() * tcm.transpose();
  std::cout << "TCM inv:\n" << tcm_inv_ << "\n";

  std::cout << "OnlyThrusterAllocator initialized!\n";
  is_init_ = true;
}

Eigen::VectorXd OnlyThrusterAllocator::compute(Request wrench)
{
  if (!is_init_)
    return Eigen::MatrixXd::Zero(n_thrusters_, 1);

  // Take wrench request if disabled false, otherwise, get 0.0.
  // std::cout << "thruster allocator\n" << wrench << "\n";
  Eigen::VectorXd wrench_req(6);
  for (unsigned int i = 0; i < wrench.getDisabledAxis().size(); i++)
  {
    if (wrench.getDisabledAxis().at(i))
    {
      wrench_req[i] = 0.0;
    }
    else
    {
      wrench_req[i] = wrench.getValues().at(i);
    }
  }
  // std::cout << "Wrench:\n" << wrench_req << "\n";

  // Merge Surge and Yaw
  double surge = wrench_req[0];
  double yaw = wrench_req[5];
  mergeSurgeYaw(surge, yaw);
  wrench_req[0] = surge;
  wrench_req[5] = yaw;
  // std::cout << "wrench:\n" << wrench_req << "\n";

  // Multiply wrench by thruster allocation matrix
  Eigen::VectorXd force_per_thruster;
  force_per_thruster = tcm_inv_ * wrench_req;
  // std::cout << "force_per_thruster: \n" << force_per_thruster << "\n";

  // Force to setpoint
  Eigen::VectorXd setpoint = forceToSetpoint(force_per_thruster);
  // std::cout << "setpoint: \n" << setpoint << "\n";

  // Publish
  return setpoint;
}

Eigen::VectorXd OnlyThrusterAllocator::forceToSetpoint(Eigen::VectorXd thruster_forces)
{
  Eigen::VectorXd setpoints = Eigen::VectorXd::Zero(thruster_forces.size());

  // Compute newtons to setpoints for each thruster
  for (Eigen::Index i = 0; i < thruster_forces.size(); ++i)
  {
    if (thruster_forces[i] > 0.0)
    {
      cola2::utils::saturate(thruster_forces[i], max_force_thruster_positive_);
      setpoints[i] = poly_positive_.compute(0.0, thruster_forces[i], 0.0);
    }
    else if (thruster_forces[i] < 0.0)
    {
      cola2::utils::saturate(thruster_forces[i], max_force_thruster_negative_);
      setpoints[i] = -poly_negative_.compute(0.0, -thruster_forces[i], 0.0);
    }

    cola2::utils::saturate(setpoints[i], 1.0);
  }

  return setpoints;
}

void OnlyThrusterAllocator::mergeSurgeYaw(double& surge, double& yaw)
{
  /*  If the composition of Surge (v[0]) and Yaw (v[5]) overrides
      the maximum force per thruster,  Yaw is respected and Surge is
      reduced. */

  // Saturate surge
  cola2::utils::saturate(surge, 2.0 * max_force_thruster_positive_, -2.0 * max_force_thruster_negative_);

  // Saturate yaw
  double min_force = std::min(max_force_thruster_positive_, max_force_thruster_negative_);

  cola2::utils::saturate(yaw, 2.0 * min_force * thruster_distance_yaw_);

  // Compose left and rigth thruster forces
  double l_thruster = (surge / 2.0) + (yaw / thruster_distance_yaw_ / 2.0);
  double r_thruster = (surge / 2.0) - (yaw / thruster_distance_yaw_ / 2.0);

  // Check limits
  if (fabs(l_thruster - r_thruster) > max_force_thruster_positive_ + max_force_thruster_negative_)
  {
    if (l_thruster > r_thruster)
    {
      l_thruster = max_force_thruster_positive_;
      r_thruster = -max_force_thruster_negative_;
    }
    else
    {
      r_thruster = max_force_thruster_positive_;
      l_thruster = -max_force_thruster_negative_;
    }
  }
  else
  {
    double diff = fabs(l_thruster - r_thruster);
    if (l_thruster > r_thruster)
    {
      if (l_thruster > max_force_thruster_positive_)
      {
        l_thruster = max_force_thruster_positive_;
        r_thruster = max_force_thruster_positive_ - diff;
      }
      if (r_thruster < -max_force_thruster_negative_)
      {
        r_thruster = -max_force_thruster_negative_;
        l_thruster = -max_force_thruster_negative_ + diff;
      }
    }
    if (r_thruster > l_thruster)
    {
      if (r_thruster > max_force_thruster_positive_)
      {
        r_thruster = max_force_thruster_positive_;
        l_thruster = max_force_thruster_positive_ - diff;
      }
      if (l_thruster < -max_force_thruster_negative_)
      {
        l_thruster = -max_force_thruster_negative_;
        r_thruster = -max_force_thruster_negative_ + diff;
      }
    }
  }

  // Compose again surge force and yaw torque
  yaw = (l_thruster - r_thruster) * thruster_distance_yaw_;
  surge = l_thruster + r_thruster;
}
