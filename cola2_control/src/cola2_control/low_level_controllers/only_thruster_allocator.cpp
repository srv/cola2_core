
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/only_thruster_allocator.h>

OnlyThrusterAllocator::OnlyThrusterAllocator(unsigned int n_thrusters) :
    n_thrusters_(n_thrusters),
    poly_("thruster_allocator_"),
    is_init_(false)
{
  // Load params
  max_force_thruster_forward_ = 100.0;
  max_force_thruster_backward_ = 70.0;
  thruster_distance_yaw_ = 0.51;
  force_to_thrusters_ratio_ = 100.0;
  asymmetry_ = 1.0;
  Eigen::MatrixXd tcm(6, n_thrusters_);
}

OnlyThrusterAllocator::~OnlyThrusterAllocator()
{
}

void OnlyThrusterAllocator::setParams(const double max_force_thruster_forward,
                                           const double max_force_thruster_backward, const double thruster_distance_yaw,
                                           const double force_to_thrusters_ratio, const double asymmetry,
                                           const std::vector<double> thruster_poly,
                                           const std::vector<double> tcm_values)
{
  std::cout << "OnlyThrusterAllocator set params\n";

  max_force_thruster_forward_ = max_force_thruster_forward;
  max_force_thruster_backward_ = max_force_thruster_backward;
  thruster_distance_yaw_ = thruster_distance_yaw;
  force_to_thrusters_ratio_ = force_to_thrusters_ratio;
  asymmetry_ = asymmetry;

  // Init thruster linearization polynomium
  std::map<std::string, double> params;
  params["n_dof"] = thruster_poly.size();
  for (unsigned int i = 0; i < thruster_poly.size(); i++)
  {
    std::ostringstream s;
    s << i;
    const std::string i_as_string(s.str());
    params[i_as_string] = thruster_poly[i];
  }
  poly_.setParameters(params);

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

Eigen::VectorXd OnlyThrusterAllocator::forceToSetpoint(const Eigen::VectorXd& wrench)
{
  Eigen::VectorXd ret(wrench.size());

  // Compute newtons to setpoints for each thruster
  for (unsigned int i = 0; i < wrench.size(); i++)
  {
    std::cout << "ret " << i << ": " << ret[i] << "\n";

    ret[i] = wrench[i] / force_to_thrusters_ratio_;
    // std::cout << "ret normalized " << i << ": " << ret[i] << "\n";

    // Apply asymmetry
    if (ret[i] < 0.0)
    {
      ret[i] = ret[i] * asymmetry_;
    }
    // std::cout << "ret asymetry " << i << ": " << ret[i] << "\n";

    // Saturate between -1.0 and 1.0
    cola2::utils::saturate(ret[i], 1.0);
    // std::cout << "ret saturate " << i << ": " << ret[i] << "\n";

    // Pass through polynomy
    ret[i] = poly_.compute(0.0, ret[i], 0.0);
    // std::cout << "ret polynomy " << i << ": " << ret[i] << "\n";
  }
  return ret;
}

void OnlyThrusterAllocator::mergeSurgeYaw(double& surge, double& yaw)
{
  /*  If the composition of Surge (v[0]) and Yaw (v[5]) overrides
      the maximum force per thruster,  Yaw is respected and Surge is
      reduced. */

  // Saturate surge
  cola2::utils::saturate(surge, 2.0 * max_force_thruster_forward_, -2.0 * max_force_thruster_backward_);

  // Saturate yaw
  double min_force = std::min(max_force_thruster_forward_, max_force_thruster_backward_);

  cola2::utils::saturate(yaw, 2.0 * min_force * thruster_distance_yaw_);

  // Compose left and rigth thruster forces
  double l_thruster = (surge / 2.0) + (yaw / thruster_distance_yaw_ / 2.0);
  double r_thruster = (surge / 2.0) - (yaw / thruster_distance_yaw_ / 2.0);

  // Check limits
  if (fabs(l_thruster - r_thruster) > max_force_thruster_forward_ + max_force_thruster_backward_)
  {
    if (l_thruster > r_thruster)
    {
      l_thruster = max_force_thruster_forward_;
      r_thruster = -max_force_thruster_backward_;
    }
    else
    {
      r_thruster = max_force_thruster_forward_;
      l_thruster = -max_force_thruster_backward_;
    }
  }
  else
  {
    double diff = fabs(l_thruster - r_thruster);
    if (l_thruster > r_thruster)
    {
      if (l_thruster > max_force_thruster_forward_)
      {
        l_thruster = max_force_thruster_forward_;
        r_thruster = max_force_thruster_forward_ - diff;
      }
      if (r_thruster < -max_force_thruster_backward_)
      {
        r_thruster = -max_force_thruster_backward_;
        l_thruster = -max_force_thruster_backward_ + diff;
      }
    }
    if (r_thruster > l_thruster)
    {
      if (r_thruster > max_force_thruster_forward_)
      {
        r_thruster = max_force_thruster_forward_;
        l_thruster = max_force_thruster_forward_ - diff;
      }
      if (l_thruster < -max_force_thruster_backward_)
      {
        l_thruster = -max_force_thruster_backward_;
        r_thruster = -max_force_thruster_backward_ + diff;
      }
    }
  }

  // Compose again surge force and yaw torque
  yaw = (l_thruster - r_thruster) * thruster_distance_yaw_;
  surge = l_thruster + r_thruster;
}
