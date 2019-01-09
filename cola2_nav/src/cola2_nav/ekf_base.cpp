/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_base.h"

EKFBase::EKFBase(const unsigned int state_vector_size) : state_vector_size_(state_vector_size)
{
  x_ = Eigen::VectorXd::Zero(state_vector_size_);
  P_ = Eigen::MatrixXd::Identity(state_vector_size_, state_vector_size_);
}

bool EKFBase::makePrediction(const double now)
{
  if (init_ekf_)
  {
    const double dt = now - last_prediction_;  // time increment
    if (dt > 0.0 && dt <= 1.0)
    {
      computePredictionMatrices(dt);
      // x = f(x, u)
      // P = A P AT + W Q WT
      x_ = fx_;
      normalizeState();
      P_ = A_ * P_ * A_.transpose() + W_ * Q_ * W_.transpose();
      last_prediction_ = now;
      return true;
    }
    else if (dt > -0.15 && dt <= 0.0)
    {
      // state is the same
      return true;
    }
    else
    {
      std::cerr << "makePrediction invalid period " << dt << "\n";
      last_prediction_ = now;  // Update time, otherwise everything will fail!
      return false;
    }
  }
  else
  {
    last_prediction_ = now;  // Update time, otherwise everything will fail!
    return false;
  }
}

bool EKFBase::applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                          const Eigen::MatrixXd& V, const double mahalanobis_distance_threshold)
{
  const double distance = mahalanobisDistance(innovation, R, H);
  if (filter_updates_ < 100 || distance < mahalanobis_distance_threshold)
  {
    // Compute updated state vector
    const Eigen::MatrixXd S = H * P_ * H.transpose() + V * R * V.transpose();
    const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ += K * innovation;
    normalizeState();
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_vector_size_, state_vector_size_) - K * H;
    // P_ = (Eigen::MatrixXd::Identity(state_vector_size_, state_vector_size_) - K * H) * P_;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();  // Joseph form
    filter_updates_++;
    // Check integrity
    checkIntegrity();
  }
  else
  {
    return false;
  }
  return true;
}

void EKFBase::showStateVector() const
{
  std::cout << "x: " << x_.transpose() << '\n';
  std::cout << "P:\n" << P_ << '\n';
}

Eigen::VectorXd EKFBase::getStateVector() const
{
  return x_;
}

Eigen::MatrixXd EKFBase::getCovarianceMatrix() const
{
  return P_;
}

Eigen::Affine3d EKFBase::getTransform() const
{
  Eigen::Affine3d trans(getRotation());
  trans.translation() = getPosition();
  return trans;
}

Eigen::Matrix3d EKFBase::getRotation() const
{
  return cola2::utils::euler2quaternion(getEuler()).toRotationMatrix();
}

Eigen::Quaterniond EKFBase::getOrientation() const
{
  return cola2::utils::euler2quaternion(getEuler());
}

bool EKFBase::isInitialized() const
{
  return init_ekf_;
}

double EKFBase::mahalanobisDistance(const Eigen::VectorXd& inno, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H)
{
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::VectorXd d = inno.transpose() * S.inverse() * inno;
  return sqrt(d(0, 0));
}

void EKFBase::checkIntegrity()
{
  // NaN check
  for (unsigned int i = 0; i < state_vector_size_; i++)
  {
    if (std::isnan(x_(i)))
    {
      std::cout << "\033[1;31m"
                << "NaNs detected!!!"
                << "\033[0m\n";
    }
    if (P_(i, i) < 0.0)
    {
      P_(i, i) = 0.01;
      std::cout << "Negative values in P(" << i << "," << i << ")\n";
    }
  }
}
