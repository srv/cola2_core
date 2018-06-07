/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_position_velocity_landmarks.h"

EKFPositionVelocityLandmarks::EKFPositionVelocityLandmarks() : EKFBaseLandmarksROS(6)
{
  // state vector contains [x y z vx vy vz] = size -> 6
}

void EKFPositionVelocityLandmarks::normalizeState()
{
  // only landmarks, no angles in state
  normalizeStateLandmarks();
}

void EKFPositionVelocityLandmarks::computePredictionMatrices(const double dt)
{
  Eigen::Vector3d rpy = getEuler();
  double sr = sin(rpy(0));  // sin(roll)
  double cr = cos(rpy(0));  // cos(roll)
  double sp = sin(rpy(1));  // sin(pitch)
  double cp = cos(rpy(1));  // cos(pitch)
  double sy = sin(rpy(2));  // sin(yaw)
  double cy = cos(rpy(2));  // cos(yaw)
  unsigned int size = state_vector_size_ + LANDMARK_SIZE * getNumberOfLandmarks();
  double dt2 = (dt * dt) / 2.0;

  // A is the jacobian matrix of f(x)
  A_ = Eigen::MatrixXd::Identity(size, size);
  A_(0, 3) = +cp * cy * dt;
  A_(0, 4) = -cr * sy * dt + sr * sp * cy * dt;
  A_(0, 5) = +sr * sy * dt + cr * sp * cy * dt;

  A_(1, 3) = +cp * sy * dt;
  A_(1, 4) = +cr * cy * dt + sr * sp * sy * dt;
  A_(1, 5) = -sr * cy * dt + cr * sp * sy * dt;

  A_(2, 3) = -sp * dt;
  A_(2, 4) = +sr * cp * dt;
  A_(2, 5) = +cr * cp * dt;

  // The noise in the system is a term added to the acceleration:
  // e.g. x[0] = x1 + cos(pitch)*cos(yaw)*(vx1*t +  Eax) *t^2/2)-..
  // then, dEax/dt of x[0] = cos(pitch)*cos(yaw)*t^2/2
  W_ = Eigen::MatrixXd::Zero(size, Q_.rows());
  W_(0, 0) = +cp * cy * dt2;
  W_(0, 1) = -cr * sy * dt2 + sr * sp * cy * dt2;
  W_(0, 2) = +sr * sy * dt2 + cr * sp * cy * dt2;

  W_(1, 0) = +cp * sy * dt2;
  W_(1, 1) = +cr * cy * dt2 + sr * sp * sy * dt2;
  W_(1, 2) = -sr * cy * dt2 + cr * sp * sy * dt2;

  W_(2, 0) = -sp * dt2;
  W_(2, 1) = +sr * cp * dt2;
  W_(2, 2) = +cr * cp * dt2;

  W_(3, 0) = dt;
  W_(4, 1) = dt;
  W_(5, 2) = dt;

  // Add some noise to landmark orientation, otherwise they will not move at all
  for (unsigned int i = 0; i < getNumberOfLandmarks(); ++i)
  {
    W_(9 + i * 6, 0) = 0.005;
    W_(10 + i * 6, 1) = 0.005;
    W_(11 + i * 6, 2) = 0.005;
  }

  // The model takes as state 3D position (x, y, z) and linear velocity (vx, vy, vz).
  // The input is the orientation (roll, pitch yaw) and the linear accelerations (ax, ay, az).
  Eigen::Vector3d vel = getVelocity();
  double vx = vel(0);
  double vy = vel(1);
  double vz = vel(2);
  // Compute Prediction Model with constant velocity
  fx_ = x_;
  fx_(0) += cp * cy * (vx * dt) - cr * sy * (vy * dt) + sr * sp * cy * (vy * dt) + sr * sy * (vz * dt) +
            cr * sp * cy * (vz * dt);
  fx_(1) += cp * sy * (vx * dt) + cr * cy * (vy * dt) + sr * sp * sy * (vy * dt) - sr * cy * (vz * dt) +
            cr * sp * sy * (vz * dt);
  fx_(2) += -sp * (vx * dt) + sr * cp * (vy * dt) + cr * cp * (vz * dt);
  fx_(3) = vx;
  fx_(4) = vy;
  fx_(5) = vz;
}

bool EKFPositionVelocityLandmarks::updatePositionXY(const double t, const Eigen::Vector2d& pose_xy,
                                                    const Eigen::Matrix2d& cov)
{
  // Check usage
  if (!config_.use_gps_data_)
  {
    return false;
  }
  // Update time
  last_gps_time_ = t;
  // Initialization
  if (!init_ekf_)
  {
    x_.head(2) = pose_xy;
    init_gps_ = true;
    ROS_INFO_ONCE("gps init");
    return true;
  }
  // Update
  unsigned int size = state_vector_size_ + LANDMARK_SIZE * getNumberOfLandmarks();
  Eigen::Vector2d h = x_.head(2);                      // h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, size);  // H = dh(x) / dx
  H(0, 0) = 1;
  H(1, 1) = 1;
  return applyUpdate(pose_xy - h, H, cov, Eigen::Matrix2d::Identity(), 30.0);
}
bool EKFPositionVelocityLandmarks::updatePositionZ(const double t, const Eigen::Vector1d& pose_z,
                                                   const Eigen::Matrix1d& cov)
{
  // Update time
  last_depth_time_ = t;
  // Initialization
  if (!init_ekf_)
  {
    x_(2) = pose_z(0);
    init_depth_ = true;
    ROS_INFO_ONCE("depth init");
    return true;
  }
  // Update
  unsigned int size = state_vector_size_ + LANDMARK_SIZE * getNumberOfLandmarks();
  Eigen::Vector1d h = x_.segment<1>(2);                // h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, size);  // H = dh(x) / dx
  H(0, 2) = 1;
  return applyUpdate(pose_z - h, H, cov, Eigen::Matrix1d::Identity(), 30.0);
}
bool EKFPositionVelocityLandmarks::updateOrientation(const double t, const Eigen::Vector3d& rpy,
                                                     const Eigen::Matrix3d& cov)
{
  // Update time
  last_imu_time_ = t;
  // Initialization
  if (!init_ekf_)
  {
    if ((!config_.use_gps_data_ || init_gps_) && (init_depth_) && (init_dvl_))
    {
      init_ekf_ = true;
      ROS_INFO_ONCE("ekf init");
    }
    rpy_ = rpy;
    rpy_cov_ = cov;
    init_imu_ = true;
    ROS_INFO_ONCE("imu init");
    return true;
  }
  // Update (used as true reading)
  rpy_ = rpy;
  rpy_cov_ = cov;
  return true;
}
bool EKFPositionVelocityLandmarks::updateVelocity(const double t, const Eigen::Vector3d& vel,
                                                  const Eigen::Matrix3d& cov, const bool from_dvl)
{
  // Update time
  if (from_dvl)
  {
    last_dvl_time_ = t;
  }
  // Initialization
  if (!init_ekf_)
  {
    if (from_dvl)
    {
      x_.segment<3>(3) = vel;
      init_dvl_ = true;
      ROS_INFO_ONCE("dvl init");
    }
    return true;
  }
  // Update
  unsigned int size = state_vector_size_ + LANDMARK_SIZE * getNumberOfLandmarks();
  Eigen::Vector3d h = x_.segment<3>(3);                // h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, size);  // H = dh(x) / dx
  H(0, 3) = 1;
  H(1, 4) = 1;
  H(2, 5) = 1;
  return applyUpdate(vel - h, H, cov, Eigen::Matrix3d::Identity(), 16.0);
}
bool EKFPositionVelocityLandmarks::updateOrientationRate(const double t, const Eigen::Vector3d& rate,
                                                         const Eigen::Matrix3d& cov)
{
  // Update time
  last_imu_time_ = t;
  // Update (used as true reading)
  ang_vel_ = rate;
  ang_vel_cov_ = cov;
  return true;
}

bool EKFPositionVelocityLandmarks::updateLandmarkMeasure(const double, const Eigen::Vector3d& pose_xyz,
                                                         const Eigen::Vector3d& rpy, const std::string& id,
                                                         const Eigen::Matrix6d& cov)
{
  // Prediction already done
  const unsigned int position = static_cast<unsigned int>(getLandmarkPosition(id));
  // Define measurement
  Eigen::Vector6d z;
  z.head(3) = pose_xyz;
  z.tail(3) = rpy;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(LANDMARK_SIZE, state_vector_size_ + LANDMARK_SIZE * getNumberOfLandmarks());
  H.block<3, 3>(0, 0) = -1 * getRotation().transpose();
  H.block<3, 3>(0, state_vector_size_ + position * LANDMARK_SIZE) = getRotation().transpose();
  H.block<3, 3>(3, state_vector_size_ + position * LANDMARK_SIZE + 3) = Eigen::MatrixXd::Identity(3, 3);

  // If the landmark detector does not fill the orientation covariance
  Eigen::Matrix6d R = cov;
  if (R(3, 3) == 0.0 && R(4, 4) == 0.0 && R(5, 5) == 0.0)
  {
    R(3, 3) = R(0, 0);
    R(4, 4) = R(0, 0);
    R(5, 5) = R(0, 0);
  }
  const Eigen::Matrix6d V = Eigen::MatrixXd::Identity(6, 6);

  // Do the update
  return applyUpdate(normalizeInnovationLandmark(z - H * P_), R, H, V, 25.0);
}

Eigen::Vector3d EKFPositionVelocityLandmarks::getPosition() const
{
  return x_.head(3);
}
Eigen::Vector3d EKFPositionVelocityLandmarks::getVelocity() const
{
  return x_.segment<3>(3);
}
Eigen::Vector3d EKFPositionVelocityLandmarks::getEuler() const
{
  return rpy_;
}
Eigen::Vector3d EKFPositionVelocityLandmarks::getAngularVelocity() const
{
  return ang_vel_;
}
Eigen::Matrix3d EKFPositionVelocityLandmarks::getPositionUncertainty() const
{
  return P_.topLeftCorner(3, 3);
}
Eigen::Matrix3d EKFPositionVelocityLandmarks::getVelocityUncertainty() const
{
  return P_.block<3, 3>(3, 3);
}
Eigen::Matrix3d EKFPositionVelocityLandmarks::getOrientationUncertainty() const
{
  return rpy_cov_;
}
Eigen::Matrix3d EKFPositionVelocityLandmarks::getAngularVelocityUncertainty() const
{
  return ang_vel_cov_;
}
