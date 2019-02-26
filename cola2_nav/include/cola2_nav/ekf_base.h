
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_BASE_H
#define COLA2_NAV_EKF_BASE_H

#include <cola2_lib/utils/angles.h>
#include <Eigen/Dense>
#include <iostream>

namespace
{
constexpr double MAX_COVARIANCE = 0.025;
}

namespace Eigen
{
using Vector1d = Matrix<double, 1, 1>;
using Matrix1d = Matrix<double, 1, 1>;
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
}

/**
*  \brief EKF base class with all basic equations.
*/
class EKFBase
{
private:
  /**
   * \brief Compute Mahalanobis distance for the innovation.
   */
  double mahalanobisDistance(const Eigen::VectorXd& inno, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);
  /**
   *  \brief cehck integrity of the state vector and the diagonal of the covariance matrix.
   */
  void checkIntegrity();

protected:
  unsigned int state_vector_size_;  // size of state vector (not including landmarks)
  Eigen::VectorXd x_;               // state vector
  Eigen::MatrixXd P_;               // covariance matrix
  Eigen::MatrixXd Q_;               // noise prediction matrix
  bool init_ekf_ = false;           // filter initialized
  size_t filter_updates_ = 0;       // number of updates already done
  double last_prediction_ = 0.0;    // time of last prediction

  Eigen::VectorXd fx_;  // computePredictionMatrices result
  Eigen::MatrixXd A_;   // computePredictionMatrices result
  Eigen::MatrixXd W_;   // computePredictionMatrices result

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   *  \brief EKF base where the basic equations are applied.
   */
  explicit EKFBase(const unsigned int state_vector_size);
  /**
   *  \brief EKF virtual destructor.
   */
  virtual ~EKFBase() = default;

  // *****************************************
  // Basic EKF
  // *****************************************
  /**
   *  \brief Predict the filter until the specified time.
   */
  bool makePrediction(const double now);
  /**
   *  \brief Update the filter according to a measurement.
   */
  bool applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                   const Eigen::MatrixXd& V, const double mahalanobis_distance_threshold);

  // *****************************************
  // Getters
  // *****************************************
  /**
   *  \brief Print state vector x and covariance matrix onscreen.
   */
  void showStateVector() const;
  /**
   *  \brief Get state vector x.
   */
  Eigen::VectorXd getStateVector() const;
  /**
   *  \brief Get covariance matrix P.
   */
  Eigen::MatrixXd getCovarianceMatrix() const;
  /**
   *  \brief Get transform world to vehicle.
   */
  Eigen::Affine3d getTransform() const;
  /*!
   * \brief Get rotation matrix according to filter orientation.
   */
  Eigen::Matrix3d getRotation() const;
  /*!
   * \brief Get orientation quaternion according to filter orientation.
   */
  Eigen::Quaterniond getOrientation() const;
  /*!
   * \brief Return whether the ekf filter is initialized.
   */
  bool isInitialized() const;

  // *****************************************
  // To be implemented
  // *****************************************
protected:
  /**
   *  \brief Compute prediction matrices fx, A, W.
   */
  virtual void computePredictionMatrices(const double dt) = 0;
  /**
   *  \brief Normalize state vector x (when it has orientation).
   */
  virtual void normalizeState() = 0;
  /**
   *  \brief Update state from a GPS measurement.
   */
  virtual bool updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov) = 0;
  /**
   *  \brief Update state from a Depth measurement.
   */
  virtual bool updatePositionZ(const double t, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov) = 0;
  /**
   *  \brief Update state from a Orientation measurement.
   */
  virtual bool updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov) = 0;
  /**
   *  \brief Update state from a DVL measurement.
   */
  virtual bool updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                              const bool from_dvl = true) = 0;
  /**
   *  \brief Update state from a OrientationRate measurement.
   */
  virtual bool updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov) = 0;

public:
  /**
   *  \brief Get position.
   */
  virtual Eigen::Vector3d getPosition() const = 0;
  /**
   *  \brief Get velocity.
   */
  virtual Eigen::Vector3d getVelocity() const = 0;
  /**
   *  \brief Get euler orientation.
   */
  virtual Eigen::Vector3d getEuler() const = 0;
  /**
   *  \brief Get angular velocity.
   */
  virtual Eigen::Vector3d getAngularVelocity() const = 0;
  /**
   *  \brief Get position uncertainty.
   */
  virtual Eigen::Matrix3d getPositionUncertainty() const = 0;
  /**
   *  \brief Get velocity uncertainty.
   */
  virtual Eigen::Matrix3d getVelocityUncertainty() const = 0;
  /**
   *  \brief Get orientation uncertainty.
   */
  virtual Eigen::Matrix3d getOrientationUncertainty() const = 0;
  /**
   *  \brief Get angular velocity uncertainty.
   */
  virtual Eigen::Matrix3d getAngularVelocityUncertainty() const = 0;
};

#endif  // COLA2_NAV_EKF_BASE_H
