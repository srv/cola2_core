/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_BASE_LANDMARKS_H_
#define COLA2_NAV_EKF_BASE_LANDMARKS_H_

#include <cola2_lib/utils/angles.h>
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "./ekf_base.h"

namespace
{
constexpr int LANDMARK_SIZE = 6;
constexpr int LANDMARK_CANDIDATES_MIN = 4;
}

/**
 *  \brief Add operations with landmarks to the EKFBase class (all landmarks are size 6).
 */
class EKFBaseLandmarks : public EKFBase
{
private:
  unsigned int number_of_landmarks_ = 0;                // number of landmarks in state vector
  std::map<std::string, unsigned int> id2position_;     // from message id to position in state vector
  std::map<unsigned int, std::string> position2id_;     // from position in state vector to message id
  std::map<std::string, double> landmark_last_update_;  // save last time a landmark was seen
  std::map<std::string, std::vector<Eigen::VectorXd>> candidate_landmarks_;  // candidates observed

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   *  \brief EKF base with landmark operations.
   */
  explicit EKFBaseLandmarks(const unsigned int state_vector_size);
  /**
   *  \brief Virtual destructor.
   */
  virtual ~EKFBaseLandmarks() = default;

  // *****************************************
  // Landmarks on state vector
  // *****************************************
  unsigned int getNumberOfLandmarks() const;
  int getLandmarkPosition(const std::string& id) const;
  std::string getLandmarkId(const unsigned int position) const;
  Eigen::Vector3d getLandmarkPositionVector(const unsigned int& position) const;
  Eigen::Quaterniond getLandmarkOrientationQuaternion(const unsigned int& position) const;
  Eigen::Matrix6d getLandmarkUncertainty(const unsigned int& position) const;
  double getLandmarkLastUpdate(const std::string& id) const;
  void setLandmarkLastUpdate(const std::string& id, const double time);

  // *****************************************
  // Modify landmarks
  // *****************************************
  void resetLandmarks();
  void addLandmark(const Eigen::VectorXd& landmark, const Eigen::MatrixXd& landmark_cov,
                   const std::string& landmark_id);
  void normalizeStateLandmarks();
  Eigen::Vector6d normalizeInnovationLandmark(const Eigen::Vector6d& innovation) const;

  // *****************************************
  // Candidates
  // *****************************************
  /**
   * \brief Check if a landmark id is a candidate.
   */
  bool isLandmarkCandidate(const std::string& id) const;
  /**
   * \brief Add a landmark to the candidates list and return true if it is the first one with that id.
   */
  bool addLandmarkCandidate(const std::string& id, const Eigen::Vector6d& candidate);
  /**
   * \brief Check if a landmark id is a valid candidate to be included in the state vector.
   */
  bool isValidCandidate(const std::string& candidate_id) const;

  // *****************************************
  // To be implemented
  // *****************************************
protected:
  // From EKFBase
  virtual void computePredictionMatrices(const double dt) = 0;
  virtual void normalizeState() = 0;
  virtual bool updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov) = 0;
  virtual bool updatePositionZ(const double t, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov) = 0;
  virtual bool updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov) = 0;
  virtual bool updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                              const bool from_dvl = true) = 0;
  virtual bool updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov) = 0;

public:
  // From EKFBaseLandmarks
  /**
   *  \brief Update state from a landmark measurement.
   */
  virtual bool updateLandmarkMeasure(const double t, const Eigen::Vector3d& pose_xyz, const Eigen::Vector3d& rpy,
                                     const std::string& id, const Eigen::Matrix6d& cov) = 0;
  // From EKFBase
  virtual Eigen::Vector3d getPosition() const = 0;
  virtual Eigen::Vector3d getVelocity() const = 0;
  virtual Eigen::Vector3d getEuler() const = 0;
  virtual Eigen::Vector3d getAngularVelocity() const = 0;
  virtual Eigen::Matrix3d getPositionUncertainty() const = 0;
  virtual Eigen::Matrix3d getVelocityUncertainty() const = 0;
  virtual Eigen::Matrix3d getOrientationUncertainty() const = 0;
  virtual Eigen::Matrix3d getAngularVelocityUncertainty() const = 0;
};

#endif  // COLA2_NAV_EKF_BASE_LANDMARKS_H_
