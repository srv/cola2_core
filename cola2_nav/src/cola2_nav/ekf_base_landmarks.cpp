/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_base_landmarks.h"

EKFBaseLandmarks::EKFBaseLandmarks(const unsigned int state_vector_size) : EKFBase(state_vector_size)
{
}

unsigned int EKFBaseLandmarks::getNumberOfLandmarks() const
{
  return number_of_landmarks_;
}

std::string EKFBaseLandmarks::getLandmarkId(const unsigned int position) const
{
  return position2id_.at(position);
}

int EKFBaseLandmarks::getLandmarkPosition(const std::string& id) const
{
  try
  {
    return static_cast<int>(id2position_.at(id));
  }
  catch (std::out_of_range)
  {
    return -1;
  }
}

Eigen::Vector3d EKFBaseLandmarks::getLandmarkPositionVector(const unsigned int& position) const
{
  return x_.segment<3>(state_vector_size_ + LANDMARK_SIZE * position);
}
Eigen::Quaterniond EKFBaseLandmarks::getLandmarkOrientationQuaternion(const unsigned int& position) const
{
  const Eigen::Vector3d rpy = x_.segment<3>(state_vector_size_ + LANDMARK_SIZE * position + 3);
  return cola2::utils::euler2quaternion(rpy);
}

Eigen::Matrix6d EKFBaseLandmarks::getLandmarkUncertainty(const unsigned int& position) const
{
  const unsigned int i = state_vector_size_ + LANDMARK_SIZE * position;
  return P_.block<LANDMARK_SIZE, LANDMARK_SIZE>(i, i);
}

double EKFBaseLandmarks::getLandmarkLastUpdate(const std::string& id) const
{
  try
  {
    return landmark_last_update_.at(id);
  }
  catch (std::out_of_range)
  {
    return -1.0;
  }
}

void EKFBaseLandmarks::setLandmarkLastUpdate(const std::string& id, const double time)
{
  landmark_last_update_[id] = time;
}

void EKFBaseLandmarks::resetLandmarks()
{
  // Clear all landmarks
  number_of_landmarks_ = 0;
  position2id_.clear();
  id2position_.clear();
  landmark_last_update_.clear();
  candidate_landmarks_.clear();
  // Remove from state
  x_ = x_.head(state_vector_size_);
  P_ = P_.topLeftCorner(state_vector_size_, state_vector_size_);
  // Show
  showStateVector();
}

void EKFBaseLandmarks::addLandmark(const Eigen::VectorXd& landmark, const Eigen::MatrixXd& landmark_cov,
                                   const std::string& landmark_id)
{
  // Map landmark name
  id2position_[landmark_id] = number_of_landmarks_;  // new landmark is next position
  position2id_[number_of_landmarks_] = landmark_id;
  const unsigned int position = state_vector_size_ + LANDMARK_SIZE * number_of_landmarks_;
  number_of_landmarks_++;

  // Increase P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(x_.size() + LANDMARK_SIZE, x_.size() + LANDMARK_SIZE);
  P.block(0, 0, x_.size(), x_.size()) = P_;
  // Correlations landmark position and vehicle position
  P.block(position, 0, 3, 3) = getPositionUncertainty();
  P.block(0, position, 3, 3) = getPositionUncertainty();
  const Eigen::Matrix3d m1 =
      getRotation() * landmark_cov.block(0, 0, 3, 3) * getRotation().transpose() + getPositionUncertainty();
  const Eigen::Matrix3d m2 = getRotation() * landmark_cov.block(3, 3, 3, 3) * getRotation().transpose();
  P.block<3, 3>(position, position) = m1;  // TODO: check!!!
  P.block<3, 3>(position + 3, position + 3) = m2;
  // Show
  std::cout << "Old P:\n" << P_ << "\n\n";
  std::cout << "New P:\n" << P << "\n\n";
  P_ = P;  // overwrite

  // Increase state vector
  Eigen::VectorXd x = Eigen::VectorXd::Zero(x_.size() + LANDMARK_SIZE);
  x.head(x_.size()) = x_;
  x.tail(LANDMARK_SIZE) = landmark;
  // Show
  std::cout << "Old x: " << x_.transpose() << '\n';
  std::cout << "New x: " << x.transpose() << '\n';
  x_ = x;  // overwrite
}

Eigen::Vector6d EKFBaseLandmarks::normalizeInnovationLandmark(const Eigen::Vector6d& innovation) const
{
  Eigen::Vector6d normalized = innovation;
  normalized(3) = cola2::utils::wrapAngle(innovation(3));
  normalized(4) = cola2::utils::wrapAngle(innovation(4));
  normalized(5) = cola2::utils::wrapAngle(innovation(5));
  return normalized;
}

void EKFBaseLandmarks::normalizeStateLandmarks()
{
  for (unsigned int i = 0; i < number_of_landmarks_; ++i)
  {
    x_(state_vector_size_ + 3 + i * 6) = cola2::utils::wrapAngle(x_(state_vector_size_ + 3 + i * 6));
    x_(state_vector_size_ + 4 + i * 6) = cola2::utils::wrapAngle(x_(state_vector_size_ + 4 + i * 6));
    x_(state_vector_size_ + 5 + i * 6) = cola2::utils::wrapAngle(x_(state_vector_size_ + 5 + i * 6));
  }
}

// *****************************************
// Candidates
// *****************************************
bool EKFBaseLandmarks::isLandmarkCandidate(const std::string& id) const
{
  try
  {
    candidate_landmarks_.at(id);
    return true;
  }
  catch (std::out_of_range)
  {
    return false;
  }
}

bool EKFBaseLandmarks::addLandmarkCandidate(const std::string& id, const Eigen::Vector6d& candidate)
{
  if (isLandmarkCandidate(id))
  {
    candidate_landmarks_[id].push_back(candidate);
    return false;
  }
  candidate_landmarks_[id] = { candidate };
  return true;  // is new
}

bool EKFBaseLandmarks::isValidCandidate(const std::string& candidate_id) const
{
  const std::vector<Eigen::VectorXd> candidates = candidate_landmarks_.at(candidate_id);
  std::cout << "Is the candidate good enough to enter in the filter?\n";
  if (candidates.size() >= LANDMARK_CANDIDATES_MIN)
  {
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(LANDMARK_SIZE, LANDMARK_CANDIDATES_MIN);
    unsigned int i = static_cast<unsigned int>(candidates.size()) - LANDMARK_CANDIDATES_MIN;  // check the last ones
    unsigned int j = 0;
    for (; i < candidates.size(); ++i)
    {
      mat.block(0, j, LANDMARK_SIZE, 1) = candidates.at(i);  // put in columns
      ++j;
    }
    std::cout << "last candidates are:\n" << mat << "\n";

    // TODO: Check that these equations work.
    Eigen::MatrixXd centered = mat.colwise() - mat.rowwise().mean();
    Eigen::MatrixXd cov = (centered * centered.adjoint()) / static_cast<double>(LANDMARK_CANDIDATES_MIN);
    std::cout << "Covariance: \n" << cov.diagonal() << "\n";
    for (unsigned int k = 0; k < LANDMARK_SIZE; ++k)
    {
      if (cov(k, k) > MAX_COVARIANCE)
      {
        return false;
      }
    }
    std::cout << "The candidate is valid!\n";
    return true;
  }
  return false;
}
