/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_base_landmarks_ros.h"

// *****************************************
// Constructor and destructor
// *****************************************
EKFBaseLandmarksROS::EKFBaseLandmarksROS(const unsigned int state_vector_size)
  : EKFBaseLandmarks(state_vector_size)
  , ned_(0.0, 0.0, 0.0)
  , diag_help_(nh_, cola2::rosutils::getUnresolvedNodeName(), "software")
{
  // Get correct namespace and vehicle frame
  ns_ = cola2::rosutils::getNamespace();  // nh_.getNamespace();
  frame_vehicle_ = ns_ + std::string("/base_link");
  ROS_INFO("namespace: %s", ns_.c_str());  // TODO: delete
  ROS_INFO("vehicle frame: %s", frame_vehicle_.c_str());

  // Load configurations and reset filter
  getConfig(true);
  resetFilter();

  // Debug in output
  if (config_.enable_debug_)
  {
    // Current UTC time
    time_t t = time(nullptr);     // get time now
    struct tm* now = gmtime(&t);  // UTC time
    char buffer[200];
    strftime(buffer, sizeof(buffer), "/debug_navigator_%Y-%m-%d_%H-%M-%S.txt", now);
    // Create debug file
    ofh_ = std::ofstream(std::string(std::getenv("HOME")) + std::string(buffer));
    ofh_.setf(std::ios::fixed, std::ios::floatfield);
    ofh_.precision(4);
  }

  // Publishers
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  pub_map_ = nh_.advertise<cola2_msgs::Map>("landmarks", 1);
  pub_nav_ = nh_.advertise<cola2_msgs::NavSts>("navigation", 1);  // TODO change type
  pub_gps_ned_ = nh_.advertise<geometry_msgs::PoseStamped>("gps_ned", 1);
  pub_usbl_ned_ = nh_.advertise<geometry_msgs::PoseStamped>("usbl_ned", 1);
  pub_range_update_ = nh_.advertise<visualization_msgs::Marker>("markers/range_update", 1);
  pub_landmarks_ = nh_.advertise<visualization_msgs::MarkerArray>("markers/landmarks", 1);
  pub_altitude_ = nh_.advertise<sensor_msgs::Range>("altitude_filtered", 1);

  // Init services
  // clang-format off
  srv_reload_params_ = nh_.advertiseService("reload_params", &EKFBaseLandmarksROS::srvResetNavigation, this);
  srv_reset_navigation_ = nh_.advertiseService("reset_navigation", &EKFBaseLandmarksROS::srvResetNavigation, this);
  srv_reset_landmarks_ = nh_.advertiseService("reset_landmarks", &EKFBaseLandmarksROS::srvResetLandmarks, this);
  srv_set_depth_sensor_offset_ = nh_.advertiseService("set_depth_sensor_offset", &EKFBaseLandmarksROS::srvSetDepthSensorOffset, this);
  // clang-format on

  // Init timer
  timer_ = nh_.createTimer(ros::Duration(1.0), &EKFBaseLandmarksROS::checkDiagnostics, this);

  // Check NED and GPS configuration
  if (config_.initialize_ned_from_gps_ && !config_.initialize_filter_from_gps_ && config_.use_gps_data_)
  {
    ROS_ERROR("Weird configuration: Initialize NED from GPS and use GPS data but do not initialize filter from GPS");
  }
  else if (config_.initialize_ned_from_gps_ && !config_.initialize_filter_from_gps_ && !config_.use_gps_data_)
  {
    ROS_ERROR("Weird configuration: Initialize NED from GPS but do not initialize filter neither use data from GPS");
  }
  else if (!config_.initialize_ned_from_gps_ && !config_.initialize_filter_from_gps_ && config_.use_gps_data_)
  {
    ROS_ERROR("Invalid configuration: Do not initialize NED and filter from GPS but use data from it");
    ROS_WARN("Use GPS data will be set to false");
    config_.use_gps_data_ = false;
  }
}

std::string EKFBaseLandmarksROS::getNamespace() const
{
  return ns_;
}

void EKFBaseLandmarksROS::resetFilter()
{
  // Reset flags
  // general
  init_depth_offset_ = false;
  init_ekf_ = false;
  init_ned_ = false;
  diag_help_.add("ekf_init", false);
  diag_help_.add("ned_init", false);
  // sensors
  init_gps_ = false;
  init_depth_ = false;
  init_dvl_ = false;
  init_imu_ = false;
  diag_help_.add("gps_init", false);
  diag_help_.add("depth_init", false);
  diag_help_.add("dvl_init", false);
  diag_help_.add("imu_init", false);

  // Delete landmarks
  resetLandmarks();
  // Reset state vector
  x_ = Eigen::VectorXd::Zero(state_vector_size_);
  // Reset covariance
  Eigen::VectorXd p_var = Eigen::VectorXd::Zero(static_cast<unsigned int>(config_.initial_state_covariance_.size()));
  for (size_t i = 0; i < config_.initial_state_covariance_.size(); ++i)
  {
    p_var(static_cast<unsigned int>(i)) = config_.initial_state_covariance_[i];
  }
  P_ = Eigen::MatrixXd::Identity(p_var.rows(), p_var.rows());
  P_.diagonal() = p_var;
  // Reset prediction noise
  Eigen::VectorXd q_var = Eigen::VectorXd::Zero(static_cast<unsigned int>(config_.prediction_model_covariance_.size()));
  for (size_t i = 0; i < config_.prediction_model_covariance_.size(); ++i)
  {
    q_var(static_cast<unsigned int>(i)) = config_.prediction_model_covariance_[i];
  }
  Q_ = Eigen::MatrixXd::Identity(q_var.rows(), q_var.rows());
  Q_.diagonal() = q_var;

  // Reset NED without GPS
  if (!config_.initialize_ned_from_gps_)
  {
    ROS_INFO("Init NED from config file");
    ROS_INFO("NED: %.8f, %.8f", config_.ned_latitude_, config_.ned_longitude_);
    ned_ = cola2::utils::NED(config_.ned_latitude_, config_.ned_longitude_, 0.0);
    init_ned_ = true;
    diag_help_.add("ned_init", true);
  }
}

Eigen::Vector3d EKFBaseLandmarksROS::getPositionIncrementFrom(const double time) const
{
  for (unsigned int i = 0; i < last_usbl_positions_.size(); ++i)
  {
    if (last_usbl_positions_.at(i)[0] > time)
    {
      return last_usbl_positions_.at(last_usbl_positions_.size() - 1) - last_usbl_positions_.at(i);
    }
  }
  // If no position found, return a negative time
  return Eigen::Vector3d(-1.0, 0.0, 0.0);
}

void EKFBaseLandmarksROS::getConfig(const bool show)
{
  double declination_deg;  // auxiliar

  // Load params from ROS param server
  // Flags
  cola2::rosutils::getParam("navigator/initialize_filter_from_gps", config_.initialize_filter_from_gps_, false);
  cola2::rosutils::getParam("navigator/initialize_ned_from_gps", config_.initialize_ned_from_gps_, false);
  cola2::rosutils::getParam("navigator/gps_samples_to_init", config_.gps_samples_to_init_, 10);
  cola2::rosutils::getParam("navigator/use_gps_data", config_.use_gps_data_, false);
  cola2::rosutils::getParam("navigator/use_usbl_data", config_.use_usbl_data_, false);
  cola2::rosutils::getParam("navigator/use_force_model", config_.use_force_model_, false);
  cola2::rosutils::getParam("navigator/use_depth_data", config_.use_depth_data_, true);
  cola2::rosutils::getParam("navigator/use_dvl_data", config_.use_dvl_data_, true);
  cola2::rosutils::getParam("navigator/enable_debug", config_.enable_debug_, false);
  // NED
  cola2::rosutils::getParam("navigator/ned_latitude", config_.ned_latitude_, 0.0);
  cola2::rosutils::getParam("navigator/ned_longitude", config_.ned_longitude_, 0.0);
  // Depth offset
  cola2::rosutils::getParam("navigator/initialize_depth_sensor_offset", config_.initialize_depth_sensor_offset_, false);
  cola2::rosutils::getParam("navigator/surface_to_depth_sensor_distance", config_.surface2depth_sensor_distance_, 0.0);
  cola2::rosutils::getParam("navigator/depth_sensor_offset", config_.depth_sensor_offset_, 0.0);
  // Sensors
  cola2::rosutils::getParam("navigator/declination_in_degrees", declination_deg, 0.0);
  cola2::rosutils::getParam("navigator/dvl_max_v", config_.dvl_max_v_, 1.5);
  config_.declination_ = cola2::utils::degreesToRadians(declination_deg);
  cola2::rosutils::getParam("navigator/water_density", config_.water_density_, 1030.0);
  // Covariances
  cola2::rosutils::getParamVector("navigator/initial_state_covariance", config_.initial_state_covariance_);
  cola2::rosutils::getParamVector("navigator/prediction_model_covariance", config_.prediction_model_covariance_);
  cola2::rosutils::getParamVector("navigator/force_model_covariance", config_.force_model_covariance_);
  cola2::rosutils::getParamVector("navigator/force_model_scale", config_.force_model_scale_);
  // Diagnostics
  cola2::rosutils::getParam("navigator/min_diagnostics_frequency", config_.min_diagnostics_frequency_, 25.0);

  // Show
  if (show)
  {
    ROS_INFO("Loaded config from param server");
    ROS_INFO("===============================");
    ROS_INFO("init filter from gps: %d", config_.initialize_filter_from_gps_);
    ROS_INFO("   init ned from gps: %d", config_.initialize_ned_from_gps_);
    ROS_INFO(" gps samples to init: %d", config_.gps_samples_to_init_);
    ROS_INFO("        use gps data: %d", config_.use_gps_data_);
    ROS_INFO("       use usbl data: %d", config_.use_usbl_data_);
    ROS_INFO("     use force model: %d", config_.use_force_model_);
    ROS_INFO("      use depth data: %d", config_.use_depth_data_);
    ROS_INFO("        use dvl data: %d", config_.use_dvl_data_);
    ROS_INFO("        enable debug: %d\n", config_.enable_debug_);
    ROS_INFO("       ned latitude: %3.6f", config_.ned_latitude_);
    ROS_INFO("      ned longitude: %3.6f\n", config_.ned_longitude_);
    ROS_INFO("init depth sensor offset: %d", config_.initialize_depth_sensor_offset_);
    ROS_INFO("    surface2depth sensor: %.3f", config_.surface2depth_sensor_distance_);
    ROS_INFO("     depth sensor offset: %.3f\n", config_.depth_sensor_offset_);
    ROS_INFO(" declination deg: %.3f", declination_deg);
    ROS_INFO("dvl max velocity: %.3f", config_.dvl_max_v_);
    ROS_INFO("   water density: %.3f\n", config_.water_density_);
    ROS_INFO("min diagnostics frequancy: %.3f\n", config_.min_diagnostics_frequency_);
    // vectors
    std::stringstream ss;
    ss << "   initial state covariance: ";
    for (const double v : config_.initial_state_covariance_)
    {
      ss << v << ' ';
    }
    ROS_INFO_STREAM(ss.str());
    ss.str(std::string());  // empty it
    ss << "prediction model covariance: ";
    for (const double v : config_.prediction_model_covariance_)
    {
      ss << v << ' ';
    }
    ROS_INFO_STREAM(ss.str());
    ss.str(std::string());
    ss << "     force model covariance: ";
    for (const double v : config_.force_model_covariance_)
    {
      ss << v << ' ';
    }
    ROS_INFO_STREAM(ss.str());
    ss.str(std::string());
    ss << "          force model scale: ";
    for (const double v : config_.force_model_scale_)
    {
      ss << v << ' ';
    }
    ROS_INFO_STREAM(ss.str());
  }
}

void EKFBaseLandmarksROS::checkDiagnostics(const ros::TimerEvent& e)
{
  // *****************************************
  // Check sensors
  // *****************************************
  double now = e.current_real.toSec();
  bool is_nav_data_ok = true;
  // Check IMU data
  diag_help_.add("last_imu_data", std::to_string(now - last_imu_time_));
  if (now - last_imu_time_ > 1.0)
  {
    is_nav_data_ok = false;
    ROS_WARN("IMU too old");
  }
  // Check DVL data
  if (config_.use_dvl_data_)
  {
    diag_help_.add("last_dvl_data", std::to_string(now - last_dvl_time_));
    if (now - last_dvl_time_ > 2.0)
    {
      is_nav_data_ok = false;
      ROS_WARN("DVL too old");
    }
  }
  // Check altitude data (if it was ever received)
  diag_help_.add("last_altitude_data", std::to_string(now - last_altitude_time_));
  if ((last_altitude_time_ != 0.0) && (now - last_altitude_time_ > 5.0))
  {
    is_nav_data_ok = false;
    ROS_WARN("Altitude too old");
  }
  // Check depth data
  if (config_.use_depth_data_)
  {
    diag_help_.add("last_depth_data", std::to_string(now - last_depth_time_));
    if (now - last_depth_time_ > 2.0)
    {
      is_nav_data_ok = false;
      ROS_WARN("Depth too old");
    }
  }
  // Check gps data
  if (config_.use_gps_data_)
  {
    diag_help_.add("last_gps_data", std::to_string(now - last_gps_time_));
    if (now - last_gps_time_ > 3.0)
    {
      if (getPosition()(2) < 1.0)
      {
        is_nav_data_ok = false;
        ROS_WARN("GPS too old");
      }
    }
  }
  // *****************************************
  // Check other
  // *****************************************
  // Check current freq
  diag_help_.add("freq", std::to_string(diag_help_.getCurrentFreq()));
  if (diag_help_.getCurrentFreq() < config_.min_diagnostics_frequency_)
  {
    is_nav_data_ok = false;
    ROS_FATAL("Diagnostics frequency too low");
  }
  // If filter or NED not initialized set to Warning
  if (!init_ekf_)
  {
    is_nav_data_ok = false;
    ROS_FATAL("EKF not yet init");
  }
  else
  {
    diag_help_.add("ekf_init", true);
  }
  if (!init_ned_)
  {
    is_nav_data_ok = false;
    ROS_FATAL("NED not yet init");
  }
  else
  {
    diag_help_.add("ned_init", true);
  }

  // If all nav data is ok set navigator to Ok
  if (is_nav_data_ok && !ned_error_)
  {
    diag_help_.setLevel(diagnostic_msgs::DiagnosticStatus::OK);
  }
  else
  {
    diag_help_.setLevel(diagnostic_msgs::DiagnosticStatus::WARN);
  }

  // *****************************************
  // Sensor inits to diagnostics
  // *****************************************
  diag_help_.add("gps_init", !(config_.use_gps_data_ && !init_gps_));
  diag_help_.add("depth_init", init_depth_);
  diag_help_.add("dvl_init", init_dvl_);
  diag_help_.add("imu_init", init_imu_);

  // *****************************************
  // Output to console
  // *****************************************
  if (config_.use_dvl_data_ && !init_dvl_)
  {
    ROS_FATAL("DVL not initialized");
  }
  if (config_.use_depth_data_ && !init_depth_)
  {
    ROS_FATAL("Depth not initialized");
  }
  if (config_.use_gps_data_ && !init_gps_)
  {
    ROS_FATAL("GPS not initialized");
  }
  if (!init_imu_)
  {
    ROS_FATAL("IMU not initialized");
  }
  if (!init_ekf_)
  {
    ROS_FATAL("EKF not initialized");
  }
  if (!init_ned_)
  {
    ROS_FATAL("NED not initialized");
  }
  if (!is_nav_data_ok)
  {
    ROS_FATAL("Missing NAV data");
  }
}

void EKFBaseLandmarksROS::updatePositionGPSMsg(const sensor_msgs::NavSatFix& msg)
{
  // Check usage
  if (!config_.use_gps_data_)
  {
    return;
  }
  // Valid measurement
  if ((msg.status.status >= msg.status.STATUS_FIX) && (msg.position_covariance[0] < 10.0))  // TODO: 10.0 --> 4.0
  {
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Increase number of received messages
    gps_samples_++;

    // Continue if enough samples
    if (gps_samples_ >= static_cast<size_t>(config_.gps_samples_to_init_))
    {
      // Init depth offset
      if (!init_depth_offset_ && config_.initialize_depth_sensor_offset_)
      {
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        srvSetDepthSensorOffset(req, res);
        init_depth_offset_ = true;
      }
      // Init NED if necessary
      if (!init_ned_ && config_.initialize_ned_from_gps_)
      {
        ROS_INFO("Init NED from GPS at %.6f %.6f", msg.latitude, msg.longitude);
        ned_ = cola2::utils::NED(msg.latitude, msg.longitude, 0.0);
        init_ned_ = true;
      }
      // Construct measurement
      const Eigen::Vector3d latlonh(msg.latitude, msg.longitude, 0.0);
      Eigen::Vector3d ned = ned_.geodetic2Ned(latlonh);
      publishGPSNED(msg.header.stamp, ned);
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      cov(0, 0) = msg.position_covariance[0];
      cov(1, 1) = msg.position_covariance[4];
      // Transform to vehicle frame
      Eigen::Affine3d trans;
      if (!tf_handler_.getTransform(msg.header.frame_id, trans))
      {
        return;  // not possible to transform
      }
      ned = transforms::position(ned, getOrientation(), trans.translation());
      cov = transforms::positionCovariance(cov, getOrientationUncertainty(), getOrientation(), trans.translation());
      // Predict and update
      const double tim = msg.header.stamp.toSec();
      if (makePrediction(tim) || !init_ekf_)
      {
        // Debug
        if (config_.enable_debug_)
        {
          ofh_ << "#gps " << tim << ' ' << ned(0) << ' ' << ned(1) << ' ' << cov(0, 0) << ' ' << cov(0, 1) << ' '
               << cov(1, 0) << ' ' << cov(1, 1) << '\n';
        }
        // Update and publish
        updatePositionXY(msg.header.stamp.toSec(), ned.head(2), cov.topLeftCorner(2, 2));
        publishNavigationAndLandmarks(msg.header.stamp);
      }
    }
  }
  else if (!init_ned_)
  {
    gps_samples_wrong_before_init_++;
    if (gps_samples_wrong_before_init_ > 60)
    {
      // If after 60 samples the NED has not been initialized change GPS configuration to
      config_.initialize_ned_from_gps_ = false;
      config_.initialize_filter_from_gps_ = false;
      config_.use_gps_data_ = false;
      resetFilter();
      ned_error_ = true;
      diag_help_.add("ned_init", "ERROR");
      diag_help_.setLevel(diagnostic_msgs::DiagnosticStatus::WARN);
      ROS_WARN("Impossible to initialize filter with GPS");
    }
  }
}

void EKFBaseLandmarksROS::updatePositionUSBLMsg(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // Check usage
  if (!config_.use_usbl_data_)
  {
    return;
  }
  // Valid measurement TODO: maybe able to initialize filter from USBL?
  if (init_ned_ && init_ekf_)
  {
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Get delayed position increment [dt dx dy]
    const Eigen::Vector3d position_increment = getPositionIncrementFrom(msg.header.stamp.toSec());
    // Valid time increment
    if (position_increment(0) >= 0.0)
    {
      // Current time
      const ros::Time current_time(msg.header.stamp.toSec() + position_increment(0));
      // Construct measurement
      const Eigen::Vector3d latlonh(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0);
      Eigen::Vector3d ned = ned_.geodetic2Ned(latlonh);
      ned.head(2) += position_increment.tail(2);  // increment the same we increased
      ned(2) = getPosition()(2);                  // show in current depth
      publishUSBLNED(current_time, ned);          // show
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (unsigned int i = 0; i < 3; ++i)
      {
        for (unsigned int j = 0; j < 3; ++j)
        {
          cov(i, j) = msg.pose.covariance[6 * i + j];  // from 6x6 matrix
        }
      }
      // Transform to vehicle frame
      Eigen::Affine3d trans;
      if (!tf_handler_.getTransform(msg.header.frame_id, trans))
      {
        return;  // not possible to transform
      }
      ned = transforms::position(ned, getOrientation(), trans.translation());
      cov = transforms::positionCovariance(cov, getOrientationUncertainty(), getOrientation(), trans.translation());
      // Predict and update
      const double tim = current_time.toSec();
      if (makePrediction(tim) || !init_ekf_)
      {
        // Debug
        if (config_.enable_debug_)
        {
          ofh_ << "#usbl " << tim << ' ' << ned(0) << ' ' << ned(1) << ' ' << cov(0, 0) << ' ' << cov(0, 1) << ' '
               << cov(1, 0) << ' ' << cov(1, 1) << '\n';
        }
        // Update and publish
        updatePositionXY(tim, ned.head(2), cov.topLeftCorner(2, 2));
        publishNavigationAndLandmarks(current_time);
      }
    }
  }
}

void EKFBaseLandmarksROS::updatePositionDepthMsg(const sensor_msgs::FluidPressure& msg)
{
  // Valid measurement
  const double meters = msg.fluid_pressure / (config_.water_density_ * 9.81);  // pascals to meters
  if (meters > -1.0)
  {
    // Save pressure message for setDepthSensorOffset
    pressure_meters_ = meters;
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Construct measurement
    Eigen::Vector3d xyz(0.0, 0.0, meters + config_.depth_sensor_offset_);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(2, 2) = msg.variance / (config_.water_density_ * 9.81);  // variance pascals to meters
    // Transform to vehicle frame
    Eigen::Affine3d trans;
    if (!tf_handler_.getTransform(msg.header.frame_id, trans))
    {
      return;  // not possible to transform
    }
    xyz = transforms::position(xyz, getOrientation(), trans.translation());
    cov = transforms::positionCovariance(cov, getOrientationUncertainty(), getOrientation(), trans.translation());
    // Predict and update
    const double tim = msg.header.stamp.toSec();
    if (makePrediction(tim) || !init_ekf_)
    {
      // Debug
      if (config_.enable_debug_)
      {
        ofh_ << "#depth " << tim << ' ' << xyz.tail(1) << ' ' << cov(2, 2) << '\n';
      }
      // Update and publish
      updatePositionZ(msg.header.stamp.toSec(), xyz.tail(1), cov.bottomRightCorner(1, 1));
      publishNavigationAndLandmarks(msg.header.stamp);
    }
  }
  else
  {
    ROS_WARN("Pressure in meters is smaller than -1.0");
  }
}

void EKFBaseLandmarksROS::updateVelocityDVLMsg(const cola2_msgs::DVL& msg)
{
  // Valid measurement
  if ((msg.velocity_covariance[0] > 0.0) && (std::abs(msg.velocity.x) < config_.dvl_max_v_) &&
      (std::abs(msg.velocity.y) < config_.dvl_max_v_) && (std::abs(msg.velocity.z) < config_.dvl_max_v_))
  {
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Construct measurement
    Eigen::Vector3d vel(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    Eigen::Matrix3d cov;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        cov(i, j) = msg.velocity_covariance[static_cast<size_t>(i * 3 + j)];
      }
    }
    // Transform to vehicle frame
    Eigen::Affine3d trans;
    if (!tf_handler_.getTransform(msg.header.frame_id, trans))
    {
      return;  // not possible to transform
    }
    Eigen::Quaterniond quat(trans.rotation());
    vel = transforms::linearVelocity(vel, getAngularVelocity(), quat, trans.translation());
    cov = transforms::positionCovariance(cov, getAngularVelocityUncertainty(), quat, trans.translation());
    // Predict and update
    const double tim = msg.header.stamp.toSec();
    if (makePrediction(tim) || !init_ekf_)
    {
      // Debug
      if (config_.enable_debug_)
      {
        ofh_ << "#dvl " << tim << ' ' << vel(0) << ' ' << vel(1) << ' ' << vel(2) << ' ' << cov(0, 0) << ' '
             << cov(0, 1) << ' ' << cov(0, 2) << ' ' << cov(1, 0) << ' ' << cov(1, 1) << ' ' << cov(1, 2) << ' '
             << cov(2, 0) << ' ' << cov(2, 1) << ' ' << cov(2, 2) << '\n';
      }
      // Update and publish
      updateVelocity(msg.header.stamp.toSec(), vel, cov);
      publishNavigationAndLandmarks(msg.header.stamp);
    }
  }
}

void EKFBaseLandmarksROS::updateIMUMsg(const sensor_msgs::Imu& msg)
{
  // Diagnostics
  diag_help_.increaseFrequencyCounter();
  // Construct measurement
  Eigen::Quaterniond ori(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  Eigen::Vector3d rpy = cola2::utils::quaternion2euler(ori);
  rpy(2) = cola2::utils::wrapAngle(rpy(2) - config_.declination_);  // add declination
  ori = cola2::utils::euler2quaternion(rpy);
  Eigen::Vector3d ang_vel(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  Eigen::Matrix3d rpy_cov;
  Eigen::Matrix3d ang_vel_cov;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      rpy_cov(i, j) = msg.orientation_covariance[static_cast<size_t>(i * 3 + j)];
      ang_vel_cov(i, j) = msg.angular_velocity_covariance[static_cast<size_t>(i * 3 + j)];
    }
  }
  // Transform to vehicle frame
  Eigen::Affine3d trans;
  if (!tf_handler_.getTransform(msg.header.frame_id, trans))
  {
    return;  // not possible to transform
  }
  Eigen::Quaterniond quat(trans.rotation());
  ori = transforms::orientation(ori, quat);  // transform orientation
  rpy = cola2::utils::quaternion2euler(ori);
  ang_vel = transforms::angularVelocity(ang_vel, quat);  // transform angular velocity
  // Predict and update
  const double tim = msg.header.stamp.toSec();
  if (makePrediction(tim) || !init_ekf_)
  {
    // Debug
    if (config_.enable_debug_)
    {
      ofh_ << "#imu " << tim << ' ' << rpy(0) << ' ' << rpy(1) << ' ' << rpy(2) << ' ' << rpy_cov(0, 0) << ' '
           << rpy_cov(0, 1) << ' ' << rpy_cov(0, 2) << ' ' << rpy_cov(1, 0) << ' ' << rpy_cov(1, 1) << ' '
           << rpy_cov(1, 2) << ' ' << rpy_cov(2, 0) << ' ' << rpy_cov(2, 1) << ' ' << rpy_cov(2, 2) << '\n';
      ofh_ << "#rate " << tim << ' ' << ang_vel(0) << ' ' << ang_vel(1) << ' ' << ang_vel(2) << ' ' << ang_vel_cov(0, 0)
           << ' ' << ang_vel_cov(0, 1) << ' ' << ang_vel_cov(0, 2) << ' ' << ang_vel_cov(1, 0) << ' '
           << ang_vel_cov(1, 1) << ' ' << ang_vel_cov(1, 2) << ' ' << ang_vel_cov(2, 0) << ' ' << ang_vel_cov(2, 1)
           << ' ' << ang_vel_cov(2, 2) << '\n';
    }
    // Update and publish
    updateOrientation(msg.header.stamp.toSec(), rpy, rpy_cov);
    updateOrientationRate(msg.header.stamp.toSec(), ang_vel, ang_vel_cov);
    publishNavigationAndLandmarks(msg.header.stamp);
  }
}

void EKFBaseLandmarksROS::updateLandmarkMsg(const cola2_msgs::Detection& msg)
{
  // Valid measurement
  if (init_ekf_ && msg.detected)
  {
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Construct measurement
    Eigen::Vector3d lpos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Quaterniond lori(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z);
    Eigen::Matrix6d cov;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        cov(i, j) = msg.pose.covariance[static_cast<size_t>(i * 6 + j)];
      }
    }
    // Transform to vehicle frame
    Eigen::Affine3d trans;
    if (!tf_handler_.getTransform(msg.header.frame_id, trans))
    {
      return;  // not possible to transform
    }
    Eigen::Quaterniond quat(trans.rotation());
    lpos = transforms::landmarkPosition(lpos, quat, trans.translation());
    lori = transforms::landmarkOrientation(lori, quat);
    // TODO: Transform covariance to vehicle frame not properly done!!!
    cov.topLeftCorner(3, 3) = transforms::rotatedCovariance(cov.topLeftCorner(3, 3), quat);
    // Predict and update
    if (makePrediction(msg.header.stamp.toSec()))
    {
      // Compose vehicle position with the relative measure
      Eigen::Vector3d wrpy = cola2::utils::rotation2euler(getRotation() * lori.toRotationMatrix());
      // If the landmark has not been mapped
      int position = getLandmarkPosition(msg.id);
      if (position == -1)
      {
        // Create a candidate
        // Transform landmark position from vehicle to world
        Eigen::Vector3d wpos = transforms::landmarkPosition(lpos, getOrientation(), getPosition());
        Eigen::Vector6d candidate;
        candidate.head(3) = wpos;
        candidate.tail(3) = wrpy;
        ROS_INFO_STREAM("candidate (xyz rpy): " << msg.id << ' ' << candidate.transpose() << '\n');

        // Add to candidates (true if first addition)
        if (addLandmarkCandidate(msg.id, candidate))
        {
          ROS_INFO("New candidate: %s", msg.id.c_str());
        }

        // Check if enough to add to vector state
        if (isValidCandidate(msg.id))
        {
          ROS_INFO("Validated candidate: %s", msg.id.c_str());
          addLandmark(candidate, cov, msg.id);
        }
        return;  // nothing else to do
      }
      else
      {
        // It was already mapped TODO: check world vs relative measures frames
        updateLandmarkMeasure(msg.header.stamp.toSec(), lpos, wrpy, msg.id, cov);
        setLandmarkLastUpdate(msg.id, msg.header.stamp.toSec());
        publishNavigationAndLandmarks(msg.header.stamp);
      }
    }  // end prediction
  }    // end valid measure
}

void EKFBaseLandmarksROS::updateRangeMsg(const cola2_msgs::RangeDetection& msg)
{
  // Valid measurement (has to be already mapped)
  const int position = getLandmarkPosition(msg.id);
  if (init_ekf_ && position >= 0)
  {
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Construct measurement
    const Eigen::Matrix1d cov(msg.range);
    // Predict and update
    if (makePrediction(msg.header.stamp.toSec()))
    {
      // inno = z - h(x)
      const Eigen::Vector3d pos = getPosition();
      const Eigen::Vector3d lpos = getLandmarkPositionVector(static_cast<unsigned int>(position));
      const Eigen::Vector3d diff = lpos - pos;
      const double h = diff.norm();
      const Eigen::Vector1d inno(msg.range - h);
      // Linearization of h() into H
      const double inv = 1.0 / h;
      const unsigned int index = state_vector_size_ + LANDMARK_SIZE * static_cast<unsigned int>(position);
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, state_vector_size_ + LANDMARK_SIZE * getNumberOfLandmarks());
      H(0, 0) = -diff(0) * inv;  // TODO: position should always be on the first 3 positions
      H(0, 1) = -diff(1) * inv;
      H(0, 2) = -diff(2) * inv;
      H(0, index) = -H(0, 0);
      H(0, index + 1) = -H(0, 1);
      H(0, index + 2) = -H(0, 2);
      // Update and publish
      applyUpdate(inno, cov, H, Eigen::MatrixXd::Identity(1, 1), 25.0);
      setLandmarkLastUpdate(msg.id, msg.header.stamp.toSec());
      publishRangeMarker(msg.id, msg.range, msg.sigma);
    }
  }
}

void EKFBaseLandmarksROS::updateBodyForceReqMsg(const cola2_msgs::BodyForceReq& msg)
{
  // Valid measurement (we are not receiveng velocity messages from anywhere)
  if (init_ekf_ && (msg.header.stamp.toSec() - last_dvl_time_ > 1.0))
  {
    // Diagnostics
    diag_help_.increaseFrequencyCounter();
    // Construct measurement
    const Eigen::Vector3d force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (not msg.disable_axis.x)
    {
      velocity(0) =
          (force(0) > 0.0) ? force(0) / config_.force_model_scale_[0] : force(0) / config_.force_model_scale_[1];
    }
    if (not msg.disable_axis.y)
    {
      velocity(1) =
          (force(1) > 0.0) ? force(1) / config_.force_model_scale_[2] : force(1) / config_.force_model_scale_[3];
    }
    if (not msg.disable_axis.x)
    {
      velocity(2) =
          (force(2) > 0.0) ? force(2) / config_.force_model_scale_[4] : force(2) / config_.force_model_scale_[5];
    }
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = config_.force_model_covariance_[0];
    cov(1, 1) = config_.force_model_covariance_[1];
    cov(2, 2) = config_.force_model_covariance_[2];
    // Transform to vehicle frame => Velocities already in vehicle frame
    // Predict and update
    const double tim = msg.header.stamp.toSec();
    if (makePrediction(tim))
    {
      // Debug
      if (config_.enable_debug_)
      {
        ofh_ << "#force " << tim << ' ' << velocity(0) << ' ' << velocity(1) << ' ' << velocity(2) << ' ' << cov(0, 0)
             << ' ' << cov(0, 1) << ' ' << cov(0, 2) << ' ' << cov(1, 0) << ' ' << cov(1, 1) << ' ' << cov(1, 2) << ' '
             << cov(2, 0) << ' ' << cov(2, 1) << ' ' << cov(2, 2) << '\n';
      }
      // Update and publish
      updateVelocity(msg.header.stamp.toSec(), velocity, cov, false);  // not coming from dvl
      publishNavigationAndLandmarks(msg.header.stamp);
    }
  }
}

void EKFBaseLandmarksROS::updateSoundVelocityMsg(const cola2_msgs::Float32Stamped& msg)
{
  sound_velocity_ = static_cast<double>(msg.data);
}

void EKFBaseLandmarksROS::updateAltitudeMsg(const sensor_msgs::Range& msg)
{
  // Check valid
  if (msg.range > 0.0f)
  {
    altitude_ = static_cast<double>(msg.range);
    last_altitude_time_ = msg.header.stamp.toSec();
    publishNavigationAndLandmarks(msg.header.stamp);
  }
}

void EKFBaseLandmarksROS::publishNavigationAndLandmarks(const ros::Time& stamp)
{
  // State
  const Eigen::Vector3d pos = getPosition();
  const Eigen::Vector3d vel = getVelocity();
  const Eigen::Vector3d rpy = getEuler();
  const Eigen::Quaterniond quat = getOrientation();
  const Eigen::Vector3d ang_vel = getAngularVelocity();
  // Covariance
  const Eigen::Matrix3d pos_cov = getPositionUncertainty();
  const Eigen::Matrix3d vel_cov = getVelocityUncertainty();
  const Eigen::Matrix3d rpy_cov = getOrientationUncertainty();
  const Eigen::Matrix3d ang_vel_cov = getAngularVelocityUncertainty();

  // Debug
  if (config_.enable_debug_)
  {
    ofh_ << stamp.toSec() << ' ';
    // State
    for (size_t i = 0; i < state_vector_size_; ++i)
    {
      ofh_ << x_(static_cast<long>(i)) << ' ';
    }
    // Covariance
    for (size_t i = 0; i < state_vector_size_; ++i)
    {
      for (size_t j = 0; j < state_vector_size_; ++j)
      {
        ofh_ << P_(static_cast<long>(i), static_cast<long>(j)) << ' ';
      }
    }
    ofh_ << '\n';
  }

  // Save last 10s of positions for the USBL delayed data TODO: make more efficient
  last_usbl_positions_.push_back(Eigen::Vector3d(stamp.toSec(), pos(0), pos(1)));
  bool found = false;
  while (!found)
  {
    if (last_usbl_positions_.size() > 0 && (stamp.toSec() - last_usbl_positions_.at(0)[0]) > USBL_KEEP_TIME)
    {
      last_usbl_positions_.erase(last_usbl_positions_.begin());  // delete first if it is too old
    }
    else
    {
      found = true;
    }
  }

  // Check that enough time has passed
  if (stamp.toSec() - last_publication_time_ < TIME_BETWEEN_PUBLISHING)
  {
    return;
  }
  last_publication_time_ = stamp.toSec();  // now is the last time

  // Publish Odometry message
  nav_msgs::Odometry odom;
  odom.header.frame_id = frame_world_;
  odom.header.stamp = stamp;
  odom.pose.pose.position.x = pos(0);
  odom.pose.pose.position.y = pos(1);
  odom.pose.pose.position.z = pos(2);
  odom.pose.pose.orientation.w = quat.w();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.twist.twist.linear.x = vel(0);
  odom.twist.twist.linear.y = vel(1);
  odom.twist.twist.linear.z = vel(2);
  odom.twist.twist.angular.x = ang_vel(0);
  odom.twist.twist.angular.y = ang_vel(1);
  odom.twist.twist.angular.z = ang_vel(2);
  for (unsigned int i = 0; i < 3; ++i)  // TODO: copy whole matrix
  {
    for (unsigned int j = 0; j < 3; ++j)
    {
      // pose and orientation
      odom.pose.covariance.at(i * 6 + j) = pos_cov(i, j);
      odom.pose.covariance.at((i + 3) * 6 + (j + 3)) = rpy_cov(i, j);
      // velocity and rate
      odom.twist.covariance.at(i * 6 + j) = vel_cov(i, j);
      odom.twist.covariance.at((i + 3) * 6 + (j + 3)) = ang_vel_cov(i, j);
    }
  }
  pub_odom_.publish(odom);

  // Publish Nav Status
  const Eigen::Vector3d latlonh = ned_.ned2geodetic(pos);
  cola2_msgs::NavSts nav_sts;
  nav_sts.header.frame_id = frame_vehicle_;
  nav_sts.header.stamp = stamp;
  nav_sts.altitude = static_cast<float>(altitude_);
  nav_sts.body_velocity.x = vel(0);
  nav_sts.body_velocity.y = vel(1);
  nav_sts.body_velocity.z = vel(2);
  nav_sts.global_position.latitude = latlonh(0);
  nav_sts.global_position.longitude = latlonh(1);
  nav_sts.orientation.roll = static_cast<float>(rpy(0));
  nav_sts.orientation.pitch = static_cast<float>(rpy(1));
  nav_sts.orientation.yaw = static_cast<float>(rpy(2));
  nav_sts.orientation_rate.roll = static_cast<float>(ang_vel(0));
  nav_sts.orientation_rate.pitch = static_cast<float>(ang_vel(1));
  nav_sts.orientation_rate.yaw = static_cast<float>(ang_vel(2));
  nav_sts.origin.latitude = ned_.getInitLatitude();
  nav_sts.origin.longitude = ned_.getInitLongitude();
  nav_sts.position.north = pos(0);
  nav_sts.position.east = pos(1);
  nav_sts.position.depth = pos(2);
  nav_sts.position_variance.north = pos_cov(0, 0);
  nav_sts.position_variance.east = pos_cov(1, 1);
  nav_sts.position_variance.depth = pos_cov(2, 2);
  nav_sts.orientation_variance.roll = static_cast<float>(rpy_cov(0, 0));
  nav_sts.orientation_variance.pitch = static_cast<float>(rpy_cov(1, 1));
  nav_sts.orientation_variance.yaw = static_cast<float>(rpy_cov(2, 2));
  pub_nav_.publish(nav_sts);

  // Publish vehicle TF
  tf::Transform tf_vehicle;
  tf_vehicle.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  tf_vehicle.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
  tf_broadcast_.sendTransform(tf::StampedTransform(tf_vehicle, stamp, frame_world_, frame_vehicle_));

  // Publish altitude range and TF
  sensor_msgs::Range range;
  range.header.frame_id = ns_ + "/altitude";
  range.header.stamp = stamp;
  range.max_range = 60.0;
  range.min_range = 0.3f;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = 0.05f;
  range.range = (altitude_ > 0.0) ? static_cast<float>(altitude_) : 0.0;
  pub_altitude_.publish(range);

  // Landmarks
  if (getNumberOfLandmarks() > 0)
  {
    // Publish map of landmarks and their marker representation
    visualization_msgs::MarkerArray landmarks_array;
    cola2_msgs::Map map;
    map.header.frame_id = frame_world_;
    map.header.stamp = stamp;
    for (unsigned int l = 0; l < getNumberOfLandmarks(); ++l)
    {
      Eigen::Vector3d xyz = getLandmarkPositionVector(l);
      Eigen::Quaterniond lquat = getLandmarkOrientationQuaternion(l);
      // Custom message
      cola2_msgs::Landmark landmark;
      landmark.landmark_id = getLandmarkId(l);
      landmark.last_update.fromSec(getLandmarkLastUpdate(getLandmarkId(l)));
      landmark.pose.pose.position.x = xyz(0);
      landmark.pose.pose.position.y = xyz(1);
      landmark.pose.pose.position.z = xyz(2);
      landmark.pose.pose.orientation.w = lquat.w();
      landmark.pose.pose.orientation.x = lquat.x();
      landmark.pose.pose.orientation.y = lquat.y();
      landmark.pose.pose.orientation.z = lquat.z();
      Eigen::MatrixXd landmark_cov = getLandmarkUncertainty(l);
      for (unsigned int i = 0; i < LANDMARK_SIZE; ++i)
      {
        for (unsigned int j = 0; j < LANDMARK_SIZE; ++j)
        {
          landmark.pose.covariance.at(i * LANDMARK_SIZE + j) = landmark_cov(i, j);
        }
      }
      map.landmark.push_back(landmark);
      // Marker representation
      visualization_msgs::Marker marker;
      marker.header = map.header;
      marker.ns = getLandmarkId(l);
      marker.id = 0;
      marker.type = marker.CUBE;
      marker.action = marker.ADD;
      marker.pose.position = landmark.pose.pose.position;
      marker.pose.orientation = landmark.pose.pose.orientation;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.1;
      marker.color.r = 0.1f;
      marker.color.g = 0.1f;
      marker.color.b = 1.0f;
      marker.color.a = 0.6f;
      marker.lifetime = ros::Duration(2.0);
      marker.frame_locked = false;
      landmarks_array.markers.push_back(marker);
    }
    pub_map_.publish(map);
    pub_landmarks_.publish(landmarks_array);
  }
}

void EKFBaseLandmarksROS::publishGPSNED(const ros::Time& stamp, const Eigen::Vector3d& ned) const
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_world_;
  msg.pose.position.x = ned(0);
  msg.pose.position.y = ned(1);
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;  // pointing upwards
  msg.pose.orientation.y = 0.70710678;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 0.70710678;
  pub_gps_ned_.publish(msg);
}

void EKFBaseLandmarksROS::publishUSBLNED(const ros::Time& stamp, const Eigen::Vector3d& ned) const
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_world_;
  msg.pose.position.x = ned(0);
  msg.pose.position.y = ned(1);
  msg.pose.position.z = ned(2);
  msg.pose.orientation.x = 0.0;  // pointing upwards
  msg.pose.orientation.y = 0.70710678;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 0.70710678;
  pub_usbl_ned_.publish(msg);
}

void EKFBaseLandmarksROS::publishRangeMarker(const std::string& landmark_id, const double range,
                                             const double sigma) const
{
  int l = getLandmarkPosition(landmark_id);
  if (l >= 0)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_world_;
    marker.header.stamp = ros::Time::now();
    marker.ns = std::string("range_") + landmark_id;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    const Eigen::Vector3d pos = getPosition();
    geometry_msgs::Point init;
    init.x = pos(0);
    init.y = pos(1);
    init.z = pos(2);
    marker.points.push_back(init);
    const Eigen::Vector3d xyz = getLandmarkPositionVector(static_cast<unsigned int>(l));
    geometry_msgs::Point end;
    end.x = xyz(0);
    end.y = xyz(1);
    end.z = xyz(2);
    marker.points.push_back(end);
    // Color by range
    const double current_range = (xyz - pos).norm();
    if (fabs(current_range - range) < sigma / 2.0)
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else if (fabs(current_range - range) < sigma)
    {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else if (fabs(current_range - range) < 2.0 * sigma)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
    }
    else
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    marker.scale.x = 0.25;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(1.5);
    marker.frame_locked = false;
    pub_range_update_.publish(marker);
  }
}

// *****************************************
// Services
// *****************************************
bool EKFBaseLandmarksROS::srvResetLandmarks(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Reset landmarks service called");
  resetLandmarks();
  return true;
}

bool EKFBaseLandmarksROS::srvResetNavigation(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Reset navigation service called");
  getConfig();
  resetFilter();
  return true;
}

bool EKFBaseLandmarksROS::srvSetDepthSensorOffset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Set depth sensor offset service called");
  config_.depth_sensor_offset_ = config_.surface2depth_sensor_distance_ - pressure_meters_;
  ROS_INFO("New depth sensor offset at %.3f", config_.depth_sensor_offset_);
  return true;
}
