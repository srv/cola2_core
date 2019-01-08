/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_BASE_LANDMARKS_ROS_H_
#define COLA2_NAV_EKF_BASE_LANDMARKS_ROS_H_

// messages subscribed
#include <cola2_msgs/BodyForceReq.h>                  // force velocity model
#include <cola2_msgs/DVL.h>                           // dvl
#include <cola2_msgs/Detection.h>                     // landmark detection
#include <cola2_msgs/Float32Stamped.h>                // force velocity model
#include <cola2_msgs/RangeDetection.h>                // range detection
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // usbl
#include <sensor_msgs/FluidPressure.h>                // depth
#include <sensor_msgs/Imu.h>                          // imu
#include <sensor_msgs/NavSatFix.h>                    // gps
#include <sensor_msgs/Range.h>                        // altitude
#include <std_msgs/Float32.h>                         // sound velocity
// messages published
#include <cola2_msgs/Map.h>                    // custom landmarks
#include <cola2_msgs/NavSts.h>                 // custom navigation
#include <diagnostic_msgs/DiagnosticStatus.h>  // diagnostics
#include <geometry_msgs/PoseStamped.h>         // gps_ned, usbl_ned
#include <nav_msgs/Odometry.h>                 // odometry
#include <visualization_msgs/Marker.h>         // landmark visualization
#include <visualization_msgs/MarkerArray.h>    // landmark visualization
// services
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
// all
#include <cola2_lib/rosutils/diagnostic_helper.h>
#include <cola2_lib/rosutils/param_loader.h>
#include <cola2_lib/rosutils/this_node.h>
#include <cola2_lib/rosutils/transform_handler.h>
#include <cola2_lib/utils/ned.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>
#include "./ekf_base_landmarks.h"
#include "./transformations.h"

namespace
{
constexpr double TIME_BETWEEN_PUBLISHING = 1.0 / 20.0;  //!< minimum time between publishNavigationAndLandmarks()
constexpr double USBL_KEEP_TIME = 10.0;                 //!< seconds to keep position history for delayed USBLs
constexpr size_t ALTITUDE_WINDOW_SIZE = 4;              //!< altitude window to check for valid measurements
}

/**
 * \brief EKF including all the ROS functionalities.
 * It loads parameters, publishes state
 */
class EKFBaseLandmarksROS : public EKFBaseLandmarks
{
private:
  // ROS transforms and frames
  cola2::rosutils::TransformHandler tf_handler_;  //!< provide tfs
  tf::TransformBroadcaster tf_broadcast_;         //!< to publish world->vehicle
  std::string ns_;                                //!< namespace we are in
  std::string frame_world_ = "world_ned";         //!< frame where AUV is located
  std::string frame_vehicle_;                     //!< frame of the vehicle (ns/base_link)

  // ROS Publishers
  ros::Publisher pub_odom_;          //!< odometry message
  ros::Publisher pub_nav_;           //!< custom navigation message
  ros::Publisher pub_gps_ned_;       //!< GPS projected in current NED
  ros::Publisher pub_usbl_ned_;      //!< USBL projected in current NED
  ros::Publisher pub_range_update_;  //!< show line from vehicle to landmark
  ros::Publisher pub_map_;           //!< custom message containing all features
  ros::Publisher pub_landmarks_;     //!< all features as markers
  ros::Publisher pub_altitude_;      //!< altitude from the seafloor
  double last_publication_time_ = 0.0;
  // ROS Services
  ros::ServiceServer srv_reload_params_;            //!< same as reset navigation
  ros::ServiceServer srv_reset_navigation_;         //!< realoads params and sets the filter to initial state
  ros::ServiceServer srv_reset_landmarks_;          //!< deteletes all landmarks from filter
  ros::ServiceServer srv_set_depth_sensor_offset_;  //!< compute the depth sensor offset
  ros::ServiceClient srv_publish_params_;

  // Status
  double pressure_meters_ = 0.0;    //!< last measured pressure
  double altitude_ = 0.5;           //!< altitude of the vehicle above floor
  double sound_velocity_ = 1500.0;  //!< sound velocity

  // Other
  ros::Timer timer_;                                  //!< keep checking diagnostics
  cola2::utils::NED ned_;                             //!< reference frame the AUV is in
  std::vector<Eigen::Vector3d> last_usbl_positions_;  //!< delayed USBL

  // Debug in output
  std::ofstream ofh_;

  /**
   * \brief Function called by the check diagnostics timer.
   */
  void checkDiagnostics(const ros::TimerEvent& e);
  /**
   * \brief Reset filter to initial status
   */
  void resetFilter();
  /**
   * \brief Get position increment for a position in the past.
   */
  Eigen::Vector3d getPositionIncrementFrom(const double time) const;

protected:
  ros::NodeHandle nh_ = ros::NodeHandle("~");  // ROS node handler
  bool online_;  // navigator running online

  // Init flags
  bool init_depth_offset_ = false;  //!< init depth offset
  bool init_gps_ = false;           //!< init sensor gps
  bool init_depth_ = false;         //!< init sensor depth
  bool init_dvl_ = false;           //!< init sensor dvl
  bool init_imu_ = false;           //!< init sensor imu
  bool init_ned_ = false;           //!< init NED
  bool ned_error_ = false;          //!< not enough good smaples to init NED from GPS

  // Diagnostics
  cola2::rosutils::DiagnosticHelper diag_help_;  //!< ease publishing diagnostics
  double last_gps_time_ = 0.0;                   //!< last time gps received
  double last_usbl_time_ = 0.0;                  //!< last time usbl received
  double last_depth_time_ = 0.0;                 //!< last time depth received
  double last_dvl_time_ = 0.0;                   //!< last time dvl received
  double last_imu_time_ = 0.0;                   //!< last time imu received
  double last_altitude_time_ = 0.0;              //!< last time altitude received
  double last_ekf_init_time_ = 0.0;              //!< last time ekf initialized
  size_t dvl_bottom_status_ = 0;
  size_t gps_samples_ = 0;
  size_t gps_samples_wrong_before_init_ = 0;

  // Configs from param server
  void getConfig(const bool show = false);
  struct Config
  {
    // Flags
    bool initialize_filter_from_gps_;
    bool initialize_ned_from_gps_;
    int gps_samples_to_init_;
    bool use_gps_data_;
    bool use_usbl_data_;
    bool use_force_model_;
    bool use_depth_data_;
    bool use_dvl_data_;
    bool enable_debug_;
    // NED
    double ned_latitude_;
    double ned_longitude_;
    // Depth offset
    bool initialize_depth_sensor_offset_;
    double surface2depth_sensor_distance_;
    double depth_sensor_offset_;
    // Sensors
    double declination_;
    double dvl_max_v_;
    double water_density_;
    // Covariances
    std::vector<double> initial_state_covariance_;
    std::vector<double> prediction_model_covariance_;
    std::vector<double> force_model_covariance_;
    std::vector<double> force_model_scale_;
    // Diagnostics
    double min_diagnostics_frequency_;
    // TODO: enable/disable dvl_bottom dvl_water force_model
  };
  Config config_;  //!< config loaded by getConfig()

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   *  \brief Constructor.
   */
  explicit EKFBaseLandmarksROS(const unsigned int state_vector_size, const bool online = true);
  /**
   *  \brief Destructor.
   */
  // virtual ~EKFBaseLandmarksROS() = default;
  /**
   *  \brief Get namespace.
   */
  std::string getNamespace() const;

  // *****************************************
  // Updates from ROS messages
  // *****************************************
  // Messages that affect the filter
  void updatePositionGPSMsg(const sensor_msgs::NavSatFix& msg);
  void updatePositionUSBLMsg(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void updatePositionDepthMsg(const sensor_msgs::FluidPressure& msg);
  void updateVelocityDVLMsg(const cola2_msgs::DVL& msg);
  void updateIMUMsg(const sensor_msgs::Imu& msg);
  // Landmarks
  void updateLandmarkMsg(const cola2_msgs::Detection& msg);
  void updateRangeMsg(const cola2_msgs::RangeDetection& msg);
  // Others
  void updateBodyForceReqMsg(const cola2_msgs::BodyForceReq& msg);
  void updateSoundVelocityMsg(const cola2_msgs::Float32Stamped& msg);
  void updateAltitudeMsg(const sensor_msgs::Range& msg);

  // *****************************************
  // Publishers
  // *****************************************
  void publishNavigationAndLandmarks(const ros::Time& stamp);
  void publishGPSNED(const ros::Time& stamp, const Eigen::Vector3d& ned) const;
  void publishUSBLNED(const ros::Time& stamp, const Eigen::Vector3d& ned) const;
  void publishRangeMarker(const ros::Time& stamp, const std::string& landmark_id, const double range, const double sigma) const;

  // *****************************************
  // Services
  // *****************************************
  bool srvResetLandmarks(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srvResetNavigation(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srvSetDepthSensorOffset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // *****************************************
  // Offline
  // *****************************************
  void loadTranformsFromFile(const std::string &fname);

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

#endif  // COLA2_NAV_EKF_BASE_LANDMARKS_ROS_H_
