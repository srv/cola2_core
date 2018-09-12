/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>This node merges data from different navigation sensors to estimate the robot position and velocity.<@@*/

#include <cola2_nav/ekf_position_velocity_landmarks.h>
#include <ros/ros.h>

class NavigatorNode : public EKFPositionVelocityLandmarks
{
private:
  // Subscribers
  ros::Subscriber sub_gps_;       // [x y]
  ros::Subscriber sub_usbl_;      // [x y]
  ros::Subscriber sub_pressure_;  // [z]
  ros::Subscriber sub_dvl_;       // [vx vy vz]
  ros::Subscriber sub_model_;     // [vx vy vz]
  ros::Subscriber sub_imu_;       // [roll pitch yaw vroll vpitch vyaw]
  // ros::Subscriber sub_landmark_;        // landmark_id [lx ly lz lroll lpitch lyaw]
  // ros::Subscriber sub_range_;           // landmark_id [range]
  ros::Subscriber sub_sound_velocity_;  // sound velocity from SVS
  ros::Subscriber sub_altitude_;        // altitude from seafloor

public:
  /**
   * \brief Constructor that relates all sensors to their callbacks.
   */
  NavigatorNode();
  /**
   * \brief Destructor.
   */
  ~NavigatorNode();
};

NavigatorNode::NavigatorNode()
{
  // clang-format off
  // Init subscribers
  sub_gps_ = nh_.subscribe("gps", 2, &EKFBaseLandmarksROS::updatePositionGPSMsg, reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_usbl_ = nh_.subscribe("usbl", 2, &EKFBaseLandmarksROS::updatePositionUSBLMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_pressure_ = nh_.subscribe("pressure", 2, &EKFBaseLandmarksROS::updatePositionDepthMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_dvl_ = nh_.subscribe("dvl", 2, &EKFBaseLandmarksROS::updateVelocityDVLMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_imu_ = nh_.subscribe("imu", 2, &EKFBaseLandmarksROS::updateIMUMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  //sub_landmark_ = nh_.subscribe("detection_update", 2, &EKFBaseLandmarksROS::updateLandmarkMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  //sub_range_ = nh_.subscribe("range_update", 2, &EKFBaseLandmarksROS::updateRangeMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  if (config_.use_force_model_)
  {
    sub_model_ = nh_.subscribe("merged/body_force_req", 2, &EKFBaseLandmarksROS::updateBodyForceReqMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  }
  // Other data
  sub_sound_velocity_ = nh_.subscribe("sound_velocity", 2, &EKFBaseLandmarksROS::updateSoundVelocityMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_altitude_ = nh_.subscribe("altitude", 2, &EKFBaseLandmarksROS::updateAltitudeMsg,  reinterpret_cast<EKFBaseLandmarksROS*>(this));
  // clang-format on
}

NavigatorNode::~NavigatorNode()
{
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "navigator");
  ROS_INFO("Init NavigatorNode");
  NavigatorNode node;
  ros::spin();  // spin until architecture stops
  return 0;
}
