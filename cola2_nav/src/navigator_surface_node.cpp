/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_nav/ekf_surface_2d.h>
#include <ros/ros.h>

class NavigatorSurfaceNode : public EKFSurface2D
{
private:
  // Subscribers
  ros::Subscriber sub_gps_;             // [x y]
  ros::Subscriber sub_dvl_;             // [vx vy vz]
  ros::Subscriber sub_imu_;             // [roll pitch yaw vroll vpitch vyaw]
  ros::Subscriber sub_sound_velocity_;  // sound velocity from SVS
  ros::Subscriber sub_altitude_;        // altitude from seafloor

public:
  /**
   * \brief Constructor that relates all sensors to their callbacks.
   */
  NavigatorSurfaceNode();
  /**
   * \brief Destructor.
   */
  ~NavigatorSurfaceNode();
};

NavigatorSurfaceNode::NavigatorSurfaceNode()
{
  // clang-format off
  // Init subscribers
  sub_gps_ = nh_.subscribe("gps", 2, &EKFBaseLandmarksROS::updatePositionGPSMsg, reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_dvl_ = nh_.subscribe("dvl", 2, &EKFBaseLandmarksROS::updateVelocityDVLMsg, reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_imu_ = nh_.subscribe("imu", 2, &EKFBaseLandmarksROS::updateIMUMsg, reinterpret_cast<EKFBaseLandmarksROS*>(this));
  // Other data
  sub_sound_velocity_ = nh_.subscribe("sound_velocity", 2, &EKFBaseLandmarksROS::updateSoundVelocityMsg, reinterpret_cast<EKFBaseLandmarksROS*>(this));
  sub_altitude_ = nh_.subscribe("altitude", 2, &EKFBaseLandmarksROS::updateAltitudeMsg, reinterpret_cast<EKFBaseLandmarksROS*>(this));
  // clang-format on
}

NavigatorSurfaceNode::~NavigatorSurfaceNode()
{
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "navigator");
  ROS_INFO("Init NavigatorNode");
  NavigatorSurfaceNode node;
  ros::spin();  // spin until architecture stops
  return 0;
}
