/*
 * Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>This node shows an example of how to program an external mission so that it correctly communicates with the captain.<@@*/

#include <ros/ros.h>
#include <cola2_lib/rosutils/this_node.h>
#include <std_srvs/Trigger.h>
#include <string>

class ExternalMissionExample
{
 private:
  // ROS
  ros::NodeHandle nh_;

  // Services
  ros::ServiceServer enable_srv_, disable_srv_;
  ros::ServiceClient captain_enable_srv_, captain_disable_srv_;

  // Duration
  unsigned int duration_sec_;
  unsigned int current_sec_;

  // Timer
  ros::Timer timer_;

  // Methods
  bool enableSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    // Check if running
    if (current_sec_ <= duration_sec_)
    {
      std::string msg("Still running");
      ROS_ERROR_STREAM(msg);
      res.message = msg;
      res.success = false;
      return true;
    }

    // Call the captain and check response
    std_srvs::Trigger::Request captain_req;
    std_srvs::Trigger::Response captain_res;
    captain_enable_srv_.call(captain_req, captain_res);
    if (!captain_res.success)
    {
      std::string msg("Impossible to enable. Captain responded: " + captain_res.message);
      ROS_ERROR_STREAM(msg);
      res.message = msg;
      res.success = false;
      return true;
    }

    // Start
    current_sec_ = 0;
    res.message = "Enabled";
    res.success = true;
    ROS_INFO_STREAM(res.message);
    return true;
  }

  bool disableSrv(ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event)
  {
    // Display warning if not running
    if (current_sec_ > duration_sec_)
    {
      ROS_WARN_STREAM("Not enabled, but being disabled");
    }

    // Get service response
    std_srvs::Trigger::Response& res = event.getResponse();

    // Check if the caller is the captain
    std::string caller_name(event.getCallerName());
    if (caller_name.find("captain") == std::string::npos)  // Is there a better way?
    {
      std_srvs::Trigger::Request captain_req;
      std_srvs::Trigger::Response captain_res;
      captain_disable_srv_.call(captain_req, captain_res);
      if (!captain_res.success)
      {
        ROS_WARN_STREAM("Disabled but captain complained: " << captain_res.message);
      }
    }

    // Disable
    current_sec_ = duration_sec_ + 1;
    res.message = "Disabled";
    res.success = true;
    ROS_INFO_STREAM(res.message);
    return true;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    // Check if active
    if (current_sec_ > duration_sec_) return;

    // Display message
    ROS_INFO_STREAM("External mission example running. Time = " << current_sec_ << "/" << duration_sec_);

    // If about to finish, tell the captain
    if (current_sec_ == duration_sec_)
    {
      std_srvs::Trigger::Request captain_req;
      std_srvs::Trigger::Response captain_res;
      captain_disable_srv_.call(captain_req, captain_res);
      if (captain_res.success)
      {
        ROS_INFO_STREAM("Finished");
      }
      else
      {
        ROS_WARN_STREAM("Finished but captain complained: " << captain_res.message);
      }
    }

    // Increment time
    ++current_sec_;
  }

 public:
  ExternalMissionExample(unsigned int duration_sec):
      nh_("~"), duration_sec_(duration_sec), current_sec_(duration_sec + 1)
  {
    // Service clients
    captain_enable_srv_ = nh_.serviceClient<std_srvs::Trigger>(
                                        cola2::rosutils::getNamespace() +
                                        "/captain/enable_external_mission");
    captain_disable_srv_ = nh_.serviceClient<std_srvs::Trigger>(
                                        cola2::rosutils::getNamespace() +
                                        "/captain/disable_external_mission");

    // Wait for service clients
    while (ros::ok())
    {
        if (captain_enable_srv_.waitForExistence(ros::Duration(1.0))) break;
        ROS_INFO_STREAM("Waiting " << cola2::rosutils::getNamespace() <<
                        "/captain/enable_external_mission service");
    }
    while (ros::ok())
    {
        if (captain_disable_srv_.waitForExistence(ros::Duration(1.0))) break;
        ROS_INFO_STREAM("Waiting " << cola2::rosutils::getNamespace() <<
                        "/captain/disable_external_mission service");
    }

    // Service servers
    enable_srv_ = nh_.advertiseService("enable", &ExternalMissionExample::enableSrv, this);
    disable_srv_ = nh_.advertiseService("disable", &ExternalMissionExample::disableSrv, this);

    // Create main timer
    timer_ = nh_.createTimer(ros::Duration(1.0), &ExternalMissionExample::timerCallback, this);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "external_mission_example");
  ExternalMissionExample external_mission_example(20);
  ros::spin();
  return 0;
}
