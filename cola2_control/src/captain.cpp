
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>High level controller that provides control actions and services to load and execute missions, reach waypoints,
 keep position,...
 This node mainly translates user requests to pilot action libs.<@@*/

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_control/controllers/types.h>
#include <cola2_control/mission_utils/mission.h>
#include <cola2_control/mission_utils/mission_maneuver.h>

#include <cola2_lib/rosutils/diagnostic_helper.h>
#include <cola2_lib/rosutils/get_namespace.h>
#include <cola2_lib/rosutils/param_loader.h>
#include <cola2_lib/utils/ned.h>
#include <cola2_msgs/Action.h>
#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/GoalDescriptor.h>
#include <cola2_msgs/Goto.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/String.h>
#include <cola2_msgs/WorldSectionAction.h>
#include <cola2_msgs/WorldWaypointAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <string>
#include <vector>

typedef struct
{
  double max_distance_to_waypoint;
  std::string mission_path;
} CaptainConfig;

class Captain
{
private:
  // Node handle
  ros::NodeHandle nh_;

  // Class attributes
  bool is_waypoint_running_;
  bool is_section_running_;
  bool is_mission_running_;
  bool is_mission_paused_;
  ros::MultiThreadedSpinner spinner_;
  control::Nav nav_;
  CaptainConfig config_;
  bool is_keep_pose_enabled_;
  double min_goto_vel_;
  double min_loscte_vel_;
  cola2_msgs::CaptainStatus captain_status_;

  // Diagnostics
  cola2::rosutils::DiagnosticHelper diagnostic_;

  // Publishers
  ros::Publisher pub_path_;
  ros::Publisher pub_captain_status_;

  // Services
  ros::ServiceServer enable_goto_srv_;
  ros::ServiceServer disable_goto_srv_;
  ros::ServiceServer enable_default_mission_non_block_srv_;
  ros::ServiceServer disable_mission_srv_;
  ros::ServiceServer enable_keep_position_holonomic_srv_;
  ros::ServiceServer enable_keep_position_non_holonomic_srv_;
  ros::ServiceServer disable_keep_position_srv_;
  ros::ServiceServer pause_mission_srv_;
  ros::ServiceServer resume_mission_srv_;
  ros::ServiceServer enable_external_mission_srv_;
  ros::ServiceServer disable_external_mission_srv_;
  ros::ServiceServer enable_mission_srv_;

  // Subscriber
  ros::Subscriber sub_nav_;

  // Timer
  ros::Timer captain_status_timer_;

  // Actionlib client
  boost::shared_ptr<actionlib::SimpleActionClient<cola2_msgs::WorldSectionAction> > section_client_;

  boost::shared_ptr<actionlib::SimpleActionClient<cola2_msgs::WorldWaypointAction> > waypoint_client_;

  // Thread method to wait for Goto
  boost::thread thread_waypoint_;
  void waitWaypoint();

  std::string section_server_name_;

  // Methods
  bool checkNoRequestRunning();

  void getConfig();

  nav_msgs::Path createPathFromMission(const Mission mission);

  double distanceTo(const double, const double, const double, const double, const bool);

  // ... services callbacks
  bool enableGoto(cola2_msgs::Goto::Request&, cola2_msgs::Goto::Response&);

  bool disableGoto(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  bool enableDefaultMissionNonBlock(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  bool disableMission(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  bool enableKeepPositionHolonomic(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  bool enableKeepPositionNonHolonomic(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  bool disableKeepPosition(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  void updateNav(const cola2_msgs::NavSts& msg);

  bool pauseMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool resumeMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool enableExternalMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool disableExternalMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool enableMission(cola2_msgs::String::Request&, cola2_msgs::String::Response&);

  void callAction(const bool is_empty, const std::string action_id, const std::vector<std::string> parameters);

  bool worldWaypoint(const MissionWaypoint wp);

  bool worldSection(const MissionSection sec);

  bool park(const MissionPark park);

  void captainStatusTimer(const ros::TimerEvent&);

public:
  Captain();
};

Captain::Captain()
  : nh_("~")
  , is_waypoint_running_(false)
  , is_section_running_(false)
  , is_mission_running_(false)
  , is_mission_paused_(false)
  , spinner_(2)
  , is_keep_pose_enabled_(false)
  , diagnostic_(nh_, ros::this_node::getName(), "soft")
{
  // Get config
  getConfig();

  // Init publishers
  pub_path_ = nh_.advertise<nav_msgs::Path>("trajectory_path", 1, true);
  pub_captain_status_ = nh_.advertise<cola2_msgs::CaptainStatus>("status", 1, true);

  // Actionlib client. Smart pointer is used so that client construction is
  // delayed after configuration is loaded
  ROS_INFO_STREAM("Wait for pilot action libs ...");
  section_client_ = boost::shared_ptr<actionlib::SimpleActionClient<cola2_msgs::WorldSectionAction> >(
      new actionlib::SimpleActionClient<cola2_msgs::WorldSectionAction>("world_section_req", true));
  section_client_->waitForServer();  // Wait for infinite time

  waypoint_client_ = boost::shared_ptr<actionlib::SimpleActionClient<cola2_msgs::WorldWaypointAction> >(
      new actionlib::SimpleActionClient<cola2_msgs::WorldWaypointAction>("world_waypoint_req", true));
  waypoint_client_->waitForServer();  // Wait for infinite time
  ROS_INFO_STREAM("Done!");

  // Init services
  // clang-format off
  enable_goto_srv_ = nh_.advertiseService("enable_goto", &Captain::enableGoto, this);
  disable_goto_srv_ = nh_.advertiseService("disable_goto", &Captain::disableGoto, this);
  enable_default_mission_non_block_srv_ = nh_.advertiseService("enable_default_mission_non_block", &Captain::enableDefaultMissionNonBlock, this);
  disable_mission_srv_ = nh_.advertiseService("disable_mission", &Captain::disableMission, this);
  enable_keep_position_holonomic_srv_ = nh_.advertiseService("enable_keep_position_holonomic", &Captain::enableKeepPositionHolonomic, this);
  enable_keep_position_non_holonomic_srv_ = nh_.advertiseService("enable_keep_position_non_holonomic", &Captain::enableKeepPositionNonHolonomic, this);
  disable_keep_position_srv_ = nh_.advertiseService("disable_keep_position", &Captain::disableKeepPosition, this);
  enable_mission_srv_ = nh_.advertiseService("enable_mission", &Captain::enableMission, this);
  pause_mission_srv_ = nh_.advertiseService("pause_mission", &Captain::pauseMission, this);
  resume_mission_srv_ = nh_.advertiseService("resume_mission", &Captain::resumeMission, this);
  enable_external_mission_srv_ = nh_.advertiseService("enable_external_mission", &Captain::enableExternalMission, this);
  disable_external_mission_srv_ = nh_.advertiseService("disable_external_mission", &Captain::disableExternalMission, this);

  // Subscribers
  sub_nav_ = nh_.subscribe(cola2::rosutils::getNamespace() + "/navigator/navigation", 1, &Captain::updateNav, this);

  // Captain Status
  captain_status_.active_controller = 0;
  captain_status_.altitude_mode = false;
  captain_status_.mission_active = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;
  captain_status_timer_ = nh_.createTimer(ros::Duration(2.0), &Captain::captainStatusTimer, this);
  // clang-format on

  spinner_.spin();
}

void Captain::captainStatusTimer(const ros::TimerEvent& event)
{
  captain_status_.mission_active = is_mission_running_;
  pub_captain_status_.publish(captain_status_);
}

void Captain::updateNav(const cola2_msgs::NavSts& msg)
{
  nav_.x = msg.position.north;
  nav_.y = msg.position.east;
  nav_.z = msg.position.depth;
  nav_.yaw = msg.orientation.yaw;
  nav_.altitude = msg.altitude;

  if (is_keep_pose_enabled_)
  {
    diagnostic_.add("keep_position_enabled", "True");
  }
  else
  {
    diagnostic_.add("keep_position_enabled", "False");
  }

  if (is_mission_running_)
  {
    diagnostic_.add("trajectory_enabled", "True");
    diagnostic_.setLevel(diagnostic_msgs::DiagnosticStatus::OK);
  }
  else
  {
    diagnostic_.add("trajectory_enabled", "False");
    diagnostic_.setLevel(diagnostic_msgs::DiagnosticStatus::OK);
  }
}

double Captain::distanceTo(const double x, const double y, const double depth, const double altitude,
                           const bool altitude_mode)
{
  double inc_z;
  if (altitude_mode)
  {
    inc_z = nav_.altitude - altitude;
  }
  else
  {
    inc_z = depth - nav_.z;
  }
  return sqrt(pow(x - nav_.x, 2) + pow(y - nav_.y, 2) + pow(inc_z, 2));
}

void Captain::getConfig()
{
  // Default config here
  section_server_name_ = "section_server";
  double ned_latitude;
  double ned_longitude;
  // Check that NED origin is defined
  if (!cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_latitude", ned_latitude) ||
      !cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_longitude", ned_longitude))
  {
    ROS_ASSERT_MSG(false, "NED origin not found in param server");
  }

  cola2::rosutils::getParam("~max_distance_to_waypoint", config_.max_distance_to_waypoint, 300.0);

  // Get max velocity Z from controller params and goto_max_surge or
  // los_cte_max_surge_velocity to estimate GOTO timeout.
  double surge, heave, surge_los;

  // clang-format off
  cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/controller/max_velocity_z", heave, 0.1);
  cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/pilot/goto_max_surge", surge, 0.1);
  cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/pilot/los_cte_max_surge_velocity", surge_los, 0.1);
  // clang-format on

  min_goto_vel_ = std::min(heave, surge);
  min_loscte_vel_ = std::min(heave, surge_los);

  // Get path were missions are stored
  if (!cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/captain/mission_path", config_.mission_path))
  {
    ROS_ASSERT_MSG(false, "Missions path not found in param server. Set as ./");
    config_.mission_path = ".";
  }
}

bool Captain::enableGoto(cola2_msgs::Goto::Request& req, cola2_msgs::Goto::Response& res)
{
  if (checkNoRequestRunning())
  {
    cola2_msgs::WorldWaypointGoal waypoint;
    waypoint.goal.priority = req.priority;
    waypoint.goal.requester = ros::this_node::getName();
    waypoint.altitude_mode = req.altitude_mode;
    waypoint.altitude = req.altitude;

    // Check req.reference to transform req.position to appropiate reference frame.
    if (req.reference == cola2_msgs::GotoRequest::REFERENCE_NED)
    {
      waypoint.position.north = req.position.x;
      waypoint.position.east = req.position.y;
    }
    else if (req.reference == cola2_msgs::GotoRequest::REFERENCE_GLOBAL)
    {
      double north, east, depth;
      double ned_latitude;
      double ned_longitude;
      // Load NED origin. It can be modified at any time
      if (!cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_latitude", ned_latitude) ||
          !cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_longitude", ned_longitude))
      {
        ROS_ASSERT_MSG(false, "NED origin not found in param server");
      }
      cola2::utils::NED ned(ned_latitude, ned_longitude, 0.0);
      ned.geodetic2Ned(req.position.x, req.position.y, 0.0, north, east, depth);
      waypoint.position.north = north;
      waypoint.position.east = east;
    }
    else
    {
      ROS_WARN_STREAM("Invalid GOTO reference. REFERENCE_VEHICLE not yet implemented.");
    }

    // Check max distance to waypoint
    double distance_to_waypoint =
        distanceTo(waypoint.position.north, waypoint.position.east, req.position.z, req.altitude, req.altitude_mode);

    // Reload max_distance to waypoint. User can modify it!
    cola2::rosutils::getParam("~max_distance_to_waypoint", config_.max_distance_to_waypoint, 300.0);
    if (!(req.disable_axis.x && req.disable_axis.y) && (distance_to_waypoint > config_.max_distance_to_waypoint))
    {
      ROS_WARN_STREAM("Max distance to waypoint is " << config_.max_distance_to_waypoint << " requested waypoint is at "
                                                     << distance_to_waypoint);
      return false;
    }

    waypoint.position.depth = req.position.z;
    waypoint.orientation.yaw = req.yaw;
    waypoint.disable_axis.x = req.disable_axis.x;
    waypoint.disable_axis.y = req.disable_axis.y;
    waypoint.disable_axis.z = req.disable_axis.z;
    waypoint.disable_axis.roll = req.disable_axis.roll;
    waypoint.disable_axis.pitch = req.disable_axis.pitch;
    waypoint.disable_axis.yaw = req.disable_axis.yaw;
    waypoint.position_tolerance.x = req.position_tolerance.x;
    waypoint.position_tolerance.y = req.position_tolerance.y;
    waypoint.position_tolerance.z = req.position_tolerance.z;
    waypoint.orientation_tolerance.yaw = req.orientation_tolerance.yaw;
    waypoint.linear_velocity.x = req.linear_velocity.x;

    if (req.linear_velocity.y != 0.0 || req.linear_velocity.z != 0.0 || req.angular_velocity.yaw != 0.0)
    {
      ROS_WARN_STREAM("GOTO velocity can only be defined in surge. Heave, sway and yaw depend on pose controller.");
    }

    // Choose WorldWaypointReq mode taking into account disable axis & tolerance
    if (req.keep_position && !req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw)
    {
      // Non holonomic keep position
      ROS_INFO_STREAM("ANCHOR mode on!\n");
      waypoint.controller_type = cola2_msgs::WorldWaypointGoal::ANCHOR;
    }
    else if (!req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw)
    {
      // X, Z, Yaw Goto (with or wiyhou Z)
      waypoint.controller_type = cola2_msgs::WorldWaypointGoal::GOTO;
    }
    else if (!req.disable_axis.x && !req.disable_axis.y)
    {
      // Holonomic X, Y, Z, Yaw goto (with or without Z and Yaw)
      waypoint.controller_type = cola2_msgs::WorldWaypointGoal::HOLONOMIC_GOTO;
    }
    else if (req.disable_axis.x && req.disable_axis.y && !req.disable_axis.z)
    {
      // Submerge Z, Yaw (with or without Yaw)
      waypoint.controller_type = cola2_msgs::WorldWaypointGoal::GOTO;
    }

    // Is a goto request to keep position?
    waypoint.keep_position = req.keep_position;

    // Indicate that a waypoint is under execution
    is_waypoint_running_ = true;

    // Compute timeout
    double min_vel = min_goto_vel_;
    if (waypoint.linear_velocity.x != 0.0 and min_vel > waypoint.linear_velocity.x)
    {
      min_vel = waypoint.linear_velocity.x;
    }
    waypoint.timeout = (2.0 * distance_to_waypoint) / min_vel;
    if (waypoint.timeout < 30.0)
    {
      waypoint.timeout = 30.0;
    }
    if (req.timeout > 0 && req.timeout < waypoint.timeout)
    {
      ROS_WARN_STREAM("Warning! Goto request timeout is " << req.timeout << " while computed timeout is "
                                                          << waypoint.timeout << ". Override.\n");
      waypoint.timeout = req.timeout;
    }

    if (req.keep_position)
    {
      if (req.timeout > 0)
      {
        ROS_INFO_STREAM("Keep position TRUE. Setting timeout to GOTO request value: " << req.timeout << "\n");
        waypoint.timeout = req.timeout;
      }
      else
      {
        ROS_INFO_STREAM("Keep position TRUE but timeout GOTO request value is 0. Set timeout to 3600\n");
        waypoint.timeout = 3600;
      }
      // Set active controller to park
      captain_status_.active_controller = 3;
    }
    else
    {
      // Set active controller to waypoint
      captain_status_.active_controller = 1;
    }

    if (waypoint.altitude_mode)
    {
      ROS_INFO_STREAM("Send WorldWaypointRequest at " << waypoint.position.north << ", " << waypoint.position.east << ", " << waypoint.altitude << " altitude. Timeout = " << waypoint.timeout << "\n");
      captain_status_.altitude_mode = true;
    }
    else
    {
      ROS_INFO_STREAM("Send WorldWaypointRequest at " << waypoint.position.north << ", " << waypoint.position.east << ", " << waypoint.position.depth << " depth. Timeout = " << waypoint.timeout << "\n");
      captain_status_.altitude_mode = false;
    }

    waypoint_client_->sendGoal(waypoint);
    res.success = true;
    // If blocking wait for the result
    if (req.blocking)
    {
      waitWaypoint();
    }
    else
    {
      // If goto is not blocking start a thread that waits for result
      // and then sets _is_waypoint_running to false
      thread_waypoint_ = boost::thread(&Captain::waitWaypoint, this);
    }
  }
  else
  {
    res.success = false;
  }
  return true;
}

bool Captain::disableGoto(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  if (is_waypoint_running_)
  {
    waypoint_client_->cancelGoal();
    is_waypoint_running_ = false;
  }
  return true;
}

bool Captain::enableDefaultMissionNonBlock(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  boost::thread* t;
  cola2_msgs::String::Request req;
  cola2_msgs::String::Response res;
  req.request = "last_mission.xml";
  t = new boost::thread(&Captain::enableMission, this, req, res);
  return true;
}

bool Captain::disableMission(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  if (is_waypoint_running_)
  {
    is_waypoint_running_ = false;
    captain_status_.active_controller = 0;
    waypoint_client_->cancelGoal();
  }
  if (is_section_running_)
  {
    is_section_running_ = false;
    captain_status_.active_controller = 0;
    section_client_->cancelGoal();
  }
  if (is_mission_running_)
  {
    is_mission_running_ = false;
    captain_status_.mission_active = false;
  }
  return true;
}

bool Captain::enableKeepPositionHolonomic(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  cola2_msgs::Goto::Request goto_req;
  cola2_msgs::Goto::Response goto_res;

  goto_req.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
  goto_req.altitude_mode = false;
  goto_req.blocking = false;
  goto_req.keep_position = true;
  goto_req.disable_axis.x = false;
  goto_req.disable_axis.y = false;
  goto_req.disable_axis.z = false;
  goto_req.disable_axis.roll = true;
  goto_req.disable_axis.pitch = true;
  goto_req.disable_axis.yaw = false;
  goto_req.position.x = nav_.x;
  goto_req.position.y = nav_.y;
  goto_req.position.z = nav_.z;
  goto_req.yaw = nav_.yaw;

  // If toloerance is 0.0 position, the waypoint is impossible to reach
  // and therefore, the controller will never finish.
  goto_req.position_tolerance.x = 0.0;
  goto_req.position_tolerance.y = 0.0;
  goto_req.position_tolerance.z = 0.0;
  goto_req.orientation_tolerance.yaw = 0.0;

  ROS_INFO_STREAM("Start holonomic keep position at " << nav_.x << ", " << nav_.y << ", " << nav_.z << ", " << nav_.yaw
                                                      << ".\n");
  is_keep_pose_enabled_ = true;
  goto_req.reference = cola2_msgs::Goto::Request::REFERENCE_NED;
  enableGoto(goto_req, goto_res);

  return true;
}

bool Captain::enableKeepPositionNonHolonomic(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // TODO: To be modified for real non-holonomic keep pose controller!

  cola2_msgs::Goto::Request goto_req;
  cola2_msgs::Goto::Response goto_res;

  goto_req.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
  goto_req.altitude_mode = false;
  goto_req.blocking = false;
  goto_req.keep_position = true;
  goto_req.disable_axis.x = false;
  goto_req.disable_axis.y = true;
  goto_req.disable_axis.z = false;
  goto_req.disable_axis.roll = true;
  goto_req.disable_axis.pitch = true;
  goto_req.disable_axis.yaw = false;
  goto_req.position.x = nav_.x;
  goto_req.position.y = nav_.y;
  goto_req.position.z = nav_.z;
  goto_req.yaw = nav_.yaw;

  // If tolerance is 0.0 position, the waypoint is impossible to reach
  // and therefore, the controller will never finish.
  goto_req.position_tolerance.x = 0.0;
  goto_req.position_tolerance.y = 0.0;
  goto_req.position_tolerance.z = 0.0;
  goto_req.orientation_tolerance.yaw = 0.0;

  ROS_INFO_STREAM("Start non holonomic keep position at " << nav_.x << ", " << nav_.y << ", " << nav_.z << ", "
                                                          << nav_.yaw << ".\n");
  is_keep_pose_enabled_ = true;
  goto_req.reference = cola2_msgs::Goto::Request::REFERENCE_NED;
  enableGoto(goto_req, goto_res);

  return true;
}

bool Captain::disableKeepPosition(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (is_keep_pose_enabled_)
  {
    ROS_INFO_STREAM("Disable keep position.");
    disableGoto(req, res);
    is_keep_pose_enabled_ = false;
  }

  return true;
}

void Captain::waitWaypoint()
{
  waypoint_client_->waitForResult();
  is_waypoint_running_ = false;
  captain_status_.active_controller = 0;
  ROS_INFO_STREAM("World Waypoint Request finalized");
}

bool Captain::checkNoRequestRunning()
{
  if (is_waypoint_running_)
  {
    ROS_WARN_STREAM("A World Waypoint Request is running!");
    return false;
  }
  else if (is_section_running_)
  {
    ROS_WARN_STREAM("A World Section Request is running!");
    return false;
  }
  return true;
}

nav_msgs::Path Captain::createPathFromMission(Mission mission)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "ned";
  double ned_latitude;
  double ned_longitude;
  // Load NED origin, it can be modified at any time
  if (!cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_latitude", ned_latitude) ||
      !cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_longitude", ned_longitude))
  {
    ROS_ASSERT_MSG(false, "NED origin not found in param server");
  }

  cola2::utils::NED ned(ned_latitude, ned_longitude, 0.0);
  double x, y, z;
  for (unsigned int i = 0; i < mission.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    ned.geodetic2Ned(mission.getStep(i)->getManeuverPtr()->x(),
                     mission.getStep(i)->getManeuverPtr()->y(),
                     0.0,
                     x, y, z);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = mission.getStep(i)->getManeuverPtr()->z();
    path.poses.push_back(pose);
  }
  return path;
}

bool Captain::pauseMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  waypoint_client_->cancelGoal();
  section_client_->cancelGoal();
  is_mission_paused_ = true;
  return true;
}

bool Captain::resumeMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  is_mission_paused_ = false;
  return true;
}

bool Captain::enableMission(cola2_msgs::String::Request& req, cola2_msgs::String::Response& res)
{
  if (checkNoRequestRunning())
  {
    std::string mission_path = config_.mission_path + "/" + req.request;
    std::cout << "Load mission: " << mission_path << std::endl;
    Mission mission;
    if (mission.loadMission(mission_path) < 0)
    {
      std::cout << "Problem loading mission.\n";
      return false;
    }
    std::cout << "Mission loaded!\n";

    // Publish mission path
    nav_msgs::Path path = createPathFromMission(mission);
    pub_path_.publish(path);

    is_mission_running_ = true;
    captain_status_.mission_active = true;

    for (unsigned int i = 0; i < mission.size(); i++)
    {
      if (is_mission_paused_)
      {
        ROS_WARN_STREAM("MISSION PAUSED\n");
        while (is_mission_paused_ && is_mission_running_)
        {
          ros::Duration(1.0).sleep();
        }
        if (is_mission_running_)
        {
          ROS_WARN_STREAM("MISSION RESUMED\n");
          if (i > 0)
          {
            --i;
          }
        }
      }

      // TODO: Pass north, east, down values
      MissionStep* step = mission.getStep(i);
      std::cout << "Step " << i << std::endl;

      if (!is_mission_running_)
      {
        ROS_WARN_STREAM("disabling mission: executing all remaining actions.");
      }
      else
      {
        // Captain Status
        captain_status_.current_step = i + 1;
        captain_status_.total_steps = mission.size();

        // Play mission step maneuver
        if (step->getManeuverPtr()->getManeuverType() == WAYPOINT_MANEUVER)
        {
          MissionWaypoint* wp = static_cast<MissionWaypoint*>(step->getManeuverPtr());
          // std::cout << *wp << std::endl;
          captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_WAYPOINT;
          if (!this->worldWaypoint(*wp))
          {
            ROS_WARN_STREAM("Impossible to reach waypoint. Move to next mission step.");
          }
        }
        else if (step->getManeuverPtr()->getManeuverType() == SECTION_MANEUVER)
        {
          MissionSection* sec = static_cast<MissionSection*>(step->getManeuverPtr());
          // std::cout << *sec << std::endl;
          captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_SECTION;
          if (!this->worldSection(*sec))
          {
            ROS_WARN_STREAM("Impossible to reach section. Move to next mission step.");
          }
        }
        else if (step->getManeuverPtr()->getManeuverType() == PARK_MANEUVER)
        {
          MissionPark* park = static_cast<MissionPark*>(step->getManeuverPtr());
          // std::cout << *park << std::endl;
          captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_PARK;
          if (!this->park(*park))
          {
            ROS_WARN_STREAM("Impossible to reach park waypoint. Move to next mission step.");
          }
        }
      }
      // Play mission_step actions
      std::vector<MissionAction> actions = step->getActions();
      for (auto action : actions)
      {
        this->callAction(action.getIsEmpty(), action.getActionId(), action.getParameters());
        usleep(2000000);
      }
    }

    if (is_mission_running_)
    {
      ROS_INFO_STREAM("Mission finalized.");
      is_mission_running_ = false;
      captain_status_.mission_active = false;
    }
    else
    {
      ROS_WARN_STREAM("Mission has been disabled.");
    }

    is_mission_paused_ = false;

    // Reset captain status
    captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
    captain_status_.altitude_mode = false;
    captain_status_.mission_active = false;
    captain_status_.current_step = 0;
    captain_status_.total_steps = 0;
  }
  return true;
}

void Captain::callAction(const bool is_empty, const std::string action_id, const std::vector<std::string> parameters)
{
  if (is_empty)
  {
    ros::ServiceClient action_client = nh_.serviceClient<std_srvs::Empty>(action_id);
    std_srvs::Empty params;
    action_client.call(params);
  }
  else
  {
    ros::ServiceClient action_client = nh_.serviceClient<cola2_msgs::Action>(action_id);
    cola2_msgs::Action params;
    for (auto param : parameters)
    {
      params.request.param.push_back(param);
    }
    action_client.call(params);
  }
  ROS_INFO_STREAM("Call -> " << action_id << std::endl);
}

bool Captain::worldWaypoint(const MissionWaypoint wp)
{
  ROS_INFO_STREAM("Execute mission waypoint\n");

  // Define waypoint attributes
  cola2_msgs::Goto::Request goto_req;
  cola2_msgs::Goto::Response goto_res;

  goto_req.altitude = wp.getPosition().getZ();
  goto_req.altitude_mode = wp.getPosition().getAltitudeMode();
  goto_req.linear_velocity.x = wp.getSpeed();
  goto_req.position.x = wp.getPosition().getLatitude();
  goto_req.position.y = wp.getPosition().getLongitude();
  goto_req.position.z = wp.getPosition().getZ();
  goto_req.position_tolerance.x = wp.getTolerance().getX();
  goto_req.position_tolerance.y = wp.getTolerance().getY();
  goto_req.position_tolerance.z = wp.getTolerance().getZ();
  goto_req.blocking = true;
  goto_req.keep_position = false;
  goto_req.disable_axis.x = false;
  goto_req.disable_axis.y = true;
  goto_req.disable_axis.z = false;
  goto_req.disable_axis.roll = true;
  goto_req.disable_axis.yaw = false;
  goto_req.disable_axis.pitch = true;
  goto_req.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
  goto_req.reference = cola2_msgs::Goto::Request::REFERENCE_GLOBAL;

  // Call goto
  return enableGoto(goto_req, goto_res);
}

bool Captain::worldSection(const MissionSection sec)
{
  cola2_msgs::WorldSectionGoal section;
  section.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
  section.controller_type = cola2_msgs::WorldSectionGoal::LOSCTE;
  section.disable_z = false;
  section.tolerance.x = sec.getTolerance().getX();
  section.tolerance.y = sec.getTolerance().getY();
  section.tolerance.z = sec.getTolerance().getZ();

  double initial_north, initial_east, initial_depth;
  double ned_latitude;
  double ned_longitude;
  // Load NED origin, it can be modified at any time
  if (!cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_latitude", ned_latitude) ||
      !cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/navigator/ned_longitude", ned_longitude))
  {
    ROS_ASSERT_MSG(false, "NED origin not found in param server");
  }
  cola2::utils::NED ned(ned_latitude, ned_longitude, 0.0);

  ned.geodetic2Ned(sec.getInitialPosition().getLatitude(), sec.getInitialPosition().getLongitude(), 0.0, initial_north,
                   initial_east, initial_depth);
  section.initial_position.x = initial_north;
  section.initial_position.y = initial_east;
  section.initial_position.z = sec.getInitialPosition().getZ();

  double final_north, final_east, final_depth;
  ned.geodetic2Ned(sec.getFinalPosition().getLatitude(), sec.getFinalPosition().getLongitude(), 0.0, final_north,
                   final_east, final_depth);
  section.final_position.x = final_north;
  section.final_position.y = final_east;
  section.final_position.z = sec.getFinalPosition().getZ();
  section.altitude_mode = sec.getInitialPosition().getAltitudeMode();

  is_section_running_ = true;
  captain_status_.altitude_mode = section.altitude_mode;
  section_client_->sendGoal(section);

  // Compute timeout
  double distance_to_end_section =
      distanceTo(final_north, final_east, sec.getFinalPosition().getZ(), sec.getFinalPosition().getZ(),
                 sec.getInitialPosition().getAltitudeMode());
  double min_vel = min_loscte_vel_;
  if (sec.getSpeed() != 0.0 && sec.getSpeed() < min_loscte_vel_)
  {
    min_vel = sec.getSpeed();
  }
  double timeout = (2 * distance_to_end_section) / min_vel;
  ROS_INFO_STREAM("Section timeout = " << timeout << "\n");
  section_client_->waitForResult(ros::Duration(timeout));
  is_section_running_ = false;
  return true;
}

bool Captain::park(const MissionPark park)
{
  std::cout << "Execute mission park: Reaching park waypoint\n";

  // Define waypoint attributes
  cola2_msgs::Goto::Request goto_req;
  cola2_msgs::Goto::Response goto_res;

  goto_req.altitude = park.getPosition().getZ();
  goto_req.altitude_mode = park.getPosition().getAltitudeMode();
  goto_req.linear_velocity.x = 0.3;  // Fixed velocity when reaching park waypoint
  goto_req.position.x = park.getPosition().getLatitude();
  goto_req.position.y = park.getPosition().getLongitude();
  goto_req.position.z = park.getPosition().getZ();
  goto_req.position_tolerance.x = 3.0;
  goto_req.position_tolerance.y = 3.0;
  goto_req.position_tolerance.z = 1.5;
  goto_req.blocking = true;
  goto_req.keep_position = false;
  goto_req.disable_axis.x = false;
  goto_req.disable_axis.y = true;
  goto_req.disable_axis.z = false;
  goto_req.disable_axis.roll = true;
  goto_req.disable_axis.yaw = false;
  goto_req.disable_axis.pitch = true;
  goto_req.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
  goto_req.reference = cola2_msgs::Goto::Request::REFERENCE_GLOBAL;

  // Call goto
  if (enableGoto(goto_req, goto_res))
  {
    if (is_mission_running_)
    {
      ROS_INFO_STREAM("Execute mission park: Wait for " << park.getTime() << " seconds\n");
      goto_req.keep_position = true;
      goto_req.position_tolerance.x = 0.0;
      goto_req.position_tolerance.y = 0.0;
      goto_req.position_tolerance.z = 0.0;
      goto_req.timeout = park.getTime();
      return enableGoto(goto_req, goto_res);
    }
    return true;
  }
  return false;
}

bool Captain::enableExternalMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  // The only way to know if the mission is controlled by an external process is checking the total_steps
  // Add extra information in captainStatus msg?
  ROS_INFO_STREAM("enable_external_mission service called.");
  if (checkNoRequestRunning())
  {
    is_mission_running_ = true;
    captain_status_.mission_active = true;
    captain_status_.current_step = -1;
    captain_status_.total_steps = -1;
  }
  return true;
}

bool Captain::disableExternalMission(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM("disable_external_mission service called.");
  if (captain_status_.mission_active && captain_status_.total_steps == -1)
  {
    captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
    is_mission_running_ = false;
    captain_status_.mission_active = false;
    captain_status_.current_step = 0;
    captain_status_.total_steps = 0;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "captain");
  Captain captain;
  ros::spin();
  return 0;
}
