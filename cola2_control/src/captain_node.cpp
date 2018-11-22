/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>This node provides control actions to reach waypoints, keep position or execute missions. This node mainly translates user requests to pilot action_libs.<@@*/

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_control/controllers/types.h>
#include <cola2_control/mission_utils/mission.h>
#include <cola2_lib/rosutils/diagnostic_helper.h>
#include <cola2_lib/rosutils/param_loader.h>
#include <cola2_lib/rosutils/this_node.h>
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
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <thread>

/**
 * \brief Captain class. It can execute maneuvers like goto and keep position as well as missions
 */
class Captain
{
 private:
  // ROS
  ros::NodeHandle nh_;

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
  ros::ServiceServer enable_external_mission_srv_;
  ros::ServiceServer disable_external_mission_srv_;
  ros::ServiceServer enable_mission_srv_;

  // Subscriber
  ros::Subscriber sub_nav_;

  // Diagnostics
  cola2::rosutils::DiagnosticHelper diagnostic_;

  // Actionlib client
  actionlib::SimpleActionClient<cola2_msgs::WorldWaypointAction> waypoint_actionlib_;
  actionlib::SimpleActionClient<cola2_msgs::WorldSectionAction> section_actionlib_;
  bool is_waypoint_actionlib_running_, is_section_actionlib_running_;

  // Possible captain states
  enum class CaptainStates {Idle, Goto, Mission, KeepPosition, Backseat};
  CaptainStates state_;

  // Navigation data
  control::Nav nav_;
  bool nav_received_;
  double last_ned_lat_origin_;
  double last_ned_lon_origin_;

  // Captain status timer and data
  cola2_msgs::CaptainStatus captain_status_;
  ros::Timer captain_status_timer_;

  // Threads
  std::thread* thread_wait_waypoint_;
  std::thread* thread_mission_;

  /**
   * \brief Defunes a timer to publish the captain status
   * \param[in] Timer event
   */
  void captainStatusTimer(const ros::TimerEvent&);

  /**
   * \brief Callback to 'namespace'/navigator/navigation topic
   * \param[in] Navigation message
   */
  void updateNav(const cola2_msgs::NavSts&);

  /**
   * \brief Internal goto maneuver helper method
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableGotoInternal(cola2_msgs::Goto::Request&, cola2_msgs::Goto::Response&);

  /**
   * \brief Internal enable mission helper method
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableMissionInternal(cola2_msgs::String::Request&, cola2_msgs::String::Response&);

  /**
   * \brief Computes distance from current position to given x, y, z, altitude, altitude_moe
   * \param[in] Position x
   * \param[in] Position y
   * \param[in] Position z
   * \param[in] Position altitude
   * \param[in] Position altitude_mode
   * \return Distance
   */
  double distanceTo(double, double, double, double, bool);

  /**
   * \brief Blocks the execution thread until the waypoint finalizes
   */
  void waitWaypoint();

  /**
   * \brief Given a mission builds a nav_msgs::Path to represent it in RViz
   * \param[in] Mission
   * \return Path
   */
  nav_msgs::Path createPathFromMission(Mission);

  /**
   * \brief Calls a standard action or an empty service with name const std::string action_id and parameters
   *        const std::vector<std::string> parameters
   * \param[in] Is the action empty?
   * \param[in] Action ID
   * \param[in] Action parameters
   */
  void callAction(bool, const std::string&, std::vector<std::string>);

  /**
   * \brief Executes a world waypoint maneuver defined in a mission
   * \param[in] Mission waypoint
   * \return Success
   */
  bool worldWaypoint(const MissionWaypoint&);

  /**
   * \brief Executes a world section maneuver defined in a mission
   * \param[in] Mission section
   * \return Success
   */
  bool worldSection(const MissionSection&);

  /**
   * \brief Executes park maneuver defined in a mission
   * \param[in] Mission park
   * \return Success
   */
  bool park(const MissionPark&);

  /**
   * \brief Enable goto maneuver
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableGotoSrv(cola2_msgs::Goto::Request&, cola2_msgs::Goto::Response&);

  /**
   * \brief Disable goto maneuver
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool disableGotoSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * \brief Enable mission with name 'last_mission.xml' and returns immediately
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableDefaultMissionNonBlockSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  void enableDefaultMissionNonBlockSrvHelper();  // Helper method required to start a thread with argument references

  /**
   * \brief Disable mission
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool disableMissionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * \brief Enable mission defined in cola2_msgs::String::Request
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableMissionSrv(cola2_msgs::String::Request&, cola2_msgs::String::Response&);

  /**
   * \brief Enable keep position for surge, sway, heave and yaw DoFs
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableKeepPositionHolonomicSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * \brief Enable keep position for surge, heave, and yaw DoFs
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableKeepPositionNonHolonomicSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * \brief Disable keep position
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool disableKeepPositionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * \brief Allows an external controller to 'fake' that a mission is under execution
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool enableExternalMissionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * \brief Finalizes the exeternal mission
   * \param[in] Request
   * \param[out] Response
   * \return Success
   */
  bool disableExternalMissionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

 public:
  /**
   * \brief Class constructor
   */
  Captain();

  /**
   * \brief Class destructor
   */
  ~Captain();
};

Captain::Captain()
  : nh_("~")
  , diagnostic_(nh_, cola2::rosutils::getUnresolvedNodeName(), "soft")
  , waypoint_actionlib_("pilot/world_waypoint_req", true)
  , section_actionlib_("pilot/world_section_req", true)
  , is_waypoint_actionlib_running_(false)
  , is_section_actionlib_running_(false)
  , state_(CaptainStates::Idle)
  , nav_received_(false)
  , thread_wait_waypoint_(NULL)
  , thread_mission_(NULL)
{
  // Publishers
  pub_path_ = nh_.advertise<nav_msgs::Path>("trajectory_path", 1, true);
  pub_captain_status_ = nh_.advertise<cola2_msgs::CaptainStatus>("status", 1, true);

  // Wait actionlib clients
  for (;;)
  {
    if (waypoint_actionlib_.waitForServer(ros::Duration(5.0))) break;
    ROS_INFO_STREAM("Waiting waypoint actionlib");
  }
  for (;;)
  {
    if (section_actionlib_.waitForServer(ros::Duration(5.0))) break;
    ROS_INFO_STREAM("Waiting section actionlib");
  }

  // Services
  enable_goto_srv_ = nh_.advertiseService("enable_goto", &Captain::enableGotoSrv, this);
  disable_goto_srv_ = nh_.advertiseService("disable_goto", &Captain::disableGotoSrv, this);
  enable_keep_position_holonomic_srv_ = nh_.advertiseService("enable_keep_position_holonomic", &Captain::enableKeepPositionHolonomicSrv, this);
  enable_keep_position_non_holonomic_srv_ = nh_.advertiseService("enable_keep_position_non_holonomic", &Captain::enableKeepPositionNonHolonomicSrv, this);
  disable_keep_position_srv_ = nh_.advertiseService("disable_keep_position", &Captain::disableKeepPositionSrv, this);
  enable_mission_srv_ = nh_.advertiseService("enable_mission", &Captain::enableMissionSrv, this);
  enable_default_mission_non_block_srv_ = nh_.advertiseService("enable_default_mission_non_block", &Captain::enableDefaultMissionNonBlockSrv, this);
  disable_mission_srv_ = nh_.advertiseService("disable_mission", &Captain::disableMissionSrv, this);
  enable_external_mission_srv_ = nh_.advertiseService("enable_external_mission",
                                                      &Captain::enableExternalMissionSrv, this);
  disable_external_mission_srv_ = nh_.advertiseService("disable_external_mission",
                                                       &Captain::disableExternalMissionSrv, this);

  // Subscribers
  sub_nav_ = nh_.subscribe(cola2::rosutils::getNamespace() + "/navigator/navigation", 1, &Captain::updateNav, this);

  // Init captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  // Captain status timer
  captain_status_timer_ = nh_.createTimer(ros::Duration(2.0), &Captain::captainStatusTimer, this);

  ROS_INFO_STREAM("Initialized");
}

Captain::~Captain()
{
  // Change state, cancel goals and join threads
  state_ = CaptainStates::Idle;
  if (is_waypoint_actionlib_running_) {
    waypoint_actionlib_.cancelGoal();
    is_waypoint_actionlib_running_ = false;
  }
  if (is_section_actionlib_running_) {
    section_actionlib_.cancelGoal();
    is_section_actionlib_running_ = false;
  }
  if (thread_wait_waypoint_ != NULL) {
    thread_wait_waypoint_->join();
  }
  if (thread_mission_ != NULL) {
    thread_mission_->join();
  }
}

void Captain::captainStatusTimer(const ros::TimerEvent&)
{
  // The mission_active flag is computed here before publishing
  captain_status_.mission_active = (state_ == CaptainStates::Mission) ||
                                   (state_ == CaptainStates::Backseat);
  pub_captain_status_.publish(captain_status_);
}

void Captain::updateNav(const cola2_msgs::NavSts& msg)
{
  // Keep navigation info
  nav_.x = msg.position.north;
  nav_.y = msg.position.east;
  nav_.z = msg.position.depth;
  nav_.yaw = msg.orientation.yaw;
  nav_.altitude = msg.altitude;
  last_ned_lat_origin_ = msg.origin.latitude;
  last_ned_lon_origin_ = msg.origin.longitude;
  nav_received_ = true;

  // Diagnostics are published in navigation callback
  if (state_ == CaptainStates::KeepPosition)
  {
    diagnostic_.add("keep_position_enabled", "True");
  }
  else
  {
    diagnostic_.add("keep_position_enabled", "False");
  }

  if ((state_ == CaptainStates::Mission) || (state_ == CaptainStates::Backseat))
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

bool Captain::enableGotoInternal(cola2_msgs::Goto::Request& req, cola2_msgs::Goto::Response& res)
{
  // Create waypoint
  cola2_msgs::WorldWaypointGoal waypoint;
  waypoint.goal.priority = req.priority;
  waypoint.goal.requester = ros::this_node::getName();
  waypoint.altitude_mode = req.altitude_mode;
  waypoint.altitude = req.altitude;

  // Check req.reference to transform req.position to appropiate reference frame
  if (req.reference == cola2_msgs::GotoRequest::REFERENCE_NED)
  {
    waypoint.position.north = req.position.x;
    waypoint.position.east = req.position.y;
  }
  else if (req.reference == cola2_msgs::GotoRequest::REFERENCE_GLOBAL)
  {
    // Convert waypoint data
    cola2::utils::NED ned(last_ned_lat_origin_, last_ned_lon_origin_, 0.0);
    double north, east, depth;
    ned.geodetic2Ned(req.position.x, req.position.y, 0.0, north, east, depth);
    waypoint.position.north = north;
    waypoint.position.east = east;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid GOTO reference. REFERENCE_VEHICLE not yet implemented");
    state_ = CaptainStates::Idle;
    res.success = false;
    return false;
  }

  // Check max distance to waypoint
  double distance_to_waypoint =
      distanceTo(waypoint.position.north, waypoint.position.east, req.position.z, req.altitude, req.altitude_mode);

  // Load max distance to waypoint from param server and check request
  double max_distance_to_waypoint;
  cola2::rosutils::getParam("~max_distance_to_waypoint", max_distance_to_waypoint, 300.0);  // Default value
  if (!(req.disable_axis.x && req.disable_axis.y) && (distance_to_waypoint > max_distance_to_waypoint))
  {
    ROS_ERROR_STREAM("Invalid request. Max distance to waypoint is " << max_distance_to_waypoint <<
                     " while requested waypoint distance is at " << distance_to_waypoint);
    state_ = CaptainStates::Idle;
    res.success = false;
    return false;
  }

  // Copy data from request to our waypoint
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

  // Display warning if sway, heave or yaw velocities are given
  if (req.linear_velocity.y != 0.0 || req.linear_velocity.z != 0.0 || req.angular_velocity.yaw != 0.0)
  {
    ROS_WARN_STREAM("GOTO velocity can only be defined in surge. Heave, sway and yaw depend on pose controller");
  }

  // Choose WorldWaypointReq mode taking into account the disable_axis and tolerance
  if (req.keep_position && !req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw)
  {
    // Non holonomic keep position. Anchor mode
    waypoint.controller_type = cola2_msgs::WorldWaypointGoal::ANCHOR;
  }
  else if (!req.disable_axis.x && req.disable_axis.y && !req.disable_axis.yaw)
  {
    // Goto using x and yaw, with or without z
    waypoint.controller_type = cola2_msgs::WorldWaypointGoal::GOTO;
  }
  else if (!req.disable_axis.x && !req.disable_axis.y)
  {
    // Holonomic goto with x and y, with or without z and yaw
    waypoint.controller_type = cola2_msgs::WorldWaypointGoal::HOLONOMIC_GOTO;
  }
  else if (req.disable_axis.x && req.disable_axis.y && !req.disable_axis.z)
  {
    // Submerge z, with or without yaw
    waypoint.controller_type = cola2_msgs::WorldWaypointGoal::GOTO;
  }

  // Is it a keep position?
  waypoint.keep_position = req.keep_position;

  // Update captain status and set waypoint timeout
  if (req.keep_position)
  {
    // Set active controller to park
    captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_PARK;

    // Check timeout
    if (req.timeout > 0)
    {
      ROS_INFO_STREAM("Keep position TRUE. Setting timeout to " << req.timeout << " seconds");
      waypoint.timeout = static_cast<double>(req.timeout);
    }
    else
    {
      ROS_INFO_STREAM("Keep position TRUE but the requested timeout is 0. Keeping position for 3600 seconds");
      waypoint.timeout = 3600.0;
    }
  }
  else
  {
    // Set active controller to waypoint
    captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_WAYPOINT;

    // Compute timeout
    double surge, heave;
    cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/controller/max_velocity_z", heave, 0.1);
    cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/pilot/goto/max_surge", surge, 0.1);
    double min_vel = std::min(heave, surge);
    if ((waypoint.linear_velocity.x != 0.0) &&
        (min_vel > waypoint.linear_velocity.x)) min_vel = waypoint.linear_velocity.x;
    waypoint.timeout = 2.0 * distance_to_waypoint / min_vel;
    if (waypoint.timeout < 30.0) waypoint.timeout = 30.0;
    if ((req.timeout > 0) && (static_cast<double>(req.timeout) < waypoint.timeout))
    {
      ROS_WARN_STREAM("Goto request timeout is " << req.timeout << " while computed timeout is " <<
                      waypoint.timeout << ". Taking requested timeout");
      waypoint.timeout = static_cast<double>(req.timeout);
    }
  }

  // Check altitude mode
  if (waypoint.altitude_mode)
  {
    ROS_INFO_STREAM("Send waypoint request at [" << waypoint.position.north << ", " << waypoint.position.east
                                                     << "] with altitude " << waypoint.altitude
                                                     << ". Timeout = " << waypoint.timeout);
    captain_status_.altitude_mode = true;
  }
  else
  {
    ROS_INFO_STREAM("Send waypoint request at [" << waypoint.position.north << ", " << waypoint.position.east
                                                     << "] with depth " << waypoint.position.depth
                                                     << ". Timeout = " << waypoint.timeout);
    captain_status_.altitude_mode = false;
  }

  // Call actionlib
  is_waypoint_actionlib_running_ = true;
  waypoint_actionlib_.sendGoal(waypoint);

  // Wait, or start a thread that waits for results
  if (req.blocking)
  {
    bool result = false;
    while ((!result) && (!ros::isShuttingDown()))
    {
      result = waypoint_actionlib_.waitForResult(ros::Duration(1.0));
    }
    is_waypoint_actionlib_running_ = false;
  }
  else
  {
    // Join previous thread if present, which should be finished by now (if the state machine is correct)
    if (thread_wait_waypoint_ != NULL) thread_wait_waypoint_->join();

    // Start new thread
    thread_wait_waypoint_ = new std::thread(&Captain::waitWaypoint, this);
  }

  res.success = true;
  return true;
}

bool Captain::enableMissionInternal(cola2_msgs::String::Request& req, cola2_msgs::String::Response&)
{
  // Get path were missions are stored
  std::string package;
  if (!cola2::rosutils::getParam("~vehicle_config_launch_mission_package", package))
  {
    ROS_ERROR_STREAM("Package vehicle_config_launch_mission not defined!");
    state_ = CaptainStates::Idle;
    return false;
  }
  std::string package_path = ros::package::getPath(package);
  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Error defining mission path!");
    state_ = CaptainStates::Idle;
    return false;
  }
  std::string mission_path = package_path + "/missions/" + req.request;

  // Load mission
  ROS_INFO_STREAM("Loading mission: " << mission_path);
  Mission mission;
  bool valid_mission = true;
  try
  {
      valid_mission = (mission.loadMission(mission_path) >= 0);
  }
  catch (...)
  {
    valid_mission = false;
  }
  if (!valid_mission)
  {
    ROS_ERROR_STREAM("Problem loading mission");
    state_ = CaptainStates::Idle;
    return false;
  }

  ROS_INFO_STREAM("Mission loaded");

  // Publish mission path
  nav_msgs::Path path = createPathFromMission(mission);
  pub_path_.publish(path);

  for (std::size_t i = 0; i < mission.size(); i++)
  {
    if (ros::isShuttingDown()) return true;

    // Get step pointer
    MissionStep* step = mission.getStep(i);
    ROS_INFO_STREAM("Mission step " << i);

    if (state_ != CaptainStates::Mission)
    {
      ROS_WARN_STREAM("Disabling mission. Executing all remaining actions");
    }
    else
    {
      // Captain status
      captain_status_.current_step = i + 1;
      captain_status_.total_steps = mission.size();

      // Play mission step maneuver
      if (step->getManeuverPtr()->getManeuverType() == WAYPOINT_MANEUVER)  // This comes from mission_maneuver.h
      {
        auto* wp = static_cast<MissionWaypoint*>(step->getManeuverPtr());
        captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_WAYPOINT;
        captain_status_.altitude_mode = wp->getPosition().getAltitudeMode();
        if (!this->worldWaypoint(*wp))
        {
          ROS_WARN_STREAM("Impossible to reach waypoint. Move to next mission step");
        }
      }
      else if (step->getManeuverPtr()->getManeuverType() == SECTION_MANEUVER)
      {
        auto* sec = static_cast<MissionSection*>(step->getManeuverPtr());
        captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_SECTION;
        captain_status_.altitude_mode = sec->getInitialPosition().getAltitudeMode();
        if (!this->worldSection(*sec))
        {
          ROS_WARN_STREAM("Impossible to reach section. Move to next mission step");
        }
      }
      else if (step->getManeuverPtr()->getManeuverType() == PARK_MANEUVER)
      {
        auto* park = static_cast<MissionPark*>(step->getManeuverPtr());
        captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_PARK;
        captain_status_.altitude_mode = park->getPosition().getAltitudeMode();
        if (!this->park(*park))
        {
          ROS_WARN_STREAM("Impossible to reach park waypoint. Move to next mission step");
        }
      }
    }

    // Play mission_step actions
    std::vector<MissionAction> actions = step->getActions();
    for (const auto& action : actions)
    {
      if (ros::isShuttingDown()) return true;
      this->callAction(action.getIsEmpty(), action.getActionId(), action.getParameters());
      ros::Duration(2.0).sleep();
    }
  }

  // Display info
  if (state_ == CaptainStates::Mission)
  {
    ROS_INFO_STREAM("Mission finalized");
  }
  else
  {
    ROS_WARN_STREAM("Mission has been disabled");
  }

  // Set captain state
  state_ = CaptainStates::Idle;

  // Reset captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  return true;
}

double Captain::distanceTo(const double x, const double y, const double depth, const double altitude,
                           const bool altitude_mode)
{
  double inc_z = altitude_mode ? (altitude - nav_.altitude) : (depth - nav_.z);
  return std::sqrt(std::pow(x - nav_.x, 2) + std::pow(y - nav_.y, 2) + std::pow(inc_z, 2));
}

void Captain::waitWaypoint()
{
  bool result = false;
  while ((!result) && (!ros::isShuttingDown()))
  {
    result = waypoint_actionlib_.waitForResult(ros::Duration(1.0));
  }
  is_waypoint_actionlib_running_ = false;
  state_ = CaptainStates::Idle;
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;
}

nav_msgs::Path Captain::createPathFromMission(Mission mission)
{
  // Create path from mission using NED
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world_ned";
  cola2::utils::NED ned(last_ned_lat_origin_, last_ned_lon_origin_, 0.0);
  for (std::size_t i = 0; i < mission.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    double x, y, z;
    ned.geodetic2Ned(mission.getStep(i)->getManeuverPtr()->x(), mission.getStep(i)->getManeuverPtr()->y(), 0.0,
                     x, y, z);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = mission.getStep(i)->getManeuverPtr()->z();
    path.poses.push_back(pose);
  }
  return path;
}

void Captain::callAction(const bool is_empty, const std::string &action_id, const std::vector<std::string> parameters)
{
  ROS_INFO_STREAM("Calling id -> " << action_id);
  if (is_empty)
  {
    ROS_INFO_STREAM("Calling a trigger service");
    ros::ServiceClient action_client = nh_.serviceClient<std_srvs::Trigger>(action_id);
    std_srvs::Trigger params;
    action_client.call(params);
  }
  else
  {
    ROS_INFO_STREAM("Calling an action service");
    ros::ServiceClient action_client = nh_.serviceClient<cola2_msgs::Action>(action_id);
    cola2_msgs::Action params;
    for (const auto &param : parameters)
    {
      params.request.param.push_back(param);
    }
    action_client.call(params);
  }
}

bool Captain::worldWaypoint(const MissionWaypoint &wp)
{
  ROS_INFO_STREAM("Execute mission waypoint");

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
  return enableGotoInternal(goto_req, goto_res);
}

bool Captain::worldSection(const MissionSection &sec)
{
  ROS_INFO_STREAM("Execute mission section");

  // Define section attributes
  cola2_msgs::WorldSectionGoal section;
  section.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
  section.controller_type = cola2_msgs::WorldSectionGoal::LOSCTE;
  section.disable_z = false;
  section.tolerance.x = sec.getTolerance().getX();
  section.tolerance.y = sec.getTolerance().getY();
  section.tolerance.z = sec.getTolerance().getZ();
  section.surge_velocity = sec.getSpeed();

  // Initial position
  double initial_north, initial_east, initial_depth;
  cola2::utils::NED ned(last_ned_lat_origin_, last_ned_lon_origin_, 0.0);
  ned.geodetic2Ned(sec.getInitialPosition().getLatitude(), sec.getInitialPosition().getLongitude(), 0.0,
                   initial_north, initial_east, initial_depth);
  section.initial_position.x = initial_north;
  section.initial_position.y = initial_east;
  section.initial_position.z = sec.getInitialPosition().getZ();

  // Final position
  double final_north, final_east, final_depth;
  ned.geodetic2Ned(sec.getFinalPosition().getLatitude(), sec.getFinalPosition().getLongitude(), 0.0,
                   final_north, final_east, final_depth);
  section.final_position.x = final_north;
  section.final_position.y = final_east;
  section.final_position.z = sec.getFinalPosition().getZ();

  // Altitude is taken from the initial position
  section.altitude_mode = sec.getInitialPosition().getAltitudeMode();

  // Compute timeout
  double heave, surge_los;
  cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/controller/max_velocity_z", heave, 0.1);
  cola2::rosutils::getParam(cola2::rosutils::getNamespace() + "/pilot/los_cte/max_surge_velocity", surge_los, 0.1);
  double min_vel = std::min(heave, surge_los);
  if (sec.getSpeed() != 0.0 && sec.getSpeed() < min_vel) min_vel = sec.getSpeed();
  double distance_to_end_section =
      distanceTo(final_north, final_east, sec.getFinalPosition().getZ(), sec.getFinalPosition().getZ(),
                 sec.getInitialPosition().getAltitudeMode());
  double timeout = 2.0 * distance_to_end_section / min_vel;
  ROS_INFO_STREAM("Section timeout = " << timeout);
  section.timeout = timeout;

  // Send section using actionlib
  is_section_actionlib_running_ = true;
  section_actionlib_.sendGoal(section);

  // Wait for result or cancel if timed out
  bool result = false;
  while ((!result) && (!ros::isShuttingDown()))
  {
    result = section_actionlib_.waitForResult(ros::Duration(1.0));
  }
  is_section_actionlib_running_ = false;

  return true;
}

bool Captain::park(const MissionPark &park)
{
  // Define waypoint attributes
  cola2_msgs::Goto::Request goto_req;
  cola2_msgs::Goto::Response goto_res;
  goto_req.altitude = static_cast<float>(park.getPosition().getZ());
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
  ROS_INFO_STREAM("Executing mission park. Reaching park waypoint");
  if (enableGotoInternal(goto_req, goto_res))
  {
    // Check if it still running to do the second part
    if (state_ == CaptainStates::Mission)
    {
      ROS_INFO_STREAM("Executing mission park. Wait for " << park.getTime() << " seconds");
      goto_req.keep_position = true;
      goto_req.position_tolerance.x = 0.0;
      goto_req.position_tolerance.y = 0.0;
      goto_req.position_tolerance.z = 0.0;
      goto_req.timeout = static_cast<unsigned short>(park.getTime());
      return enableGotoInternal(goto_req, goto_res);
    }
    return true;
  }
  return false;
}


bool Captain::enableGotoSrv(cola2_msgs::Goto::Request& req, cola2_msgs::Goto::Response& res)
{
  // Initialize success to false
  res.success = false;

  // Check navigation
  if (!nav_received_)
  {
    ROS_ERROR_STREAM("Impossible to enable goto. Navigation not received yet");
    return true;
  }

  // Check current state
  if (state_ != CaptainStates::Idle)
  {
    ROS_ERROR_STREAM("Impossible to enable goto. Something is already running");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Goto;

  // Set captain status
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  // Call enable goto
  enableGotoInternal(req, res);

  // If it was blocking, put the state back to Idle now
  if (req.blocking)
  {
    state_ = CaptainStates::Idle;
    captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
    captain_status_.altitude_mode = false;
    captain_status_.current_step = 0;
    captain_status_.total_steps = 0;
  }

  return true;
}

bool Captain::disableGotoSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check current state
  if (state_ != CaptainStates::Goto)
  {
    ROS_WARN_STREAM("Impossible to disable goto. Captain not in goto state");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Idle;

  // Cancel actionlib if necessary
  if (is_waypoint_actionlib_running_)
  {
    waypoint_actionlib_.cancelGoal();
    is_waypoint_actionlib_running_ = false;
  }

  // Set captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  return true;
}

bool Captain::enableMissionSrv(cola2_msgs::String::Request& req, cola2_msgs::String::Response& res)
{
  // Check navigation
  if (!nav_received_)
  {
    ROS_ERROR_STREAM("Impossible to enable mission. Navigation not received yet");
    return true;
  }

  // Check current state
  if (state_ != CaptainStates::Idle)
  {
    ROS_ERROR_STREAM("Impossible to enable mission. Something is already running");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Mission;

  // Set captain status
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  // Call enable goto
  enableMissionInternal(req, res);

  return true;
}

bool Captain::enableDefaultMissionNonBlockSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check navigation
  if (!nav_received_)
  {
    ROS_ERROR_STREAM("Impossible to enable default mission. Navigation not received yet");
    return true;
  }

  // Check current state
  if (state_ != CaptainStates::Idle)
  {
    ROS_ERROR_STREAM("Impossible to enable default mission. Something is already running");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Mission;

  // Start thread joining first the previous one, which should be done by now (if the state machine is correct)
  if (thread_mission_ != NULL) thread_mission_->join();
  thread_mission_ = new std::thread(&Captain::enableDefaultMissionNonBlockSrvHelper, this);

  return true;
}

void Captain::enableDefaultMissionNonBlockSrvHelper()
{
  // Helper method. It is needed because enableMission() has reference arguments
  cola2_msgs::String::Request req;
  cola2_msgs::String::Response res;
  req.request = "last_mission.xml";
  enableMissionInternal(req, res);
}

bool Captain::disableMissionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check current state
  if (state_ != CaptainStates::Mission)
  {
    ROS_WARN_STREAM("Impossible to disable mission. Captain not in mission state");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Idle;

  // Cancel actionlibs if necessary
  if (is_waypoint_actionlib_running_)
  {
    waypoint_actionlib_.cancelGoal();
    is_waypoint_actionlib_running_ = false;
  }
  if (is_section_actionlib_running_)
  {
    section_actionlib_.cancelGoal();
    is_section_actionlib_running_ = false;
  }

  // Set captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  return true;
}

bool Captain::enableKeepPositionHolonomicSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check navigation
  if (!nav_received_)
  {
    ROS_ERROR_STREAM("Impossible to enable keep position. Navigation not received yet");
    return true;
  }

  // Check current state
  if (state_ != CaptainStates::Idle)
  {
    ROS_ERROR_STREAM("Impossible to enable keep position. Something is already running");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::KeepPosition;

  // Set captain status
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  // Call goto with appropriate parameters
  ROS_INFO_STREAM("Start holonomic keep position at [" <<
                  nav_.x << ", " << nav_.y << ", " << nav_.z << "] with orientation " << nav_.yaw);

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
  goto_req.yaw = static_cast<float>(nav_.yaw);

  // If toloerance is 0.0 position, the waypoint is impossible to reach
  // and therefore, the controller will never finish
  goto_req.position_tolerance.x = 0.0;
  goto_req.position_tolerance.y = 0.0;
  goto_req.position_tolerance.z = 0.0;
  goto_req.orientation_tolerance.yaw = 0.0;
  goto_req.reference = cola2_msgs::Goto::Request::REFERENCE_NED;
  enableGotoInternal(goto_req, goto_res);

  return true;
}

bool Captain::enableKeepPositionNonHolonomicSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check navigation
  if (!nav_received_)
  {
    ROS_ERROR_STREAM("Impossible to enable keep position. Navigation not received yet");
    return true;
  }

  // Check current state
  if (state_ != CaptainStates::Idle)
  {
    ROS_ERROR_STREAM("Impossible to enable keep position. Something is already running");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::KeepPosition;

  // Set captain status
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  // Call goto with appropriate parameters
  ROS_INFO_STREAM("Start non holonomic keep position at [" <<
                  nav_.x << ", " << nav_.y << ", " << nav_.z << "] with orientation " << nav_.yaw);

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
  goto_req.yaw = static_cast<float>(nav_.yaw);

  // If tolerance is 0.0 position, the waypoint is impossible to reach
  // and therefore, the controller will never finish
  goto_req.position_tolerance.x = 0.0;
  goto_req.position_tolerance.y = 0.0;
  goto_req.position_tolerance.z = 0.0;
  goto_req.orientation_tolerance.yaw = 0.0;
  goto_req.reference = cola2_msgs::Goto::Request::REFERENCE_NED;
  enableGotoInternal(goto_req, goto_res);

  return true;
}

bool Captain::disableKeepPositionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check current state
  if (state_ != CaptainStates::KeepPosition)
  {
    ROS_WARN_STREAM("Impossible to disable keep position. Captain not in keep position state");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Idle;

  // Cancel actionlib if necessary
  if (is_waypoint_actionlib_running_)
  {
    waypoint_actionlib_.cancelGoal();
    is_waypoint_actionlib_running_ = false;
  }

  // Set captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  return true;
}


bool Captain::enableExternalMissionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check current state
  if (state_ != CaptainStates::Idle)
  {
    ROS_ERROR_STREAM("Impossible to enable external mission. Something is already running");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Backseat;

  // Set captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  return true;
}

bool Captain::disableExternalMissionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Check current state
  if (state_ != CaptainStates::Backseat)
  {
    ROS_WARN_STREAM("Impossible to disable external mission. Captain not in external mission state");
    return true;
  }

  // Set captain state
  state_ = CaptainStates::Idle;

  // Set captain status
  captain_status_.active_controller = cola2_msgs::CaptainStatus::CONTROLLER_NONE;
  captain_status_.altitude_mode = false;
  captain_status_.current_step = 0;
  captain_status_.total_steps = 0;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "captain");
  Captain captain;
  ros::MultiThreadedSpinner spinner(2);  // TODO: a mutex should be used to access shared resources
  spinner.spin();
  return 0;
}
