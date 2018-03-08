
/*
 * Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>Directed by the captain, publishes position and velocity
setpoints to the position and velocity controllers.<@@*/

#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <actionlib/server/simple_action_server.h>
#include <cola2_msgs/WorldSectionAction.h>
#include <cola2_msgs/WorldWaypointAction.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/WorldWaypointReq.h>
#include <cola2_msgs/BodyVelocityReq.h>
#include <visualization_msgs/Marker.h>
#include <cola2_control/controllers/types.h>
#include <cola2_control/controllers/los_cte.h>
#include <cola2_control/controllers/goto.h>
#include <cola2_control/controllers/holonomic_goto.h>
#include <cola2_control/controllers/anchor.h>
#include <cola2_lib/rosutils/param_loader.h>
#include <cola2_lib/rosutils/diagnostic_helper.h>
#include <dynamic_reconfigure/server.h>
//#include <cola2_control/PilotConfig.h>
#include <geometry_msgs/PointStamped.h>
#include <cola2_lib/rosutils/this_node.h>

const unsigned int SECTION_MODE = 0;
const unsigned int WAYPOINT_MODE = 1;
const unsigned int PATH_MODE = 2;

class Pilot
{
private:
  // Node handle
  ros::NodeHandle nh_;

  // ROS variables
  ros::Subscriber sub_nav_;
  ros::Publisher pub_wwr_;
  ros::Publisher pub_bvr_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_goal_;

  // Actionlib servers
  boost::shared_ptr<actionlib::SimpleActionServer<cola2_msgs::WorldSectionAction> > section_server_;
  boost::shared_ptr<actionlib::SimpleActionServer<cola2_msgs::WorldWaypointAction> > waypoint_server_;

  // Reconfigure parameters
  // dynamic_reconfigure::Server<cola2_control::PilotConfig> _param_server;
  // dynamic_reconfigure::Server<cola2_control::PilotConfig>::CallbackType _f;

  // Other vars
  control::State current_state_;

  // Controllers
  std::unique_ptr<LosCteController> los_cte_controller_;
  std::unique_ptr<GotoController> goto_controller_;
  std::unique_ptr<HolonomicGotoController> holonomic_goto_controller_;
  std::unique_ptr<AnchorController> anchor_controller_;

  // Mutex between section, waypoint and path controllers
  // TODO: To be done

  // Config
  struct
  {
    LosCteControllerConfig los_cte_config;
    GotoControllerConfig goto_config;
    HolonomicGotoControllerConfig holonomic_goto_config;
    AnchorControllerConfig anchor_config;
  } config_;

  // Methods
  void navCallback(const cola2_msgs::NavSts&);
  void sectionServerCallback(const cola2_msgs::WorldSectionGoalConstPtr&);
  void waypointServerCallback(const cola2_msgs::WorldWaypointGoalConstPtr&);
  void publishControlCommands(const control::State&, unsigned int);
  void publishFeedback(const control::Feedback&, unsigned int);
  void publishMarker(double, double, double);
  void publishMarkerSections(const control::PointsList);
  void getConfig();
  //void setParams(cola2_control::PilotConfig&, uint32_t);
  void publishGoal(const double, const double, const double);

public:
  Pilot();
};

Pilot::Pilot(): nh_("~")
{
  // Get config
  getConfig();

  // Initialize controllers
  // Line of Sight with Cross Tracking Error Controller
  los_cte_controller_ = std::unique_ptr<LosCteController>(new LosCteController(config_.los_cte_config));
  // Go to waypoint
  goto_controller_ = std::unique_ptr<GotoController>(new GotoController(config_.goto_config));
  // Holonomic Goto waypoint
  holonomic_goto_controller_ = std::unique_ptr<HolonomicGotoController>(new HolonomicGotoController(config_.holonomic_goto_config));
  // Anchor controller (for keep position in non holonomic vehicles)
  anchor_controller_ = std::unique_ptr<AnchorController>(new AnchorController(config_.anchor_config));

  // Publishers
  pub_wwr_ = nh_.advertise<cola2_msgs::WorldWaypointReq>(cola2::rosutils::getNamespace() + "/controller/world_waypoint_req", 1);
  pub_bvr_ = nh_.advertise<cola2_msgs::BodyVelocityReq>(cola2::rosutils::getNamespace() + "/controller/body_velocity_req", 1);
  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
  pub_goal_ = nh_.advertise<geometry_msgs::PointStamped>("goal", 1, true);

  // Subscriber
  sub_nav_ = nh_.subscribe(cola2::rosutils::getNamespace() + "/navigator/navigation", 1, &Pilot::navCallback, this);

  // Actionlib server. Smart pointer is used so that server construction is
  // delayed after configuration is loaded
  // SECTION action lib
  section_server_ = boost::shared_ptr<actionlib::SimpleActionServer<cola2_msgs::WorldSectionAction> >(
      new actionlib::SimpleActionServer<cola2_msgs::WorldSectionAction>(
          nh_, "world_section_req", boost::bind(&Pilot::sectionServerCallback, this, _1), false));
  section_server_->start();

  // WAYPOINT action lib
  waypoint_server_ = boost::shared_ptr<actionlib::SimpleActionServer<cola2_msgs::WorldWaypointAction> >(
      new actionlib::SimpleActionServer<cola2_msgs::WorldWaypointAction>(
          nh_, "world_waypoint_req", boost::bind(&Pilot::waypointServerCallback, this, _1), false));
  waypoint_server_->start();

  // Init dynamic reconfigure
  // f_ = boost::bind(&Pilot::setParams, this, _1, _2);
  // param_server_.setCallback(f_);

  // Display message
  ROS_INFO_STREAM("Initialized.");

  ros::spin();
}

void Pilot::navCallback(const cola2_msgs::NavSts& data)
{
  // Obtain navigation data
  current_state_.pose.position.north = data.position.north;
  current_state_.pose.position.east = data.position.east;
  current_state_.pose.position.depth = data.position.depth;
  current_state_.pose.altitude = data.altitude;
  current_state_.pose.orientation.roll = data.orientation.roll;
  current_state_.pose.orientation.pitch = data.orientation.pitch;
  current_state_.pose.orientation.yaw = data.orientation.yaw;
  current_state_.velocity.linear.x = data.body_velocity.x;
  current_state_.velocity.linear.y = data.body_velocity.y;
  current_state_.velocity.linear.z = data.body_velocity.z;
}

void Pilot::waypointServerCallback(const cola2_msgs::WorldWaypointGoalConstPtr& data)
{
  // TODO: Avoid having a waypoint, or a section controller running simultaneously

  // Conversion from actionlib goal to internal Section type
  control::Waypoint waypoint;
  waypoint.altitude = data->altitude;
  waypoint.altitude_mode = data->altitude_mode;
  waypoint.controller_type = data->controller_type;
  waypoint.disable_axis.x = data->disable_axis.x;
  waypoint.disable_axis.y = data->disable_axis.y;
  waypoint.disable_axis.z = data->disable_axis.z;
  waypoint.disable_axis.roll = data->disable_axis.roll;
  waypoint.disable_axis.pitch = data->disable_axis.pitch;
  waypoint.disable_axis.yaw = data->disable_axis.yaw;
  waypoint.position.north = data->position.north;
  waypoint.position.east = data->position.east;
  waypoint.position.depth = data->position.depth;
  waypoint.orientation.roll = data->orientation.roll;
  waypoint.orientation.pitch = data->orientation.pitch;
  waypoint.orientation.yaw = data->orientation.yaw;
  waypoint.position_tolerance.x = data->position_tolerance.x;
  waypoint.position_tolerance.y = data->position_tolerance.y;
  waypoint.position_tolerance.z = data->position_tolerance.z;
  waypoint.orientation_tolerance.roll = data->orientation_tolerance.roll;
  waypoint.orientation_tolerance.pitch = data->orientation_tolerance.pitch;
  waypoint.orientation_tolerance.yaw = data->orientation_tolerance.yaw;
  waypoint.priority = data->goal.priority;
  waypoint.requester = data->goal.requester;
  waypoint.timeout = data->timeout;
  waypoint.linear_velocity.x = data->linear_velocity.x;
  waypoint.linear_velocity.y = data->linear_velocity.y;
  waypoint.linear_velocity.z = data->linear_velocity.z;
  waypoint.angular_velocity.roll = data->angular_velocity.roll;
  waypoint.angular_velocity.pitch = data->angular_velocity.pitch;
  waypoint.angular_velocity.yaw = data->angular_velocity.yaw;

  // Main loop
  double init_time = ros::Time::now().toSec();
  ros::Rate r(10);  // 10Hz
  while (ros::ok())
  {
    // Declare some vars
    control::State controller_output;
    control::Feedback feedback;
    cola2_msgs::WorldWaypointResult result_msg;
    control::PointsList points;

    // Run controller
    try
    {
      switch (data->controller_type)
      {
        case cola2_msgs::WorldWaypointGoal::GOTO:
          ROS_DEBUG_STREAM("GOTO controller");
          goto_controller_->compute(current_state_, waypoint, controller_output, feedback, points);
          break;
        case cola2_msgs::WorldWaypointGoal::HOLONOMIC_GOTO:
          ROS_DEBUG_STREAM("HOLONOMIC_GOTO controller");
          holonomic_goto_controller_->compute(current_state_, waypoint, controller_output, feedback, points);
          break;
        case cola2_msgs::WorldWaypointGoal::ANCHOR:
          ROS_DEBUG_STREAM("ANCHOR controller");
          anchor_controller_->compute(current_state_, waypoint, controller_output, feedback, points);
          break;
        default:
          std::cout << "Controller: " << data->controller_type << "\n";
          throw std::runtime_error("Unknown controller");
      }
    }
    catch (std::exception& e)
    {
      // Check for failure
      ROS_ERROR_STREAM("Controller failure\n" << e.what());
      result_msg.final_status = cola2_msgs::WorldWaypointResult::FAILURE;
      waypoint_server_->setAborted(result_msg);
      break;
    }

    // Publishers
    publishControlCommands(controller_output, data->goal.priority);
    publishFeedback(feedback, WAYPOINT_MODE);
    publishMarker(waypoint.position.north, waypoint.position.east, waypoint.position.depth);
    publishMarkerSections(points);
    publishGoal(waypoint.position.north, waypoint.position.east, waypoint.position.depth);

    // Check for success
    if (feedback.success)
    {
      ROS_INFO_STREAM("Waypoint success");
      result_msg.final_status = cola2_msgs::WorldWaypointResult::SUCCESS;
      waypoint_server_->setSucceeded(result_msg);
      break;
    }

    // Check for preempted. This happens upon user request (by preempting
    // or cancelling the goal, or when a new SectionGoal is received
    if (waypoint_server_->isPreemptRequested())
    {
      ROS_INFO_STREAM("Waypoint preempted");
      waypoint_server_->setPreempted();
      break;
    }

    // Check for timeout --> If keep position, timeout = 0.0
    if (data->timeout > 0.0)
    {
      if ((ros::Time::now().toSec() - init_time) > data->timeout)
      {
        ROS_WARN_STREAM("Waypoint timeout");
        result_msg.final_status = cola2_msgs::WorldWaypointResult::TIMEOUT;
        waypoint_server_->setAborted(result_msg);
        break;
      }
    }
    // Sleep
    r.sleep();
  }
}

void Pilot::sectionServerCallback(const cola2_msgs::WorldSectionGoalConstPtr& data)
{
  // TODO: Avoid having a waypoint or a section controller running simultaneously

  // Conversion from actionlib goal to internal Section type
  control::Section section;
  section.initial_position.x = data->initial_position.x;
  section.initial_position.y = data->initial_position.y;
  section.initial_position.z = data->initial_position.z;
  section.final_position.x = data->final_position.x;
  section.final_position.y = data->final_position.y;
  section.final_position.z = data->final_position.z;
  section.altitude_mode = data->altitude_mode;
  section.tolerance.x = data->tolerance.x;
  section.tolerance.y = data->tolerance.y;
  section.tolerance.z = data->tolerance.z;

  // Main loop
  double init_time = ros::Time::now().toSec();
  ros::Rate r(10);  // 10Hz
  while (ros::ok())
  {
    // Declare some vars
    control::State controller_output;
    control::Feedback feedback;
    cola2_msgs::WorldSectionResult result_msg;
    control::PointsList points;

    // Run controller
    try
    {
      switch (data->controller_type)
      {
        case cola2_msgs::WorldSectionGoal::LOSCTE:
          ROS_DEBUG_STREAM("LOSCTE controller");
          los_cte_controller_->compute(current_state_, section, controller_output, feedback, points);
          break;
        default:
          throw std::runtime_error("Unknown controller");
      }
    }
    catch (std::exception& e)
    {
      // Check for failure
      ROS_ERROR_STREAM("Controller failure\n" << e.what());
      result_msg.final_status = cola2_msgs::WorldSectionResult::FAILURE;
      section_server_->setAborted(result_msg);
      break;
    }

    // Publishers
    publishControlCommands(controller_output, data->priority);
    publishFeedback(feedback, SECTION_MODE);
    publishMarker(section.final_position.x, section.final_position.y, section.final_position.z);
    publishMarkerSections(points);
    publishGoal(section.final_position.x, section.final_position.y, section.final_position.z);

    // Check for success
    if (feedback.success)
    {
      ROS_INFO_STREAM("Section success");
      result_msg.final_status = cola2_msgs::WorldSectionResult::SUCCESS;
      section_server_->setSucceeded(result_msg);
      break;
    }

    // Check for preempted. This happens upon user request (by preempting
    // or cancelling the goal, or when a new SectionGoal is received
    if (section_server_->isPreemptRequested())
    {
      ROS_WARN_STREAM("Section preempted");
      section_server_->setPreempted();
      break;
    }

    // Check for timeout
    if (data->timeout > 0.0)
    {
      if ((ros::Time::now().toSec() - init_time) > data->timeout)
      {
        ROS_WARN_STREAM("Section timeout");
        result_msg.final_status = cola2_msgs::WorldSectionResult::TIMEOUT;
        section_server_->setAborted(result_msg);
        break;
      }
    }

    // Sleep
    r.sleep();
  }
}

void Pilot::publishGoal(const double x, const double y, const double z)
{
  geometry_msgs::PointStamped goal;
  goal.header.frame_id = "ned";
  goal.header.stamp = ros::Time::now();
  goal.point.x = x;
  goal.point.y = y;
  goal.point.z = z;
  pub_goal_.publish(goal);
}

void Pilot::publishControlCommands(const control::State& controller_output, const unsigned int priority)
{
  // Get time
  ros::Time now = ros::Time::now();

  // Create ROS msgs for wwr
  cola2_msgs::WorldWaypointReq wwr;
  wwr.header.frame_id = cola2::rosutils::getNamespace() + "/base_link";
  wwr.header.stamp = now;
  wwr.goal.priority = priority;
  wwr.goal.requester = ros::this_node::getName() + "_pose_req";
  wwr.disable_axis.x = controller_output.pose.disable_axis.x;
  wwr.disable_axis.y = controller_output.pose.disable_axis.y;
  wwr.disable_axis.z = controller_output.pose.disable_axis.z;
  wwr.disable_axis.roll = controller_output.pose.disable_axis.roll;
  wwr.disable_axis.pitch = controller_output.pose.disable_axis.pitch;
  wwr.disable_axis.yaw = controller_output.pose.disable_axis.yaw;
  wwr.position.north = controller_output.pose.position.north;
  wwr.position.east = controller_output.pose.position.east;
  wwr.position.depth = controller_output.pose.position.depth;
  wwr.orientation.roll = controller_output.pose.orientation.roll;
  wwr.orientation.pitch = controller_output.pose.orientation.pitch;
  wwr.orientation.yaw = controller_output.pose.orientation.yaw;
  wwr.altitude_mode = controller_output.pose.altitude_mode;
  wwr.altitude = controller_output.pose.altitude;

  // Create ROS msgs for bvr
  cola2_msgs::BodyVelocityReq bvr;
  bvr.header.frame_id = cola2::rosutils::getNamespace() + "/base_link";
  bvr.header.stamp = now;
  bvr.goal.priority = priority;
  bvr.goal.requester = ros::this_node::getName() + "_velocity_req";
  bvr.disable_axis.x = controller_output.velocity.disable_axis.x;
  bvr.disable_axis.y = controller_output.velocity.disable_axis.y;
  bvr.disable_axis.z = controller_output.velocity.disable_axis.z;
  bvr.disable_axis.roll = controller_output.velocity.disable_axis.roll;
  bvr.disable_axis.pitch = controller_output.velocity.disable_axis.pitch;
  bvr.disable_axis.yaw = controller_output.velocity.disable_axis.yaw;
  bvr.twist.linear.x = controller_output.velocity.linear.x;
  bvr.twist.linear.y = controller_output.velocity.linear.y;
  bvr.twist.linear.z = controller_output.velocity.linear.z;
  bvr.twist.angular.x = controller_output.velocity.angular.x;
  bvr.twist.angular.y = controller_output.velocity.angular.y;
  bvr.twist.angular.z = controller_output.velocity.angular.z;

  // Publish output
  pub_wwr_.publish(wwr);
  pub_bvr_.publish(bvr);
}

void Pilot::publishFeedback(const control::Feedback& feedback, unsigned int mode = SECTION_MODE)
{
  // Conversion from internal feedback type to actionlib feedback
  switch (mode)
  {
    case SECTION_MODE:
    {
      cola2_msgs::WorldSectionFeedback msg;
      msg.desired_surge = feedback.desired_surge;
      msg.desired_depth = feedback.desired_depth;
      msg.desired_yaw = feedback.desired_yaw;
      msg.cross_track_error = feedback.cross_track_error;
      msg.depth_error = feedback.depth_error;
      msg.yaw_error = feedback.yaw_error;
      msg.distance_to_section_end = feedback.distance_to_end;
      section_server_->publishFeedback(msg);
      break;
    }
    case WAYPOINT_MODE:
    {
      cola2_msgs::WorldWaypointFeedback msg;
      // TODO: To be completed
      msg.distance_to_waypoint = feedback.distance_to_end;
      waypoint_server_->publishFeedback(msg);
      break;
    }
    default:
      ROS_WARN_STREAM("Error, invalid feedback message!\n");
  }
}

void Pilot::publishMarker(double north, double east, double depth)
{
  // Publish marker. Marker is published periodically so that RViz always
  // receives it, even if RViz is started after the ActionGoal arrives
  visualization_msgs::Marker marker;
  marker.header.frame_id = "ned";
  marker.header.stamp = ros::Time::now();
  marker.ns = ros::this_node::getName();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = north;
  marker.pose.position.y = east;
  marker.pose.position.z = depth;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(1.0);
  marker.frame_locked = false;
  pub_marker_.publish(marker);
}

void Pilot::publishMarkerSections(const control::PointsList points)
{
  // Create visualization marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "ned";
  marker.header.stamp = ros::Time::now();
  marker.ns = ros::this_node::getName();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  // Add points to it
  for (const auto &i : points.points_list) {
    geometry_msgs::Point p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    marker.points.push_back(p);
  }

  marker.scale.x = 0.35;
  marker.color.r = 0.8;
  marker.color.g = 0.8;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(1.0);
  marker.frame_locked = false;
  pub_marker_.publish(marker);
}

void Pilot::getConfig()
{
  // Load config from param server
  // LOS-CTE controller
  // clang-format off
  cola2::rosutils::getParam("~los_cte_delta", config_.los_cte_config.delta, 5.0);
  cola2::rosutils::getParam("~los_cte_distance_to_max_velocity", config_.los_cte_config.distance_to_max_velocity,                           5.0);
  cola2::rosutils::getParam("~los_cte_max_surge_velocity", config_.los_cte_config.max_surge_velocity, 0.5);
  cola2::rosutils::getParam("~los_cte_min_surge_velocity", config_.los_cte_config.min_surge_velocity, 0.2);
  cola2::rosutils::getParam("~los_cte_min_velocity_ratio", config_.los_cte_config.min_velocity_ratio, 0.1);
  cola2::rosutils::getParam("~los_cte_heave_in_3D", config_.los_cte_config.heave_in_3D, false);

  // GOTO controller
  cola2::rosutils::getParam("~goto_max_angle_error", config_.goto_config.max_angle_error, 0.3);
  cola2::rosutils::getParam("~goto_max_surge", config_.goto_config.max_surge, 0.5);
  cola2::rosutils::getParam("~goto_surge_proportional_gain", config_.goto_config.surge_proportional_gain, 0.25);

  // ANCHOR controller
  cola2::rosutils::getParam("~anchor_kp", config_.anchor_config.kp, 0.1);
  cola2::rosutils::getParam("~anchor_radius", config_.anchor_config.radius, 1.0);
  cola2::rosutils::getParam("~anchor_min_surge", config_.anchor_config.min_surge, -0.1);
  cola2::rosutils::getParam("~anchor_max_surge", config_.anchor_config.max_surge, 0.3);
  cola2::rosutils::getParam("~anchor_safety_distance", config_.anchor_config.safety_distance, 50.0);
  cola2::rosutils::getParam("~anchor_max_angle_error", config_.anchor_config.max_angle_error, 0.5);
  // clang-format on
}

/*
void Pilot::setParams(cola2_control::PilotConfig& config, uint32_t level)
{
  ROS_INFO_STREAM("New parameters received!\n");
  config_.los_cte_config.delta = config.los_cte_delta;
  config_.los_cte_config.distance_to_max_velocity = config.los_cte_distance_to_max_velocity;
  config_.los_cte_config.max_surge_velocity = config.los_cte_max_surge_velocity;
  config_.los_cte_config.min_surge_velocity = config.los_cte_min_surge_velocity;
  config_.los_cte_config.min_velocity_ratio = config.los_cte_min_velocity_ratio;
  config_.los_cte_config.heave_in_3D = config.los_cte_heave_in_3D;
  _los_cte_controller->setConfig(_config.los_cte_config);

  config_.goto_config.max_angle_error = config.goto_max_angle_error;
  config_.goto_config.max_surge = config.goto_max_surge;
  config_.goto_config.surge_proportional_gain = config.goto_surge_proportional_gain;
  _goto_controller->setConfig(_config.goto_config);
}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pilot_new");
  Pilot pilot;
  return 0;
}
