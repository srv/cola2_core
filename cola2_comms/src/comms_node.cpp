/*
 * Copyright (c) 2019 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>This node hadles the data interchanged in acoustic communications.<@@*/

#include <cola2_lib/rosutils/diagnostic_helper.h>
#include <cola2_lib/rosutils/param_loader.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/Recovery.h>
#include <cola2_msgs/RecoveryAction.h>
#include <cola2_msgs/SafetySupervisorStatus.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <cstdint>
#include <ctime>
#include "cppystruct/cppystruct.h"

// from evologics_lib.custom_commands import CustomCommands

const auto FMT = PY_STRING("<1s1d2d2f1h");  //!< format of the message transmitted from the vehicle {id, time, lat, lon,
                                            //!< depth, yaw/accuracy, error_code/command }

/**
 * \brief Commands that are not part of cola2_msgs::RecoveryAction and are directly executed by cola2_comms.
 */
enum CustomCommands
{
  START_MISSION = 10  //!< start default mission
};

/**
 * \class That hadles the interchange of data between the surface USBL and the vehicle where this node is running.
 */
class CommsNode
{
private:
  /**
   * \brief Configuration loaded from ROS param server.
   */
  struct Config
  {
    double custom_max_time;       //!< max time since last custom input reception to still send it
    bool usbl_safe_always_on;     //!< check that vehicle continuously receives modem messages
    double send_to_modem_period;  //!< time between messages sent to modem
    int max_message_size;         //!< max message size (defined by modem)
    std::string identifier;       //!< letter that identifies the sender
  } config_;                      //!< configuration params

  // NodeHandle
  ros::NodeHandle nh_;  //!< node handler
  // Publishers
  ros::Publisher pub_custom_output_;  //!< binarized user specific output
  ros::Publisher pub_to_modem_;       //!< message to be sent by the modem
  ros::Publisher pub_usbl_;           //!< usbl update received from modem
  // Subscribers
  ros::Subscriber sub_from_modem_;     //!< data received by the modem
  ros::Subscriber sub_safety_status_;  //!< current safety status to get error code
  ros::Subscriber sub_navigation_;     //!< vehicle navigation
  ros::Subscriber sub_custom_input_;   //!< binarized user specific input
  // ServiceClients
  ros::ServiceClient srv_recovery_;       //!< service to call recovery actions
  ros::ServiceClient srv_start_mission_;  //!< service to call mission start
  // ServiceServers
  ros::ServiceServer srv_reload_params_;  //!< reload config of this node
  // Timer
  ros::Timer timer_;  //!< timer to send messages to modem

  // Diagnostics
  cola2::rosutils::DiagnosticHelper diag_help_;  //!< to create diagnostics

  // Basic message size
  const int fmt_size_ = pystruct::calcsize(FMT);  //!< size of the defined encoded message

  // Saves from callbacks
  std::string last_message_ = "";                         //!< last received message
  std::string custom_ = "";                               //!< custom extra message received
  double custom_time_ = ros::Time::now().toSec();         //!< time of last custom message
  int16_t error_code_ = 0;                                //!< error code received
  cola2_msgs::NavSts navigation_ = cola2_msgs::NavSts();  //!< navigation received
  bool init_navigation_ = false;                          //!< navigation arrived
  double modem_age_ = 0.0;                                //!< time since last message from modem

  // Functions
  bool loadParams();
  bool callRecoveryService(const uint16_t &code);
  // Server to reload params
  bool srvReloadParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  // Callbacks
  void cbkFromModem(const std_msgs::String &msg);
  void cbkCustomInput(const std_msgs::String &msg);
  void cbkSafetyStatus(const cola2_msgs::SafetySupervisorStatus &msg);
  void cbkNavigation(const cola2_msgs::NavSts &msg);
  // Timer callback
  void sendMessage(const ros::TimerEvent &event);

public:
  // Constructor
  CommsNode();
  // Destructor
  ~CommsNode();
};

CommsNode::CommsNode() : nh_("~"), diag_help_(nh_, cola2::rosutils::getUnresolvedNodeName(), "software")
{
  // Load params from param server
  loadParams();

  // USBL data timeout
  if (config_.usbl_safe_always_on)
  {
    modem_age_ = ros::Time::now().toSec();
  }

  // Publishers
  // clang-format off
  pub_to_modem_ = nh_.advertise<std_msgs::String>("to_modem", 1);
  pub_custom_output_ = nh_.advertise<std_msgs::String>("custom_input", 1);
  pub_usbl_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(cola2::rosutils::getNamespace() + "/navigator/usbl", 1);
  // clang-format on

  // Service clients
  std::string sname_recovery = cola2::rosutils::getNamespace() + "/recovery_actions/recover";
  srv_recovery_ = nh_.serviceClient<cola2_msgs::Recovery>(sname_recovery);
  std::string sname_start = cola2::rosutils::getNamespace() + "/captain/enable_default_mission_non_block";
  srv_start_mission_ = nh_.serviceClient<std_srvs::Empty>(sname_start);
  while (ros::ok())
  {
    if ((srv_recovery_.waitForExistence(ros::Duration(5.0))) && srv_start_mission_.waitForExistence(ros::Duration(5.0)))
    {
      break;
    }
    ROS_INFO_STREAM("Waiting for client to services " << sname_recovery << " and " << sname_start);
  }

  // Init services
  srv_reload_params_ = nh_.advertiseService("reload_params", &CommsNode::srvReloadParams, this);

  // Subscribers
  // clang-format off
  sub_from_modem_ = nh_.subscribe("from_modem", 1, &CommsNode::cbkFromModem, this);
  sub_custom_input_ = nh_.subscribe("custom_input", 1, &CommsNode::cbkCustomInput, this);
  sub_safety_status_ = nh_.subscribe(cola2::rosutils::getNamespace() + "/safety_supervisor/status", 1, &CommsNode::cbkSafetyStatus, this);
  sub_navigation_ = nh_.subscribe(cola2::rosutils::getNamespace() + "/navigator/navigation", 1, &CommsNode::cbkNavigation, this);
  // clang-format on

  // Timer
  timer_ = nh_.createTimer(ros::Duration(config_.send_to_modem_period), &CommsNode::sendMessage, this);
}

CommsNode::~CommsNode()
{
}

bool CommsNode::loadParams()
{
  // Load from ROS param server
  // clang-format off
  config_.custom_max_time = cola2::rosutils::getParam("comms/custom_max_time", config_.custom_max_time, 1.0);
  config_.usbl_safe_always_on = cola2::rosutils::getParam("comms/usbl_safe_always_on", config_.usbl_safe_always_on);
  config_.send_to_modem_period = cola2::rosutils::getParam("comms/send_to_modem_period", config_.send_to_modem_period, 0.5);
  config_.max_message_size = cola2::rosutils::getParam("comms/max_message_size", config_.max_message_size);
  config_.identifier = cola2::rosutils::getParam("comms/identifier", config_.identifier);
  // clang-format on

  // Show loaded config
  ROS_INFO("Loaded config from param server");
  ROS_INFO("===============================");
  ROS_INFO("     custom_max_time: %.2f", config_.custom_max_time);
  ROS_INFO(" usbl_safe_always_on: %d  ", config_.usbl_safe_always_on);
  ROS_INFO("send_to_modem_period: %.2f", config_.send_to_modem_period);
  ROS_INFO("    max_message_size: %d  ", config_.max_message_size);
  ROS_INFO("          identifier: %s  ", config_.identifier.c_str());

  // Checks
  assert(config_.max_message_size >= fmt_size_);  // enough message space for basic message
  assert(config_.identifier.size() == 1);         // single char identifier
}

bool CommsNode::srvReloadParams(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  return loadParams();
}

void CommsNode::cbkFromModem(const std_msgs::String &msg)
{
  // Is not exactly the previous message
  if (last_message_.compare(msg.data) == 0)
  {
    return;
  }
  last_message_ = msg.data;
  modem_age_ = ros::Time::now().toSec();

  // Unpack basic message
  std::string basic = msg.data.substr(0, fmt_size_);
  auto [id, tim, lat, lon, depth, accuracy, command] = pystruct::unpack(FMT, basic);

  // If it comes from usbl
  if (id.compare(std::string("U")) == 0)
  {
    // If it is a valid message
    if ((tim != 0.0) && (lat != 0.0))
    {
      // Construct USBL update
      geometry_msgs::PoseWithCovarianceStamped usbl;
      usbl.header.frame_id = "world_ned";
      usbl.header.stamp = ros::Time(tim);
      usbl.pose.pose.position.x = lat;
      usbl.pose.pose.position.y = lon;
      usbl.pose.pose.position.z = depth;
      usbl.pose.covariance[0] = accuracy * accuracy;
      usbl.pose.covariance[8] = usbl.pose.covariance[0];
      usbl.pose.covariance[16] = usbl.pose.covariance[0];
      // Publish
      pub_usbl_.publish(usbl);
    }
    // Check the command
    switch (command)
    {
      case cola2_msgs::RecoveryAction::NONE:
        break;
      case cola2_msgs::RecoveryAction::INFORMATIVE:
        break;
      case cola2_msgs::RecoveryAction::ABORT_MISSION:
        ROS_WARN("abort mission");
        callRecoveryService(cola2_msgs::RecoveryAction::ABORT_MISSION);
        break;
      case cola2_msgs::RecoveryAction::ABORT_AND_SURFACE:
        ROS_WARN("abort and surface");
        callRecoveryService(cola2_msgs::RecoveryAction::ABORT_AND_SURFACE);
        break;
      case cola2_msgs::RecoveryAction::EMERGENCY_SURFACE:
        ROS_WARN("emergency surface");
        callRecoveryService(cola2_msgs::RecoveryAction::EMERGENCY_SURFACE);
        break;
      case cola2_msgs::RecoveryAction::STOP_THRUSTERS:
        ROS_WARN("stop thrusters");
        callRecoveryService(cola2_msgs::RecoveryAction::STOP_THRUSTERS);
        break;
      case CustomCommands::START_MISSION:
        ROS_WARN("start mission");
        std_srvs::EmptyRequest req;
        std_srvs::EmptyResponse res;
        srv_start_mission_.call(req, res);
        break;
    }
  }

  // Rest of message is custom message
  if (msg.data.size() > fmt_size_)
  {
    std_msgs::String custom;
    custom.data = msg.data.substr(fmt_size_, msg.data.size() - fmt_size_);
    pub_custom_output_.publish(custom);
  }
}

bool CommsNode::callRecoveryService(const uint16_t &code)
{
  // Instead of directly calling a recovery action, sends a diagnostic message
  // to be interpreted by the safety_supervisor
  diag_help_.add("modem_recovery_action", std::to_string(code));
  diag_help_.setLevel(diagnostic_msgs::DiagnosticStatus::WARN);

  // Call the recovery action
  cola2_msgs::RecoveryAction ra;
  ra.header.stamp = ros::Time::now();
  ra.error_level = code;
  cola2_msgs::RecoveryRequest req;
  req.requested_action = ra;
  cola2_msgs::RecoveryResponse res;
  if (srv_recovery_.call(req, res))
  {
    return true;
  }
  ROS_ERROR("error calling recovery service");
  return false;
}

void CommsNode::cbkCustomInput(const std_msgs::String &msg)
{
  custom_ = msg.data;
  custom_time_ = ros::Time::now().toSec();
}
void CommsNode::cbkSafetyStatus(const cola2_msgs::SafetySupervisorStatus &msg)
{
  error_code_ = msg.error_code;
}
void CommsNode::cbkNavigation(const cola2_msgs::NavSts &msg)
{
  navigation_ = msg;
  init_navigation_ = true;
  // Avoid modem errors when vehicle is on surface
  if (navigation_.position.depth < 1.0)
  {
    modem_age_ = 1.0;
  }
}

void CommsNode::sendMessage(const ros::TimerEvent &event)
{
  // Navigation available
  if (!init_navigation_)
  {
    return;
  }
  // Create message
  auto packed = pystruct::pack(FMT, config_.identifier, navigation_.header.stamp.toSec(),
                               navigation_.global_position.latitude, navigation_.global_position.longitude,
                               navigation_.position.depth, navigation_.orientation.yaw, error_code_);
  std_msgs::String msg;
  msg.data = std::string(std::begin(packed), std::end(packed));
  // Add custom data if not too old
  if ((event.current_real.toSec() - custom_time_) < config_.custom_max_time)
  {
    // Check that it fits
    if ((fmt_size_ + custom_.size() <= config_.max_message_size))
    {
      msg.data += custom_;
    }
    else
    {
      ROS_WARN("ignoring custom message of size %zu because it doesn't fit", custom_.size());
    }
  }
  // Send message
  pub_to_modem_.publish(msg);

  // Check that we are receiving
  if (modem_age_ > 0)
  {
    const double tim = ros::Time::now().toSec();
    diag_help_.add("last_modem_data", std::to_string(tim - modem_age_));
    diag_help_.setLevel(diagnostic_msgs::DiagnosticStatus::OK);
  }
}

int main(int argc, char **argv)
{
  // Init
  ros::init(argc, argv, "comms");
  CommsNode node;
  ros::spin();  // spin until architecture stops
  return EXIT_SUCCESS;
}