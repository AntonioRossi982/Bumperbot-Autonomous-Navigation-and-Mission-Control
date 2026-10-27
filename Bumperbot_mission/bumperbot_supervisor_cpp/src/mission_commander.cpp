#include "bumperbot_supervisor_cpp/mission_commander.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

// namespace bumperbot_supervisor_cpp
namespace bumperbot_supervisor_cpp
{

MissionCommander::MissionCommander(const rclcpp::NodeOptions & options) : Node("mission_commander", 
                                                                          rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  frame_id_ = this->declare_parameter<std::string>("goal_frame_id", "map");
  max_retries_ = this->declare_parameter<int>("max_retries", 3);

  loadLocations();
  if (locations_.empty()) {
    RCLCPP_WARN(get_logger(), "No locations found under the \"locations.*\" parameters. " "Check the YAML file passed to the node.");
  }

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  status_pub_ = this->create_publisher<std_msgs::msg::String>("mission_status", 10);
  command_sub_ = this->create_subscription<std_msgs::msg::String>("mission_command", 10, std::bind(&MissionCommander::commandCallback, this, _1));

  std::ostringstream names;
  for (const auto & [name, loc] : locations_) {
    names << name << " ";
  }
  RCLCPP_INFO(get_logger(), "Mission Commander ready. Known locations: %s", names.str().c_str());
}

void MissionCommander::loadLocations()
{
  // Iterate over all parameters under the "locations" prefix (depth 10
  // to include "locations.<name>.<field>") and group them by location.
  auto result = this->list_parameters({"locations"}, 10);

  for (const auto & full_name : result.names) {
    if (full_name.rfind("locations.", 0) != 0) {
      continue;
    }
    std::string rest = full_name.substr(std::string("locations.").size());
    auto dot = rest.find('.');
    if (dot == std::string::npos) {
      continue;
    }
    std::string loc_name = rest.substr(0, dot);
    std::string field = rest.substr(dot + 1);

    double value = this->get_parameter(full_name).as_double();
    if (field == "x") {
      locations_[loc_name].x = value;
    } else if (field == "y") {
      locations_[loc_name].y = value;
    } else if (field == "theta") {
      locations_[loc_name].theta = value;
    }
  }
}

void MissionCommander::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string text = msg->data;
  auto not_space = [](unsigned char c) {return !std::isspace(c);};
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  if (text.empty()) {
    return;
  }

  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "cancel") {
    cancelGoal();
    return;
  }

  if (lower.rfind("go_to", 0) != 0) {
    RCLCPP_WARN(get_logger(), "Unrecognized command: \"%s\"", text.c_str());
    return;
  }

  auto space = text.find(' ');
  if (space == std::string::npos) {
    RCLCPP_WARN(get_logger(), "\"go_to\" command without a location: use \"go_to <location>\"");
    return;
  }

  std::string location_name = text.substr(space + 1);
  location_name.erase(
    location_name.begin(),
    std::find_if(location_name.begin(), location_name.end(), not_space));

  startMission(location_name);
}

void MissionCommander::startMission(const std::string & location_name)
{
  if (locations_.find(location_name) == locations_.end()) {
    RCLCPP_ERROR(get_logger(), "Unknown location: \"%s\"", location_name.c_str());
    publishStatus("ERROR unknown_location " + location_name);
    return;
  }

  if (goal_active_) {
    RCLCPP_WARN(get_logger(), "A goal is already active: ignoring the new request. " "Send \"cancel\" before submitting another goal.");
    publishStatus("REJECTED goal_already_active");
    return;
  }

  current_location_ = location_name;
  retry_count_ = 0;
  sendGoal(location_name);
}

void MissionCommander::sendGoal(const std::string & location_name)
{
  if (!action_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(get_logger(), "Action server \"navigate_to_pose\" is unavailable");
    publishStatus("ERROR action_server_unavailable");
    goal_active_ = false;
    return;
  }

  const auto & loc = locations_.at(location_name);
  NavigateToPose::Goal goal_msg;
  goal_msg.pose = buildPose(loc);

  goal_active_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Sending goal to \"%s\" (attempt %d/%d): x=%.2f y=%.2f theta=%.2f",
    location_name.c_str(), retry_count_ + 1, max_retries_ + 1, loc.x, loc.y, loc.theta);
  publishStatus("SENDING " + location_name);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MissionCommander::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&MissionCommander::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&MissionCommander::resultCallback, this, _1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
}

geometry_msgs::msg::PoseStamped MissionCommander::buildPose(const Location & location) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id_;
  pose.header.stamp = this->get_clock()->now();
  pose.pose.position.x = location.x;
  pose.pose.position.y = location.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, location.theta);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

void MissionCommander::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Goal rejected by the server for \"%s\"", current_location_.c_str());
    publishStatus("REJECTED " + current_location_);
    handleFailure();
    return;
  }

  RCLCPP_INFO(get_logger(), "Goal accepted");
  goal_handle_ = goal_handle;
  publishStatus("ACCEPTED " + current_location_);
}

void MissionCommander::feedbackCallback(
  GoalHandleNav::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
}

void MissionCommander::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  goal_active_ = false;
  goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal reached: \"%s\"", current_location_.c_str());
      publishStatus("SUCCEEDED " + current_location_);
      resetMissionState();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "Goal canceled: \"%s\"", current_location_.c_str());
      publishStatus("CANCELED " + current_location_);
      resetMissionState();
      break;
    default:
      RCLCPP_WARN(get_logger(), "Goal failed for \"%s\"", current_location_.c_str());
      handleFailure();
      break;
  }
}

void MissionCommander::handleFailure()
{
  retry_count_++;
  if (retry_count_ > max_retries_) {
    RCLCPP_ERROR(
      get_logger(),
      "Maximum number of attempts exceeded for \"%s\". Mission aborted.",
      current_location_.c_str());
    publishStatus("FAILED " + current_location_);
    resetMissionState();
    return;
  }

  RCLCPP_INFO(get_logger(), "Retrying goal submission (%d/%d)", retry_count_, max_retries_);
  sendGoal(current_location_);
}

void MissionCommander::cancelGoal()
{
  if (!goal_active_ || !goal_handle_) {
    RCLCPP_INFO(get_logger(), "No active goal to cancel");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Sending cancellation request for \"%s\"", current_location_.c_str());
  action_client_->async_cancel_goal(goal_handle_);
}

void MissionCommander::resetMissionState()
{
  retry_count_ = 0;
  current_location_.clear();
}

void MissionCommander::publishStatus(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
}

}  
