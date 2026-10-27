#include "bumperbot_supervisor_cpp/navigation_health_monitor.hpp"

#include <algorithm>
#include <cmath>

#include "action_msgs/msg/goal_status.hpp"
#include "tf2/time.h"
#include "unique_identifier_msgs/msg/uuid.hpp"

using std::placeholders::_1;

// namespace bumperbot_health_cpp
namespace bumperbot_supervisor_cpp
{

namespace
{
std::array<uint8_t, 16> toArray(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::array<uint8_t, 16> out{};
  std::copy(uuid.uuid.begin(), uuid.uuid.end(), out.begin());
  return out;
}
} 

NavigationHealthMonitor::NavigationHealthMonitor(const rclcpp::NodeOptions & options) : Node("navigation_health_monitor", 
                                      rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  hardware_id_ = this->declare_parameter<std::string>("hardware_id", "bumperbot");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");

  pos_cov_warn_ = this->declare_parameter<double>("position_covariance_warn", 0.05);
  pos_cov_error_ = this->declare_parameter<double>("position_covariance_error", 0.25);
  yaw_cov_warn_ = this->declare_parameter<double>("yaw_covariance_warn", 0.05);
  yaw_cov_error_ = this->declare_parameter<double>("yaw_covariance_error", 0.25);

  min_scan_rate_ = this->declare_parameter<double>("min_scan_rate", 5.0);
  min_odom_rate_ = this->declare_parameter<double>("min_odom_rate", 10.0);
  rate_window_ = this->declare_parameter<double>("rate_window", 5.0);
  stale_timeout_ = this->declare_parameter<double>("stale_timeout", 2.0);

  max_aborted_goals_warn_ = this->declare_parameter<int>("max_aborted_goals_warn", 3);

  double diagnostics_period = this->declare_parameter<double>("diagnostics_period", 1.0);

  scan_rate_ = RateTracker(rate_window_);
  odom_rate_ = RateTracker(rate_window_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10, std::bind(&NavigationHealthMonitor::amclCallback, this, _1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&NavigationHealthMonitor::scanCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&NavigationHealthMonitor::odomCallback, this, _1));

  nav2_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "/navigate_to_pose/_action/status", 10,
    std::bind(&NavigationHealthMonitor::nav2StatusCallback, this, _1));

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(diagnostics_period),
    std::bind(&NavigationHealthMonitor::publishDiagnostics, this));

  RCLCPP_INFO(get_logger(), "Navigation Health Monitor ready.");
}

double NavigationHealthMonitor::nowSec() const
{
  return this->get_clock()->now().seconds();
}

void NavigationHealthMonitor::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto & cov = msg->pose.covariance;
  double cov_xx = cov[0];
  double cov_yy = cov[7];
  double cov_yaw = cov[35];
  latest_pos_std_ = std::sqrt(std::max(cov_xx, 0.0) + std::max(cov_yy, 0.0));
  latest_yaw_std_ = std::sqrt(std::max(cov_yaw, 0.0));
  amcl_received_ = true;
}

void NavigationHealthMonitor::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/)
{
  scan_rate_.tick(nowSec());
}

void NavigationHealthMonitor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
{
  odom_rate_.tick(nowSec());
}

void NavigationHealthMonitor::nav2StatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  for (const auto & entry : msg->status_list) {
    if (entry.status != action_msgs::msg::GoalStatus::STATUS_ABORTED) {
      continue;
    }
    auto id = toArray(entry.goal_info.goal_id.uuid);
    if (aborted_goal_ids_.count(id) > 0) {
      continue;
    }
    aborted_goal_ids_.insert(id);
    aborted_count_++;
    RCLCPP_WARN(get_logger(), "NavigateToPose goal aborted (total: %d)", aborted_count_);
  }
}

NavigationHealthMonitor::CheckResult NavigationHealthMonitor::checkLocalization(
  double & pos_std, double & yaw_std)
{
  if (!amcl_received_) {
    pos_std = 0.0;
    yaw_std = 0.0;
    return {diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "No messages received on /amcl_pose"};
  }

  pos_std = latest_pos_std_;
  yaw_std = latest_yaw_std_;

  // Covariance in the error range
  if (pos_std >= pos_cov_error_ || yaw_std >= yaw_cov_error_) {
    return {diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Localization severely degraded (very high covariance)"};
  }
  // Covariance in the warning range 
  if (pos_std >= pos_cov_warn_ || pos_std < pos_cov_error_ || yaw_std >= yaw_cov_warn_ || yaw_std < yaw_cov_error_) {
    return {diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Localization degraded (high covariance)"};
  }
  return {diagnostic_msgs::msg::DiagnosticStatus::OK, "Localization operating normally"};
}

NavigationHealthMonitor::CheckResult NavigationHealthMonitor::checkTfChain()
{
  // Zero timeout (default): non-blocking read from the TF buffer, populated
  // in the background by the TransformListener. Avoids deadlocks when the node
  // runs on a single-threaded executor.
  bool odom_ok = tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero);
  bool map_ok = tf_buffer_->canTransform(map_frame_, odom_frame_, tf2::TimePointZero);

  if (odom_ok && map_ok) {
    return {diagnostic_msgs::msg::DiagnosticStatus::OK,
      "TF chain " + map_frame_ + "->" + odom_frame_ + "->" + base_frame_ + " is available"};
  }

  std::string missing;
  if (!map_ok) {
    missing += map_frame_ + "->" + odom_frame_ + " ";
  }
  if (!odom_ok) {
    missing += odom_frame_ + "->" + base_frame_;
  }
  return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Missing transforms: " + missing};
}

NavigationHealthMonitor::CheckResult NavigationHealthMonitor::checkRate(
  RateTracker & tracker, double min_rate, const std::string & label, double & rate_out)
{
  if (!tracker.hasData()) {
    rate_out = 0.0;
    return {diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "No messages received on " + label};
  }

  double now = nowSec();
  double age = now - tracker.lastStamp();
  rate_out = tracker.rateHz(now);

  if (age > stale_timeout_) {
    return {diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      label + ": no data for " + std::to_string(age) + " s"};
  }
  if (rate_out < min_rate) {
    return {diagnostic_msgs::msg::DiagnosticStatus::WARN,
      label + ": low publication rate (" + std::to_string(rate_out) + " Hz)"};
  }
  return {diagnostic_msgs::msg::DiagnosticStatus::OK,
    label + ": " + std::to_string(rate_out) + " Hz"};
}

NavigationHealthMonitor::CheckResult NavigationHealthMonitor::checkAbortedGoals()
{
  if (aborted_count_ >= max_aborted_goals_warn_) {
    return {diagnostic_msgs::msg::DiagnosticStatus::WARN,
      std::to_string(aborted_count_) + " aborted goals (threshold: " +
      std::to_string(max_aborted_goals_warn_) + ")"};
  }
  return {diagnostic_msgs::msg::DiagnosticStatus::OK,
    std::to_string(aborted_count_) + " aborted goals"};
}

void NavigationHealthMonitor::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->get_clock()->now();

  double pos_std = 0.0;
  double yaw_std = 0.0;
  auto loc = checkLocalization(pos_std, yaw_std);
  diagnostic_msgs::msg::DiagnosticStatus loc_status;
  loc_status.name = "navigation_health_monitor: AMCL localization";
  loc_status.hardware_id = hardware_id_;
  loc_status.level = loc.level;
  loc_status.message = loc.message;
  diagnostic_msgs::msg::KeyValue pos_kv;
  pos_kv.key = "position_std_dev_m";
  pos_kv.value = std::to_string(pos_std);
  diagnostic_msgs::msg::KeyValue yaw_kv;
  yaw_kv.key = "yaw_std_dev_rad";
  yaw_kv.value = std::to_string(yaw_std);
  loc_status.values = {pos_kv, yaw_kv};
  array.status.push_back(loc_status);

  // Localization degradation warning: only "edge-triggered", when
  // the status worsens compared to the previous cycle.
  bool localization_ok_now = loc.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (was_localization_ok_ && !localization_ok_now) {
    RCLCPP_WARN(get_logger(), "Localization degraded: %s", loc.message.c_str());
  }
  was_localization_ok_ = localization_ok_now;

  auto tf_result = checkTfChain();
  diagnostic_msgs::msg::DiagnosticStatus tf_status;
  tf_status.name = "navigation_health_monitor: TF chain";
  tf_status.hardware_id = hardware_id_;
  tf_status.level = tf_result.level;
  tf_status.message = tf_result.message;
  array.status.push_back(tf_status);

  double scan_rate = 0.0;
  auto scan_result = checkRate(scan_rate_, min_scan_rate_, "/scan", scan_rate);
  diagnostic_msgs::msg::DiagnosticStatus scan_status;
  scan_status.name = "navigation_health_monitor: Laser scan rate";
  scan_status.hardware_id = hardware_id_;
  scan_status.level = scan_result.level;
  scan_status.message = scan_result.message;
  diagnostic_msgs::msg::KeyValue scan_kv;
  scan_kv.key = "rate_hz";
  scan_kv.value = std::to_string(scan_rate);
  scan_status.values = {scan_kv};
  array.status.push_back(scan_status);

  double odom_rate = 0.0;
  auto odom_result = checkRate(odom_rate_, min_odom_rate_, "/odom", odom_rate);
  diagnostic_msgs::msg::DiagnosticStatus odom_status;
  odom_status.name = "navigation_health_monitor: Odometry rate";
  odom_status.hardware_id = hardware_id_;
  odom_status.level = odom_result.level;
  odom_status.message = odom_result.message;
  diagnostic_msgs::msg::KeyValue odom_kv;
  odom_kv.key = "rate_hz";
  odom_kv.value = std::to_string(odom_rate);
  odom_status.values = {odom_kv};
  array.status.push_back(odom_status);

  auto aborts_result = checkAbortedGoals();
  diagnostic_msgs::msg::DiagnosticStatus aborts_status;
  aborts_status.name = "navigation_health_monitor: Navigation aborts";
  aborts_status.hardware_id = hardware_id_;
  aborts_status.level = aborts_result.level;
  aborts_status.message = aborts_result.message;
  diagnostic_msgs::msg::KeyValue aborts_kv;
  aborts_kv.key = "aborted_goals_total";
  aborts_kv.value = std::to_string(aborted_count_);
  aborts_status.values = {aborts_kv};
  array.status.push_back(aborts_status);

  diag_pub_->publish(array);
}

}  
