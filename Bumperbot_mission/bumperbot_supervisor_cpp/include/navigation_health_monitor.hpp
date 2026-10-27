#ifndef BUMPERBOT_SUPERVISOR_CPP__NAVIGATION_HEALTH_MONITOR_HPP_
#define BUMPERBOT_SUPERVISOR_CPP__NAVIGATION_HEALTH_MONITOR_HPP_

#include <array>
#include <deque>
#include <memory>
#include <set>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// namespace bumperbot_supervisor_cpp
namespace bumperbot_supervisor_cpp
{

/// Tracks the arrival rate of a topic over a sliding time window.
class RateTracker
{
public:
  RateTracker()
  : RateTracker(5.0) {}

  explicit RateTracker(double window_sec)
  : window_sec_(window_sec) {}

  void tick(double now_sec)
  {
    stamps_.push_back(now_sec);
    prune(now_sec);
  }

  double rateHz(double now_sec)
  {
    prune(now_sec);
    if (stamps_.size() < 2) {
      return 0.0;
    }
    double span = stamps_.back() - stamps_.front();
    if (span <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(stamps_.size() - 1) / span;
  }

  bool hasData() const {return !stamps_.empty();}
  double lastStamp() const {return stamps_.back();}

private:
  void prune(double now_sec)
  {
    while (!stamps_.empty() && now_sec - stamps_.front() > window_sec_) {
      stamps_.pop_front();
    }
  }

  double window_sec_;
  std::deque<double> stamps_;
};

/// Monitors the health of the navigation stack: AMCL covariance, the
/// map->odom->base_footprint TF chain, the publication rates of /scan and
/// /odom, and the number of aborted NavigateToPose goals. Periodically
/// publishes a diagnostic_msgs/DiagnosticArray.
class NavigationHealthMonitor : public rclcpp::Node
{
public:
  explicit NavigationHealthMonitor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct CheckResult
  {
    uint8_t level;
    std::string message;
  };

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void nav2StatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  double nowSec() const;

  CheckResult checkLocalization(double & pos_std, double & yaw_std);
  CheckResult checkTfChain();
  CheckResult checkRate(
    RateTracker & tracker, double min_rate, const std::string & label, double & rate_out);
  CheckResult checkAbortedGoals();

  void publishDiagnostics();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav2_status_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string hardware_id_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double pos_cov_warn_{0.05};
  double pos_cov_error_{0.25};
  double yaw_cov_warn_{0.05};
  double yaw_cov_error_{0.25};
  double min_scan_rate_{5.0};
  double min_odom_rate_{10.0};
  double rate_window_{5.0};
  double stale_timeout_{2.0};
  int max_aborted_goals_warn_{3};

  // State
  bool amcl_received_{false};
  double latest_pos_std_{0.0};
  double latest_yaw_std_{0.0};
  bool was_localization_ok_{true};

  RateTracker scan_rate_;
  RateTracker odom_rate_;

  std::set<std::array<uint8_t, 16>> aborted_goal_ids_;
  int aborted_count_{0};
};

} 

#endif  // BUMPERBOT_SUPERVISOR_CPP__NAVIGATION_HEALTH_MONITOR_HPP_