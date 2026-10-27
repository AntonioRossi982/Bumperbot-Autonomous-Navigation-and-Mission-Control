#ifndef BUMPERBOT_SUPERVISOR_CPP__MISSION_COMMANDER_HPP_
#define BUMPERBOT_SUPERVISOR_CPP__MISSION_COMMANDER_HPP_

#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"


// namespace bumperbot_supervisor_cpp
namespace bumperbot_supervisor_cpp
{

struct Location
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/// Receives symbolic commands ("go_to room_a", "cancel") and translates them
/// into NavigateToPose goals sent to Nav2 through an action client. Handles
/// one goal at a time, including rejection, feedback, results, cancellation,
/// and limited retries. Locations are loaded from parameters (YAML), not hardcoded.
class MissionCommander : public rclcpp::Node
{

public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MissionCommander(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadLocations();
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void startMission(const std::string & location_name);
  void sendGoal(const std::string & location_name);
  geometry_msgs::msg::PoseStamped buildPose(const Location & location) const;

  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleNav::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);
  void handleFailure();
  void cancelGoal();
  void resetMissionState();
  void publishStatus(const std::string & text);

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::map<std::string, Location> locations_;
  GoalHandleNav::SharedPtr goal_handle_;

  std::string frame_id_;
  int max_retries_{3};
  int retry_count_{0};
  bool goal_active_{false};
  std::string current_location_;
};

}

#endif  // BUMPERBOT_SUPERVISOR_CPP__MISSION_COMMANDER_HPP_