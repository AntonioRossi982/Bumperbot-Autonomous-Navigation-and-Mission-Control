#include "bumperbot_supervisor_cpp/mission_commander.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_supervisor_cpp::MissionCommander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
