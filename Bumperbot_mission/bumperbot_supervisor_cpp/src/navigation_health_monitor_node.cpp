#include "bumperbot_supervisor_cpp/navigation_health_monitor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_supervisor_cpp::NavigationHealthMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
