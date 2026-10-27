cmake_minimum_required(VERSION 3.8)
project(bumperbot_supervisor_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bumperbot_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_executable(mission_commander_node src/mission_commander.cpp src/mission_commander_node.cpp)
ament_target_dependencies(mission_commander_node rclcpp rclcpp_action nav2_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs)

add_executable(navigation_health_monitor_node src/navigation_health_monitor.cpp src/navigation_health_monitor_node.cpp)
ament_target_dependencies(navigation_health_monitor_node rclcpp action_msgs diagnostic_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros unique_identifier_msgs)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS mission_commander_node navigation_health_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
