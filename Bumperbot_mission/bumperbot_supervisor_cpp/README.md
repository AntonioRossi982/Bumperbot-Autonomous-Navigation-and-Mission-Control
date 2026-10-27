# Bumperbot Supervisor

Application-level nodes that sit on top of the Nav2 stack: they send goals and monitor its health. Two packages with the same behavior:

- `bumperbot_supervisor_py` — Python implementation (`rclpy`)
- `bumperbot_supervisor_cpp` — C++ implementation (`rclcpp` / `rclcpp_action`)

Each package contains two nodes:

1) Mission Commander | `mission_commander` / `mission_commander_node` | Receives symbolic commands (`go_to room_a`, `cancel`), translates them into `NavigateToPose` goals, and sends them to Nav2 through an action client.                             |
2) | Navigation Health Monitor | `navigation_health_monitor` / `navigation_health_monitor_node` | Monitors AMCL covariance, the `map→odom→base_footprint` TF chain, `/scan` and `/odom` frequencies, and the number of aborted goals; publishes `diagnostic_msgs/DiagnosticArray`. |

## Main Interfaces

- `mission_command` (`std_msgs/String`) — input to the Mission Commander
- `mission_status` (`std_msgs/String`) — Mission Commander status
- `diagnostics` (`diagnostic_msgs/DiagnosticArray`) — Health Monitor status
- Action client on `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

## Parameters

Loaded from YAML files rather than hardcoded:

- `config/mission_locations.yaml` — locations (x, y, theta) and `max_retries`
- `config/health_monitor.yaml` — covariance thresholds, minimum frequencies, and aborted-goal threshold

## Build

```
cd ~/bumperbot_ws/src/BumperBot-Odometry-and-Control
cp -r bumperbot_supervisor_py bumperbot_supervisor_cpp .

cd ~/bumperbot_ws
colcon build --packages-select bumperbot_supervisor_py bumperbot_supervisor_cpp
source install/setup.bash
```

## Usage

```
# Run one node at a time
ros2 launch bumperbot_supervisor_cpp mission_commander.launch.py
ros2 launch bumperbot_supervisor_cpp health_monitor.launch.py

# Commands
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'go_to room_a'"
ros2 topic echo /mission_status
ros2 topic echo /diagnostics
```

## Notes

- These packages should be copied into the repository root, at the same level as `bumperbot_motion`, `bumperbot_navigation`, and the other packages. They should not be placed inside those packages, which contain Nav2 plugins (controllers/planners) rather than application-level nodes.
- They require `bumperbot_msgs` for `MissionStatus` (see `bumperbot_msgs_patch/`) only if structured status publishing with per-mission CSV output is restored. In the current version, `mission_status` is still a plain string.