# Bumperbot Supervisor (Python) — `bumperbot_supervisor_py`

Python package (`rclpy`) containing two application-level nodes that sit on top of the Nav2 stack.

| Node | Executable | Role |
1) | Mission Commander | `mission_commander` | Receives symbolic commands (`go_to room_a`, `cancel`), translates them into `NavigateToPose` goals, and sends them to Nav2 through an action client. |
2) | Navigation Health Monitor | `navigation_health_monitor` | Monitors AMCL covariance, the `map→odom→base_footprint` TF chain, `/scan` and `/odom` frequencies, and the number of aborted goals; publishes `diagnostic_msgs/DiagnosticArray`. |

## Interfaces

- `mission_command` (`std_msgs/String`) input: `"go_to <location>"` / `"cancel"`
- `mission_status` (`std_msgs/String`) output — simple status messages (`SENDING`, `ACCEPTED`, `REJECTED`, `SUCCEEDED`, `CANCELED`, `FAILED`, `ERROR ...`). **It does not yet use the structured `bumperbot_msgs/MissionStatus` message with per-mission CSV output**: that version exists as a design, but is not part of the current codebase.
- `diagnostics` (`diagnostic_msgs/DiagnosticArray`) output from the Health Monitor
- Action client on `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

## Parameters

Loaded from YAML files rather than hardcoded:

- `config/mission_locations.yaml` — locations (x, y, theta) and `max_retries`
- `config/health_monitor.yaml` — covariance thresholds, minimum frequencies, and aborted-goal threshold

## Build

```bash
cd ~/bumperbot_ws/src/BumperBot-Odometry-and-Control
cp -r bumperbot_supervisor_py .

cd ~/bumperbot_ws
colcon build --packages-select bumperbot_supervisor_py
source install/setup.bash
```

## Usage

```bash
ros2 launch bumperbot_supervisor_py mission_commander.launch.py
ros2 launch bumperbot_supervisor_py health_monitor.launch.py

ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'go_to room_a'"
ros2 topic echo /mission_status
ros2 topic echo /diagnostics
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'cancel'"
```

## Notes

- The package should be copied into the repository root, at the same level as `bumperbot_motion`, `bumperbot_navigation`, and the other packages. It should **not** be placed inside those packages, which contain Nav2 plugins (controllers/planners) rather than application-level nodes.
- Equivalent C++ implementation: `bumperbot_supervisor_cpp`.