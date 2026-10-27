#!/usr/bin/env python3
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

"""
Mission Commander
==================
Receives symbolic commands on a topic ("go_to room_a", "cancel"), translates
them into a PoseStamped in the map frame, and sends them as NavigateToPose goals
to the Nav2 action server. Handles goal rejection/feedback/results, allows only
one goal at a time, supports cancellation, and performs a limited number of
retries in case of failure.

Locations are NOT hardcoded: they are loaded from ROS 2 parameters
(YAML file, see config/mission_locations.yaml).
"""

def yaw_to_quaternion(yaw: float):
    """Converts a yaw angle (rad) into a quaternion (x, y, z, w) for planar rotation."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionCommander(Node):

    def __init__(self):
        super().__init__('mission_commander', automatically_declare_parameters_from_overrides=True)

        # Scalar parameters: if not provided in YAML, use default values.
        self._max_retries = self._get_or_declare('max_retries', 3)
        self._frame_id = self._get_or_declare('goal_frame_id', 'map')

        self._locations = self._load_locations()
        if not self._locations:
            self.get_logger().warn('No locations found in the "locations.*" parameters. ' 'Check the YAML file passed to the node.')

        # Reentrant callback group: allows feedback/result/goal_response callbacks
        # from the action client and the command subscription to be processed
        # concurrently by a MultiThreadedExecutor.
        self._cb_group = ReentrantCallbackGroup()

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group)

        self._status_pub = self.create_publisher(String, 'mission_status', 10)

        self._command_sub = self.create_subscription(String, 'mission_command', self._command_callback, 10, callback_group=self._cb_group)

        # Internal state: only one goal at a time.
        self._goal_handle = None
        self._current_location = None
        self._retry_count = 0
        self._goal_active = False

        self.get_logger().info(f'Mission Commander ready. Known locations: {list(self._locations.keys())}')

    # ------------------------------------------------------------------ #
    # Parameters / locations
    # ------------------------------------------------------------------ #
    def _get_or_declare(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value

    def _load_locations(self):
        """
        Reads all locations from the "locations.*" parameter namespace.
        Expected format in the YAML file:

        mission_commander:
          ros__parameters:
            locations:
              room_a:
                x: 1.0
                y: 2.0
                theta: 0.0
        """
        raw = {}
        for full_name, param in self.get_parameters_by_prefix('locations').items():
            # full_name, e.g. "room_a.x"
            if '.' not in full_name:
                continue
            loc_name, field = full_name.split('.', 1)
            raw.setdefault(loc_name, {})[field] = param.value

        locations = {}
        for name, fields in raw.items():
            locations[name] = (
                float(fields.get('x', 0.0)),
                float(fields.get('y', 0.0)),
                float(fields.get('theta', 0.0)),
            )
        return locations
    
    # ------------------------------------------------------------------ #
    # Incoming commands
    # ------------------------------------------------------------------ #
    def _command_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        if text.lower() == 'cancel':
            self._cancel_goal()
            return

        if not text.lower().startswith('go_to'):
            self.get_logger().warn(f'Unrecognized command: "{text}"')
            return

        parts = text.split(maxsplit=1)
        if len(parts) != 2:
            self.get_logger().warn('The "go_to" command is missing a location: use "go_to <location>"')
            return

        self._start_mission(parts[1].strip())

    def _start_mission(self, location_name: str):
        if location_name not in self._locations:
            self.get_logger().error(
                f'Unknown location: "{location_name}". '
                f'Available locations: {list(self._locations.keys())}'
            )
            self._publish_status(f'ERROR unknown_location {location_name}')
            return

        if self._goal_active:
            self.get_logger().warn(
                'A goal is already active: ignoring the new request. '
                'Send "cancel" to cancel the current goal before sending a new one.')
            self._publish_status('REJECTED goal_already_active')
            return

        self._current_location = location_name
        self._retry_count = 0
        self._send_goal(location_name)

    # ------------------------------------------------------------------ #
    # Sending goals
    # ------------------------------------------------------------------ #
    def _build_pose(self, x: float, y: float, theta: float) -> PoseStamped:
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quaternion(theta)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            return pose

    def _send_goal(self, location_name: str):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server "navigate_to_pose" is unavailable')
            self._publish_status('ERROR action_server_unavailable')
            self._goal_active = False
            return

        x, y, theta = self._locations[location_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(x, y, theta)

        self._goal_active = True
        self.get_logger().info(
            f'Sending goal to "{location_name}" '
            f'(attempt {self._retry_count + 1}/{self._max_retries + 1}): '
            f'x={x:.2f} y={y:.2f} theta={theta:.2f}'
        )
        self._publish_status(f'SENDING {location_name}')

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------ #
    # Action client callbacks
    # ------------------------------------------------------------------ #
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected by the server for "{self._current_location}"')
            self._publish_status(f'REJECTED {self._current_location}')
            self._handle_failure()
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        self._publish_status(f'ACCEPTED {self._current_location}')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    def _result_callback(self, future):
        status = future.result().status
        self._goal_active = False
        self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal reached: "{self._current_location}"')
            self._publish_status(f'SUCCEEDED {self._current_location}')
            self._reset_mission_state()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Goal canceled: "{self._current_location}"')
            self._publish_status(f'CANCELED {self._current_location}')
            self._reset_mission_state()
        else:
            self.get_logger().warn(f'Goal failed (status={status}) for "{self._current_location}"')
            self._handle_failure()

    # ------------------------------------------------------------------ #
    # Retry / cancellation
    # ------------------------------------------------------------------ #
    def _handle_failure(self):
        self._retry_count += 1
        if self._retry_count > self._max_retries:
            self.get_logger().error(f'Maximum number of attempts exceeded for "{self._current_location}". ' 'Mission aborted.')
            self._publish_status(f'FAILED {self._current_location}')
            self._reset_mission_state()
            return

        self.get_logger().info(f'Retrying goal submission ({self._retry_count}/{self._max_retries})')
        self._send_goal(self._current_location)

    def _cancel_goal(self):
        if not self._goal_active or self._goal_handle is None:
            self.get_logger().info('No active goal to cancel')
            return

        self.get_logger().info(f'Sending cancellation request for "{self._current_location}"')
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancellation request accepted')
        else:
            self.get_logger().warn('Cancellation request rejected')

    def _reset_mission_state(self):
        self._retry_count = 0
        self._current_location = None

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = MissionCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
