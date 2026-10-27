#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

"""
Navigation Health Monitor
==========================
Monitors the health of the navigation stack:
  - covariance of /amcl_pose (localization quality)
  - availability of the map -> odom -> base_footprint transform chain
  - frequency of /scan and /odom
  - number of aborted NavigateToPose goals

Periodically publishes a diagnostic_msgs/DiagnosticArray and logs a
warning when localization degrades (edge-triggered: only when the
state worsens, not on every cycle).
"""

class RateTracker:
    """Tracks the arrival rate of a topic over a sliding time window."""

    def __init__(self, window_sec: float):
        self._window_sec = window_sec
        self._stamps = deque()

    def tick(self, now_sec: float):
        self._stamps.append(now_sec)
        self._prune(now_sec)

    def rate_hz(self, now_sec: float) -> float:
        self._prune(now_sec)
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self._stamps) - 1) / span

    def last_stamp(self):
        return self._stamps[-1] if self._stamps else None

    def _prune(self, now_sec: float):
        while self._stamps and now_sec - self._stamps[0] > self._window_sec:
            self._stamps.popleft()


class NavigationHealthMonitor(Node):

    def __init__(self):
        super().__init__('navigation_health_monitor', automatically_declare_parameters_from_overrides=True)

        self._hardware_id = self._get_or_declare('hardware_id', 'bumperbot')
        self._map_frame = self._get_or_declare('map_frame', 'map')
        self._odom_frame = self._get_or_declare('odom_frame', 'odom')
        self._base_frame = self._get_or_declare('base_frame', 'base_footprint')

        self._pos_cov_warn = self._get_or_declare('position_covariance_warn', 0.05)
        self._pos_cov_error = self._get_or_declare('position_covariance_error', 0.25)
        self._yaw_cov_warn = self._get_or_declare('yaw_covariance_warn', 0.05)
        self._yaw_cov_error = self._get_or_declare('yaw_covariance_error', 0.25)

        self._min_scan_rate = self._get_or_declare('min_scan_rate', 5.0)
        self._min_odom_rate = self._get_or_declare('min_odom_rate', 10.0)
        self._rate_window = self._get_or_declare('rate_window', 5.0)
        self._stale_timeout = self._get_or_declare('stale_timeout', 2.0)

        self._max_aborted_goals_warn = self._get_or_declare('max_aborted_goals_warn', 3)

        diagnostics_period = self._get_or_declare('diagnostics_period', 1.0)

        self._cb_group = ReentrantCallbackGroup()

        # --- AMCL ---
        self._amcl_received = False
        self._latest_pos_std = 0.0
        self._latest_yaw_std = 0.0
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_callback, 10, callback_group=self._cb_group)

        # --- TF ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        # --- Scan / Odom rate ---
        self._scan_rate = RateTracker(self._rate_window)
        self._odom_rate = RateTracker(self._rate_window)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10, callback_group=self._cb_group)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10, callback_group=self._cb_group)

        # --- Goal aborts ---
        self._aborted_goal_ids = set()
        self._aborted_count = 0
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._nav2_status_callback, 10, callback_group=self._cb_group)

        self._diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self._was_localization_ok = True

        self.create_timer(diagnostics_period, self._publish_diagnostics, callback_group=self._cb_group)
        self.get_logger().info('Navigation Health Monitor ready.')

    def _get_or_declare(self, name, default):
        
        if self.has_parameter(name):
            return self.get_parameter(name).value
        
        return self.declare_parameter(name, default).value
    
    # ------------------------------------------------------------------ #
    # Subscription callbacks
    # ------------------------------------------------------------------ #
    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yaw = cov[35]
        self._latest_pos_std = math.sqrt(max(cov_xx, 0.0) + max(cov_yy, 0.0))
        self._latest_yaw_std = math.sqrt(max(cov_yaw, 0.0))
        self._amcl_received = True

    def _scan_callback(self, msg: LaserScan):
        self._scan_rate.tick(self._now_sec())

    def _odom_callback(self, msg: Odometry):
        self._odom_rate.tick(self._now_sec())

    def _nav2_status_callback(self, msg: GoalStatusArray):
        
        for entry in msg.status_list:
            
            if entry.status != GoalStatus.STATUS_ABORTED:
                continue
            
            goal_id = bytes(entry.goal_info.goal_id.uuid)
            
            if goal_id in self._aborted_goal_ids:
                continue
            
            self._aborted_goal_ids.add(goal_id)
            self._aborted_count += 1
            self.get_logger().warn(f'NavigateToPose goal aborted (total: {self._aborted_count})')

    # ------------------------------------------------------------------ #
    # Checks
    # ------------------------------------------------------------------ #
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _check_localization(self):
        if not self._amcl_received:
            return DiagnosticStatus.ERROR, 'No messages received on /amcl_pose', 0.0, 0.0

        pos_std = self._latest_pos_std
        yaw_std = self._latest_yaw_std

        if pos_std >= self._pos_cov_error or yaw_std >= self._yaw_cov_error:
            level = DiagnosticStatus.ERROR
            message = 'Severely degraded localization (very high covariance)'
        elif pos_std >= self._pos_cov_warn or yaw_std >= self._yaw_cov_warn:
            level = DiagnosticStatus.WARN
            message = 'Degraded localization (high covariance)'
        else:
            level = DiagnosticStatus.OK
            message = 'Localization nominal'

        return level, message, pos_std, yaw_std

    def _check_tf_chain(self):
        
        # timeout=0: non-blocking read from the TF buffer, populated in the
        # background by the TransformListener. This avoids any risk of
        # deadlock if the node runs on a single-threaded executor
        # (the timer callback must never block while waiting for data that
        # would arrive from another callback in the same executor).
        
        odom_ok = self._tf_buffer.can_transform(self._odom_frame, self._base_frame, Time())
        map_ok = self._tf_buffer.can_transform(self._map_frame, self._odom_frame, Time())

        if odom_ok and map_ok:
            return DiagnosticStatus.OK, (
                f'TF chain {self._map_frame}->{self._odom_frame}->'
                f'{self._base_frame} available')

        missing = []
        if not map_ok:
            missing.append(f'{self._map_frame}->{self._odom_frame}')
        if not odom_ok:
            missing.append(f'{self._odom_frame}->{self._base_frame}')
        return DiagnosticStatus.ERROR, 'Missing transforms: ' + ', '.join(missing)

    def _check_rate(self, tracker: RateTracker, min_rate: float, label: str):
        now = self._now_sec()
        last = tracker.last_stamp()
        if last is None:
            return DiagnosticStatus.ERROR, f'No messages received on {label}', 0.0

        age = now - last
        rate = tracker.rate_hz(now)

        if age > self._stale_timeout:
            return (DiagnosticStatus.ERROR, f'{label}: no data for {age:.1f} s (expected >= {min_rate:.1f} Hz)', rate)
        if rate < min_rate:
            return (DiagnosticStatus.WARN, f'{label}: low frequency ({rate:.1f} Hz, expected >= {min_rate:.1f} Hz)', rate)
        return DiagnosticStatus.OK, f'{label}: {rate:.1f} Hz', rate

    def _check_aborted_goals(self):
        if self._aborted_count >= self._max_aborted_goals_warn:
            return (
                DiagnosticStatus.WARN,
                f'{self._aborted_count} aborted goals (threshold: '
                f'{self._max_aborted_goals_warn})')
        return DiagnosticStatus.OK, f'{self._aborted_count} aborted goals'

    # ------------------------------------------------------------------ #
    # Diagnostics publishing
    # ------------------------------------------------------------------ #
    def _publish_diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        loc_level, loc_msg, pos_std, yaw_std = self._check_localization()
        loc_status = DiagnosticStatus()
        loc_status.name = 'navigation_health_monitor: AMCL localization'
        loc_status.hardware_id = self._hardware_id
        loc_status.level = loc_level
        loc_status.message = loc_msg
        loc_status.values = [
            KeyValue(key='position_std_dev_m', value=f'{pos_std:.4f}'),
            KeyValue(key='yaw_std_dev_rad', value=f'{yaw_std:.4f}'),
        ]
        array.status.append(loc_status)

        # Localization degradation warning: edge-triggered only,
        # when the state worsens compared to the previous cycle, to avoid
        # spamming the logs on every publication.
        localization_ok_now = loc_level == DiagnosticStatus.OK
        if self._was_localization_ok and not localization_ok_now:
            self.get_logger().warn(f'Localization degraded: {loc_msg}')
        self._was_localization_ok = localization_ok_now

        tf_level, tf_msg = self._check_tf_chain()
        tf_status = DiagnosticStatus()
        tf_status.name = 'navigation_health_monitor: TF chain'
        tf_status.hardware_id = self._hardware_id
        tf_status.level = tf_level
        tf_status.message = tf_msg
        array.status.append(tf_status)

        scan_level, scan_msg, scan_rate = self._check_rate(self._scan_rate, self._min_scan_rate, '/scan')
        scan_status = DiagnosticStatus()
        scan_status.name = 'navigation_health_monitor: Laser scan rate'
        scan_status.hardware_id = self._hardware_id
        scan_status.level = scan_level
        scan_status.message = scan_msg
        scan_status.values = [KeyValue(key='rate_hz', value=f'{scan_rate:.2f}')]
        array.status.append(scan_status)

        odom_level, odom_msg, odom_rate = self._check_rate(self._odom_rate, self._min_odom_rate, '/odom')
        odom_status = DiagnosticStatus()
        odom_status.name = 'navigation_health_monitor: Odometry rate'
        odom_status.hardware_id = self._hardware_id
        odom_status.level = odom_level
        odom_status.message = odom_msg
        odom_status.values = [KeyValue(key='rate_hz', value=f'{odom_rate:.2f}')]
        array.status.append(odom_status)

        aborts_level, aborts_msg = self._check_aborted_goals()
        aborts_status = DiagnosticStatus()
        aborts_status.name = 'navigation_health_monitor: Navigation aborts'
        aborts_status.hardware_id = self._hardware_id
        aborts_status.level = aborts_level
        aborts_status.message = aborts_msg
        aborts_status.values = [KeyValue(key='aborted_goals_total', value=str(self._aborted_count))]
        array.status.append(aborts_status)

        self._diag_pub.publish(array)


def main(args=None):
    
    rclpy.init(args=args)
    node = NavigationHealthMonitor()
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