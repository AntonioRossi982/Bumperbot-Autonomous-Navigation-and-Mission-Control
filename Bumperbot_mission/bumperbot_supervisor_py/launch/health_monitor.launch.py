import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bumperbot_supervisor_py'),
        'config',
        'health_monitor.yaml',
    )

    health_monitor_node = Node(
        package='bumperbot_supervisor_py',
        executable='navigation_health_monitor',
        name='navigation_health_monitor',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        health_monitor_node
        ])
