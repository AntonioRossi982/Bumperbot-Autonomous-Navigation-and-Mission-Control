import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bumperbot_supervisor_py'),
        'config',
        'mission_locations.yaml',
    )

    mission_commander_node = Node(
        package='bumperbot_supervisor_py',
        executable='mission_commander',
        name='mission_commander',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        mission_commander_node
        ])
