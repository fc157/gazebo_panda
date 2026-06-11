import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('panda')

    return LaunchDescription([
        # Panda control node - sends joint trajectory goals
        Node(
            package='panda',
            executable='panda_node',
            name='panda_controller',
            output='screen',
            parameters=[],
        ),
    ])