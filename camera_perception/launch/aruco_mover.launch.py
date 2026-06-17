"""
Launch the Aruco mover node for dynamically repositioning aruco_marker_1 in Gazebo.

Usage:
    ros2 launch camera_perception aruco_mover.launch.py
    ros2 launch camera_perception aruco_mover.launch.py update_rate:=5.0 reference_frame:=world
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mover_node = Node(
        package='camera_perception',
        executable='aruco_mover.py',
        name='aruco_mover',
        output='screen',
        parameters=[{
            'update_rate': 10.0,
            'initial_pose': [0.8, 0.0, 0.5, 0.0, 0.0, 0.0],  # x y z roll pitch yaw
            'reference_frame': 'world',
        }],
    )

    return LaunchDescription([
        mover_node,
    ])