"""
Launch the Aruco marker detector node.

Usage:
    ros2 launch camera_perception aruco_detector.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_node = Node(
        package='camera_perception',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'camera_topic': '/hand_camera/image_raw',
            'aruco_dict': 'DICT_6X6_250',
            'marker_size': 0.2,
            'publish_debug_image': True,
            'camera_fx': 554.0,
            'camera_fy': 554.0,
            'camera_cx': 320.0,
            'camera_cy': 240.0,
        }],
    )

    return LaunchDescription([
        aruco_node,
    ])