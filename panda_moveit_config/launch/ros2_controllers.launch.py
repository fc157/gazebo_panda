import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Build the robot_description from the xacro
    robot_description_config = Command(
        [
            'xacro ', os.path.join(
                get_package_share_directory('panda'),
                'urdf', 'panda_arm_hand.urdf.xacro'
            )
        ]
    )
    robot_description = {'robot_description': robot_description_config}

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(
                get_package_share_directory('panda'),
                'config',
                'ros2_controllers.yaml',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        controller_manager_node,
    ])
