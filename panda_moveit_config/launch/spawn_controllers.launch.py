import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (EnvironmentError, IOError):
        return None


def generate_launch_description():
    # Optional flag: use a fake controller (MoveIt FakeControllerManager) for offline testing
    use_fake_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='If true, use MoveIt fake controllers (no real Gazebo needed).',
    )

    controllers_yaml = load_yaml(
        'panda_moveit_config', 'config/panda_controllers.yaml'
    )

    # If the ros2_control controllers are already running in Gazebo, the simplest
    # way to bridge MoveIt to them is to expose the controller_manager as a
    # MoveItSimpleControllerManager (handled by moveit_controllers.yaml).
    # The spawn_controllers launch file is therefore mostly a placeholder used
    # by the demo / integration tests to load controllers in the real robot case.

    # Spawn MoveItSimpleControllerManager bridge via ros2_control
    # (not strictly necessary when controllers already run in Gazebo)
    load_controllers = []
    for controller in controllers_yaml['controller_names']:
        load_controllers.append(
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[controller, '--controller-manager-timeout', '30'],
            )
        )

    return LaunchDescription([
        use_fake_arg,
    ] + load_controllers)
