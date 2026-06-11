import os
import yaml
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except (EnvironmentError, IOError):
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (EnvironmentError, IOError):
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory('panda_moveit_config')

    # RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz/panda.rviz')
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []

    # ============== Robot description (URDF) ==============
    # Same xacro pipeline used in planning_context.launch.py. Wrapped in
    # ParameterValue(..., value_type=str) so ROS 2 Foxy does not try to
    # YAML-parse the XML string.
    robot_description_content = Command(
        [
            'xacro ', os.path.join(
                get_package_share_directory('panda'),
                'urdf', 'panda_arm_hand.urdf.xacro'
            )
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ============== Robot description semantic (SRDF) ==============
    robot_description_semantic_config = load_file(
        'panda_moveit_config', 'config/panda.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # ============== Kinematics ==============
    kinematics_yaml = load_yaml(
        'panda_moveit_config', 'config/kinematics.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=rviz_args,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([rviz_node])
