"""
Launch the MoveIt Control GUI (C++ / Qt5).

This launch file starts the MoveIt control GUI application which provides
a Qt5 graphical interface for controlling the Panda robot arm via
MoveIt 2. It connects to the already-running move_group node.

Usage:
    # Terminal 1: Start the full demo (Gazebo + MoveIt + RViz)
    ros2 launch panda_moveit_config demo.launch.py

    # Terminal 2: Start the control GUI (after the demo is fully up)
    ros2 launch moveit_control moveit_control.launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS 2 package."""
    try:
        abs_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def load_file(package_name, file_path):
    """Load a text file from a ROS 2 package."""
    try:
        abs_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(abs_path, 'r') as f:
            return f.read()
    except Exception:
        return ""


def generate_launch_description():
    # ---- robot_description (URDF via xacro) ----
    robot_description_content = Command(
        ['xacro ', os.path.join(
            get_package_share_directory('panda'),
            'urdf', 'panda_arm_hand.urdf.xacro'
        )]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    # ---- robot_description_semantic (SRDF) ----
    srdf_str = load_file('panda_moveit_config', 'config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic': srdf_str}

    # ---- kinematics / joint_limits / OMPL planning ----
    kinematics_yaml   = load_yaml('panda_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('panda_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('panda_moveit_config', 'config/ompl_planning.yaml')

    # ---- planning_pipelines ----
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/AddTimeOptimalParameterization'
            ),
            'response_adapters': (
                'default_planner_response_adapters/AddTimeOptimalParameterization '
                'default_planner_response_adapters/LimitMaxCartesianLinkSpeed '
                'default_planner_response_adapters/ValidateSolution'
            ),
        },
    }
    ompl_planning_pipeline_config = {
        **(ompl_planning_yaml or {}),
        **planning_pipelines,
    }

    # ---- MoveIt controllers ----
    moveit_controllers_yaml = load_yaml(
        'panda_moveit_config', 'config/moveit_controllers.yaml'
    )

    # ---- Start the GUI node with all necessary MoveIt parameters ----
    gui_node = Node(
        package='moveit_control',
        executable='moveit_control_gui',
        name='moveit_control_gui',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        gui_node,
    ])