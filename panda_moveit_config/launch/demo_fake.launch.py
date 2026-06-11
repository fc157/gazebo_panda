"""
MoveIt 2 demo for the Panda arm using the MoveIt FakeControllerManager
(no Gazebo, no real hardware). Useful for quick planning tests.

Usage:
    ros2 launch panda_moveit_config demo_fake.launch.py
"""
import os
import yaml
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(pkg, path):
    with open(os.path.join(get_package_share_directory(pkg), path), 'r') as f:
        return f.read()


def load_yaml(pkg, path):
    with open(os.path.join(get_package_share_directory(pkg), path), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Robot description from xacro
    robot_description_content = Command(
        [
            'xacro ', os.path.join(
                get_package_share_directory('panda'),
                'urdf', 'panda_arm_hand.urdf.xacro'
            )
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_description_semantic = {
        'robot_description_semantic': load_file(
            'panda_moveit_config', 'config/panda.srdf')
    }

    kinematics_yaml = load_yaml('panda_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('panda_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('panda_moveit_config', 'config/ompl_planning.yaml')
    initial_positions_yaml = load_yaml(
        'panda_moveit_config', 'config/initial_positions.yaml')

    # Fake controllers (offline test): no Gazebo, MoveIt directly drives the URDF
    fake_controllers_yaml = {
        'moveit_controller_manager':
            'moveit_fake_controller_manager/MoveItFakeControllerManager',
        'moveit_fake_controller_manager': {
            'controller_names': [
                'panda_arm_controller',
                'panda_gripper_controller',
            ],
            'panda_arm_controller': {
                'type': 'FollowJointTrajectory',
                'joints': [
                    'panda_joint1', 'panda_joint2', 'panda_joint3',
                    'panda_joint4', 'panda_joint5', 'panda_joint6',
                    'panda_joint7',
                ],
                'trajectory_execution': {
                    'allowed_execution_duration_s': 10.0,
                    'allowed_goal_duration_margin': 0.5,
                },
            },
            'panda_gripper_controller': {
                'type': 'FollowJointTrajectory',
                'joints': ['panda_finger_joint1'],
            },
        },
    }

    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planning_request_adapters/ResolveConstraintFrames '
                'default_planning_request_adapters/ValidateWorkspaceBounds '
                'default_planning_request_adapters/CheckStartStateInCollision '
                'default_planning_request_adapters/CheckStartStateBounds '
                'default_planning_request_adapters/AddTimeOptimalParameterization'
            ),
            'response_adapters': (
                'default_planning_response_adapters/AddTimeOptimalParameterization '
                'default_planning_response_adapters/ValidateSolution'
            ),
        },
    }
    ompl_planning_pipeline_config = {**planning_pipelines, **ompl_planning_yaml}

    # Publish joint states for the URDF so MoveIt knows the current state
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'rate': 50,
        }],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            fake_controllers_yaml,
            initial_positions_yaml,
            {'use_sim_time': False},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('panda_moveit_config'),
            'launch', 'moveit.rviz')],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
