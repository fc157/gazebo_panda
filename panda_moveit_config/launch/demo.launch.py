"""
Launch the full MoveIt 2 + Gazebo Panda demo.

Timing diagram (approximate):
    t=0.0 s   Gazebo server/client, robot_state_publisher, spawn robot
    t=5.0 s   joint_state_broadcaster
    t=7.0 s   panda_arm_controller
    t=9.0 s   panda_gripper_controller (trajectory controller)
    t=10.0 s  move_group (MoveIt)
    t=12.0 s  RViz (with MoveIt plugin)

Usage:
    ros2 launch panda_moveit_config demo.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Robot description (shared).
    # NOTE: in ROS 2 Foxy the launch system tries to YAML-parse the value of
    # every node parameter, which breaks for long XML/URDF strings that
    # happen to contain characters YAML treats specially (e.g. ':' inside
    # an XML comment, the leading '<?xml ...?>' declaration, etc.).
    # We therefore wrap the xacro output in ParameterValue(..., value_type=str)
    # so it is forwarded as a plain string.
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

    # ============================================================
    # 1) Robot state publisher (so /tf is published before Gazebo)
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ============================================================
    # 2) Gazebo server (headless) + client (GUI)
    # ============================================================
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            os.path.join(
                get_package_share_directory('panda'),
                'worlds', 'calibration.world'
            ),
        ],
        output='screen',
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # ============================================================
    # 3) Spawn the robot in Gazebo
    # ============================================================
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_panda',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'panda',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
        ],
    )

    # ============================================================
    # 4) Load ros2_control controllers
    # ============================================================
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen',
    )
    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'panda_arm_controller'],
        output='screen',
    )
    load_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'panda_gripper_controller'],
        output='screen',
    )

    # ============================================================
    # 5) MoveIt move_group node
    # ============================================================
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('panda_moveit_config'),
                'launch',
                'planning_context.launch.py',
            )
        )
    )

    # ============================================================
    # 6) RViz with MoveIt plugin
    # ============================================================
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('panda_moveit_config'),
                'launch',
                'moveit_rviz.launch.py',
            )
        )
    )

    # ============================================================
    # Sequencing (start everything in the correct order)
    # ============================================================
    start_jsb = TimerAction(period=5.0, actions=[load_jsb])
    start_arm = TimerAction(period=7.0, actions=[load_arm])
    start_gripper = TimerAction(period=9.0, actions=[load_gripper])

    # MoveIt: wait until the last controller is loaded, then start move_group
    start_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gripper,
            on_exit=[move_group_launch],
        )
    )

    # RViz: wait a few more seconds for move_group to be fully initialized
    start_rviz = TimerAction(period=12.0, actions=[moveit_rviz_launch])

    return LaunchDescription([
        DeclareLaunchArgument('load_gripper', default_value='true'),
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        start_jsb,
        start_arm,
        start_gripper,
        start_moveit,
        start_rviz,
    ])
