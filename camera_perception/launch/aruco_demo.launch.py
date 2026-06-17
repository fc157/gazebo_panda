"""
aruco_demo.launch.py

Complete launch for Aruco marker visual servoing demo.

This launch brings up:
  1. Gazebo with the Aruco marker world
  2. Robot state publisher + spawn Panda robot
  3. ROS 2 controllers (joint_state_broadcaster, arm, gripper)
  4. Aruco marker detector node
  5. RViz (optional)

Usage:
    ros2 launch camera_perception aruco_demo.launch.py [world:=<path_to_world>] [use_rviz:=true]

    Then in another terminal:
        ros2 launch moveit_control moveit_control.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_panda = get_package_share_directory('panda')
    pkg_camera_perception = get_package_share_directory('camera_perception')

    # Default world: Aruco world
    default_world_path = os.path.join(pkg_panda, 'worlds/aruco.world')

    # ---- Arguments ----
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to the Gazebo world file'
    )
    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # ---- Robot description ----
    default_model_path = os.path.join(pkg_panda, 'urdf/panda_arm_hand.urdf.xacro')
    robot_description_content = xacro.process_file(default_model_path).toxml()
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ---- Robot state publisher ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ---- GAZEBO_MODEL_PATH: tell Gazebo where to find Aruco marker models ----
    panda_model_path = os.path.join(pkg_panda, 'models')
    # Get existing GAZEBO_MODEL_PATH (if any) and prepend our models dir
    # We set it via environment for both gzserver and gzclient
    gazebo_env = os.environ.copy()
    existing_gmp = gazebo_env.get('GAZEBO_MODEL_PATH', '')
    if existing_gmp:
        gazebo_env['GAZEBO_MODEL_PATH'] = panda_model_path + ':' + existing_gmp
    else:
        gazebo_env['GAZEBO_MODEL_PATH'] = panda_model_path

    # ---- Gazebo server ----
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world'),
        ],
        output='screen',
        additional_env=gazebo_env,
    )

    # ---- Gazebo client GUI ----
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env=gazebo_env,
    )

    # ---- Spawn robot ----
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

    # ---- RViz ----
    rviz_config_file = os.path.join(pkg_panda, 'config/panda.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # ---- Load controllers ----
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen',
    )
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'panda_arm_controller'],
        output='screen',
    )
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'panda_gripper_controller'],
        output='screen',
    )

    delayed_jsb = TimerAction(period=5.0, actions=[load_joint_state_broadcaster])
    delayed_arm = TimerAction(period=7.0, actions=[load_arm_controller])
    delayed_gripper = TimerAction(period=9.0, actions=[load_gripper_controller])

    # ---- Aruco detector node ----
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

    # Start Aruco detector after gripper controller is loaded
    delayed_aruco = TimerAction(
        period=12.0,
        actions=[aruco_node],
    )

    # Start RViz after everything is loaded
    start_rviz_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gripper_controller,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path),
        world_arg,
        use_rviz_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        delayed_jsb,
        delayed_arm,
        delayed_gripper,
        delayed_aruco,
        start_rviz_after_spawn,
    ])