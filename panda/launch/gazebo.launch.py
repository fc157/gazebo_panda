import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('panda')

    default_model_path = os.path.join(pkg_share, 'urdf/panda_arm_hand.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds/empty.world')
    print("Default model path: ", default_model_path)
    print("Default world path: ", default_world_path)
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to the world file to load in Gazebo'
    )

    robot_description_content = xacro.process_file(default_model_path).toxml()

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Gazebo server
    # 注意：不要在这里加载 libgazebo_ros2_control.so，它会在URDF中定义并由Gazebo自动加载
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world'),
        ],
        output='screen',
    )

    # Gazebo client GUI
    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output='screen',
    )

    # Spawn robot
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

    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config/panda.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )


    # 注意：控制器加载命令现在不需要命名空间前缀，因为gazebo_ros2_control插件不再使用/panda命名空间
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "panda_arm_controller",
        ],
        output="screen",
    )
    load_gripper_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "panda_gripper_controller",
        ],
        output="screen",
    )
    delayed_jsb = TimerAction(
        period=5.0,
        actions=[load_joint_state_broadcaster],
    )

    delayed_arm = TimerAction(
        period=7.0,
        actions=[load_arm_controller],
    )

    delayed_gripper = TimerAction(
        period=9.0,
        actions=[load_gripper_controller],
    )

    # 等待 spawn_robot 进程结束后再启动 RViz，
    # 这样可以保证 robot_state_publisher、Gazebo 渲染、TF 等都准备就绪，
    # 避免 RViz 启动过早导致 robot_description 还未发布，模型无法加载
    start_rviz_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gripper_controller,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('load_gripper', default_value='true'),
        world_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        delayed_jsb,
        delayed_arm,
        delayed_gripper,
        start_rviz_after_spawn,
    ])
