import os
import yaml

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), 'r') as f:
        return f.read()


def _load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_panda = get_package_share_directory('panda')
    pkg_mc    = get_package_share_directory('panda_moveit_config')

    # ---- robot_description (URDF) ----
    robot_description_content = Command(
        ['xacro ', os.path.join(pkg_panda, 'urdf', 'panda_arm_hand.urdf.xacro')]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    # ---- robot_description_semantic (SRDF) ----
    srdf_str = _load_file('panda_moveit_config', 'config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic': srdf_str}

    # ---- kinematics / joint_limits / OMPL planning ----
    kinematics_yaml   = _load_yaml('panda_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = _load_yaml('panda_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = _load_yaml('panda_moveit_config', 'config/ompl_planning.yaml')

    # ---- planning_pipelines (string, NOT tuple) ----
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/AddTimeOptimalParameterization',
            'response_adapters': 'default_planner_response_adapters/AddTimeOptimalParameterization '
                                 'default_planner_response_adapters/LimitMaxCartesianLinkSpeed '
                                 'default_planner_response_adapters/ValidateSolution',
        },
    }
    ompl_planning_pipeline_config = {**planning_pipelines, **(ompl_planning_yaml or {})}

    # ---- MoveIt controllers (so MoveGroup knows about ros2_controllers) ----
    moveit_controllers_yaml = _load_yaml(
        'panda_moveit_config', 'config/moveit_controllers.yaml'
    )

    # ---- move_group (re-uses the existing planning_context launch) ----
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mc, 'launch', 'planning_context.launch.py')
        )
    )

    # ---- the C++ test node ----
    # CRITICAL: MoveGroupInterface needs URDF + SRDF + kinematics + joint_limits
    # + planning pipeline on THIS node's private parameter namespace.
    # If any of these is missing, the RobotModel is built without groups
    # and the first call to MoveGroupInterface throws
    #   "Group 'panda_arm' / 'panda_arm_hand' was not found."
    moveit_test_node = Node(
        package='panda_moveit_config',
        executable='moveit_test',
        name='moveit_test',
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

    # Start moveit_test a couple of seconds after move_group so parameter
    # publication has settled.
    start_moveit_test = TimerAction(period=2.0, actions=[moveit_test_node])

    return LaunchDescription([
        #move_group_launch,
        start_moveit_test,
    ])
