import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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
    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        'db', default_value='false', description='Database flag'
    )

    # ============== Robot description ==============
    # NOTE: in ROS 2 Foxy the launch system tries to YAML-parse the value of
    # every node parameter, which breaks for long XML/URDF strings that
    # happen to contain characters YAML treats specially (e.g. ':' inside
    # an XML comment, the leading '<?xml ...?>' declaration, etc.).
    # We therefore wrap the xacro output in ParameterValue(..., value_type=str)
    # so it is forwarded as a plain string.
    robot_description_config = Command(
        [
            'xacro ', os.path.join(
                get_package_share_directory('panda'),
                'urdf', 'panda_arm_hand.urdf.xacro'
            )
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_config, value_type=str
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

    # ============== Joint limits ==============
    joint_limits_yaml = load_yaml(
        'panda_moveit_config', 'config/joint_limits.yaml'
    )

    # ============== OMPL planning ==============
    ompl_planning_yaml = load_yaml(
        'panda_moveit_config', 'config/ompl_planning.yaml'
    )

    # ============== MoveIt controllers (simple controller manager) ==============
    moveit_controllers_yaml = load_yaml(
        'panda_moveit_config', 'config/moveit_controllers.yaml'
    )

    # ============== 3D sensors (octomap from camera point cloud) ==============
    sensors_yaml = {
        'sensors': [],
    }

    # ============== Planning pipeline configuration ==============
    # NOTE: the values for `request_adapters` and `response_adapters` must be
    # plain strings (NOT tuples). ROS 2 Foxy's parameter evaluator only accepts
    # primitive scalar types (float / int / str / bool / bytes) at the top
    # level of a parameter dict. Using a tuple like ( ... ) will raise:
    #   TypeError: Expected 'value' to be one of [...], but got '()' of type 'tuple'
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
    # Merge pipeline + OMPL configs into a single dict
    ompl_planning_pipeline_config = {
        **planning_pipelines,
        **ompl_planning_yaml,
    }

    # ============== MoveGroup capabilities ==============
    # NOTE: capability class names must match exactly what the pluginlib
    # declares. In MoveIt 2 Foxy the valid ones are e.g.
    #   MoveGroupExecuteTrajectoryAction  (not 'ExecuteTrajectoryAction')
    #   MoveGroupCartesianPathService     (not 'MoveGroupCartesianPath')
    # 'PlanOnlyExecutingService' does not exist in this version and has
    # been removed.
    move_group_capabilities = {
        'capabilities': 'move_group/MoveGroupExecuteTrajectoryAction '
                        'move_group/MoveGroupCartesianPathService '
                        'move_group/MoveGroupKinematicsService '
                        'move_group/MoveGroupPlanService '
                        'move_group/MoveGroupQueryPlannersService',
    }

    # ============== Start the move_group node ==============
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,
            #sensors_yaml,
            move_group_capabilities,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        db_arg,
        run_move_group_node,
    ])
