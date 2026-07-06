# Copyright 2026
#
# Launch the search mission on the real robot.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation clock'),
    DeclareLaunchArgument(
        'mission_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('tb4_project'),
            'config',
            'diem_waypoints_reduced.yaml'
        ]),
        description='Full path to the waypoint mission yaml file'),
    DeclareLaunchArgument(
        'route_file', 
        default_value=PathJoinSubstitution([
            get_package_share_directory('tb4_project'),
            'config',
            'routes_smac2d_milp_reduced.yaml'
        ]),
        description='Optional offline route yaml file'),
    DeclareLaunchArgument('cost_matrix_file', default_value='',
                          description='Deprecated alias for route_file'),
    DeclareLaunchArgument(
        'visited_waypoints_file',
        default_value='',
        description='Optional YAML state file used to resume a waypoint search mission'),
    DeclareLaunchArgument('planner_id', default_value='',
                          description='Nav2 planner plugin id; empty uses default'),
    DeclareLaunchArgument(
        'behavior_tree',
        default_value=PathJoinSubstitution([
            get_package_share_directory('tb4_project'),
            'behavior_trees',
            'BT_search.xml'
        ]),
        description='Behavior Tree XML used only by the search mission'),
    DeclareLaunchArgument('initial_pose_topic', default_value='initialpose',
                          description='Topic used by RViz 2D Pose Estimate'),
    DeclareLaunchArgument('target_pose_topic', default_value='/aruco/pose',
                          description='Pose topic published by the ArUco detector'),
    DeclareLaunchArgument('goal_timeout_sec', default_value='0.0',
                          description='Per-goal timeout; 0 disables timeout'),
    DeclareLaunchArgument('continue_on_failure', default_value='true',
                          choices=['true', 'false'],
                          description='Continue with next waypoint after a failure'),
    DeclareLaunchArgument('waypoint_local_cost_check_enabled', default_value='true',
                          choices=['true', 'false'],
                          description='Skip a waypoint if its goal is blocked in local costmap'),
    DeclareLaunchArgument('waypoint_local_costmap_service',
                          default_value='/local_costmap/get_costmap',
                          description='Nav2 local costmap GetCostmap service'),
    DeclareLaunchArgument('waypoint_local_cost_threshold', default_value='253',
                          description='Cost threshold used to skip a waypoint'),
    DeclareLaunchArgument('waypoint_local_cost_radius', default_value='0.05',
                          description=(
                              'Small radius around waypoint goal checked in local costmap'
                          )),
    DeclareLaunchArgument('waypoint_local_cost_unknown_is_blocked',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Treat unknown local costmap cells as blocked'),
    DeclareLaunchArgument('waypoint_local_cost_check_period_sec',
                          default_value='0.75',
                          description='Period between local goal cost checks while navigating'),
    DeclareLaunchArgument('waypoint_local_cost_service_timeout_sec',
                          default_value='0.20',
                          description='Timeout for local costmap service calls'),
    DeclareLaunchArgument('waypoint_local_cost_tf_timeout_sec',
                          default_value='0.10',
                          description='Timeout for TF transform into local costmap frame'),
]


def generate_launch_description():
    search_mission = Node(
        package='tb4_project',
        executable='search_mission',
        name='search_mission',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)},
            {'mission_file': LaunchConfiguration('mission_file')},
            {'route_file': LaunchConfiguration('route_file')},
            {'cost_matrix_file': LaunchConfiguration('cost_matrix_file')},
            {'visited_waypoints_file': LaunchConfiguration('visited_waypoints_file')},
            {'planner_id': LaunchConfiguration('planner_id')},
            {'behavior_tree': LaunchConfiguration('behavior_tree')},
            {'initial_pose_topic': LaunchConfiguration('initial_pose_topic')},
            {'target_pose_topic': LaunchConfiguration('target_pose_topic')},
            {'goal_timeout_sec': ParameterValue(
                LaunchConfiguration('goal_timeout_sec'), value_type=float)},
            {'continue_on_failure': ParameterValue(
                LaunchConfiguration('continue_on_failure'), value_type=bool)},
            {'waypoint_local_cost_check_enabled': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_check_enabled'), value_type=bool)},
            {'waypoint_local_costmap_service': LaunchConfiguration(
                'waypoint_local_costmap_service')},
            {'waypoint_local_cost_threshold': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_threshold'), value_type=int)},
            {'waypoint_local_cost_radius': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_radius'), value_type=float)},
            {'waypoint_local_cost_unknown_is_blocked': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_unknown_is_blocked'),
                value_type=bool)},
            {'waypoint_local_cost_check_period_sec': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_check_period_sec'),
                value_type=float)},
            {'waypoint_local_cost_service_timeout_sec': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_service_timeout_sec'),
                value_type=float)},
            {'waypoint_local_cost_tf_timeout_sec': ParameterValue(
                LaunchConfiguration('waypoint_local_cost_tf_timeout_sec'),
                value_type=float)},
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(search_mission)
    return ld
