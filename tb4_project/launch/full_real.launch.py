# Copyright 2026
#
# Bring up the full real-robot pipeline from a single launch file.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


TB4_PROJECT = FindPackageShare('tb4_project')


def _default_diem_map():
    tb4_project_share = Path(get_package_share_directory('tb4_project'))
    workspace_root = tb4_project_share.parents[3]
    map_file = workspace_root / 'src' / 'map' / 'diem_map.yaml'
    return str(map_file)


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation clock'),

    DeclareLaunchArgument(
        'map',
        default_value=_default_diem_map(),
        description='Map yaml path'),
    DeclareLaunchArgument(
        'localization_params',
        default_value=PathJoinSubstitution([
            TB4_PROJECT,
            'config',
            'localization_timeout.yaml',
        ]),
        description='Localization parameters file'),
    DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            TB4_PROJECT,
            'config',
            'nav2_timeout.yaml',
        ]),
        description='Nav2 parameters file'),

    DeclareLaunchArgument(
        'mission_file',
        default_value=PathJoinSubstitution([
            TB4_PROJECT,
            'config',
            'diem_waypoints_reduced.yaml',
        ]),
        description='Waypoint mission yaml file'),
    DeclareLaunchArgument(
        'route_file',
        default_value=PathJoinSubstitution([
            TB4_PROJECT,
            'config',
            'routes_smac2d_milp_reduced.yaml',
        ]),
        description='Offline route yaml file'),
    DeclareLaunchArgument(
        'visited_waypoints_file',
        default_value='',
        description='Optional waypoint resume yaml file'),
    DeclareLaunchArgument(
        'planner_id',
        default_value='',
        description='Optional planner id override for the search mission'),
    DeclareLaunchArgument(
        'search_behavior_tree',
        default_value=PathJoinSubstitution([
            TB4_PROJECT,
            'behavior_trees',
            'BT_search.xml',
        ]),
        description='Behavior tree used by the search mission'),
    DeclareLaunchArgument(
        'follow_behavior_tree',
        default_value=PathJoinSubstitution([
            TB4_PROJECT,
            'behavior_trees',
            'BT_follow.xml',
        ]),
        description='Behavior tree used by the ArUco follow node'),

    DeclareLaunchArgument('image_topic',
                          default_value='/oakd/rgb/preview/image_raw/compressed',
                          description='RGB image topic used by aruco_reader'),
    DeclareLaunchArgument('compressed_image', default_value='true',
                          choices=['true', 'false'],
                          description='Use sensor_msgs/CompressedImage instead of Image'),
    DeclareLaunchArgument('camera_info_topic',
                          default_value='/oakd/rgb/preview/camera_info',
                          description='CameraInfo topic used for ArUco pose estimation'),
    DeclareLaunchArgument('aruco_pose_topic', default_value='/aruco/pose',
                          description='Output PoseStamped topic for detected marker'),
    DeclareLaunchArgument('camera_frame',
                          default_value='oakd_rgb_camera_optical_frame',
                          description='Frame id used in published marker poses'),
    DeclareLaunchArgument('marker_length', default_value='0.18',
                          description='Marker side length in meters'),
    DeclareLaunchArgument('aruco_id', default_value='372',
                          description='ArUco marker id to track'),
    DeclareLaunchArgument('output_rate_hz', default_value='5.0',
                          description='Maximum valid ArUco pose publish rate in Hz'),

    DeclareLaunchArgument('include_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Launch TurtleBot4 RViz'),
    DeclareLaunchArgument('include_aruco', default_value='true',
                          choices=['true', 'false'],
                          description='Launch aruco_reader'),
    DeclareLaunchArgument('include_follow', default_value='true',
                          choices=['true', 'false'],
                          description='Launch compute_pose and BT_FSM_follow'),
    DeclareLaunchArgument('include_search', default_value='true',
                          choices=['true', 'false'],
                          description='Launch the waypoint search mission'),
]


def _include_tb4_launch(launch_name, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([TB4_PROJECT, 'launch', launch_name]),
        ]),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = _include_tb4_launch(
        'localization_real.launch.py',
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params': LaunchConfiguration('localization_params'),
        },
    )

    nav2 = _include_tb4_launch(
        'nav2_real.launch.py',
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('nav2_params'),
        },
    )

    aruco_reader = _include_tb4_launch(
        'aruco_reader.launch.py',
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'image_topic': LaunchConfiguration('image_topic'),
            'compressed_image': LaunchConfiguration('compressed_image'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'pose_topic': LaunchConfiguration('aruco_pose_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'marker_length': LaunchConfiguration('marker_length'),
            'aruco_id': LaunchConfiguration('aruco_id'),
            'output_rate_hz': LaunchConfiguration('output_rate_hz'),
        },
    )

    follow = _include_tb4_launch(
        'follow.launch.py',
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'behavior_tree': LaunchConfiguration('follow_behavior_tree'),
        },
    )

    search = _include_tb4_launch(
        'real_search_mission.launch.py',
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'mission_file': LaunchConfiguration('mission_file'),
            'route_file': LaunchConfiguration('route_file'),
            'visited_waypoints_file': LaunchConfiguration('visited_waypoints_file'),
            'planner_id': LaunchConfiguration('planner_id'),
            'behavior_tree': LaunchConfiguration('search_behavior_tree'),
            'target_pose_topic': LaunchConfiguration('aruco_pose_topic'),
        },
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'),
                'launch',
                'view_robot.launch.py',
            ]),
        ]),
        condition=IfCondition(LaunchConfiguration('include_rviz')),
    )

    return LaunchDescription(ARGUMENTS + [
        LogInfo(msg='[full_real] Avvio localization.'),
        localization,

        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='[full_real] Avvio RViz.'),
                rviz,
            ],
            condition=IfCondition(LaunchConfiguration('include_rviz')),
        ),

        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='[full_real] Avvio aruco_reader.'),
                aruco_reader,
            ],
            condition=IfCondition(LaunchConfiguration('include_aruco')),
        ),

        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg='[full_real] Avvio Nav2.'),
                nav2,
            ],
        ),

        TimerAction(
            period=11.0,
            actions=[
                LogInfo(msg='[full_real] Avvio pipeline follow ArUco.'),
                follow,
            ],
            condition=IfCondition(LaunchConfiguration('include_follow')),
        ),

        TimerAction(
            period=13.0,
            actions=[
                LogInfo(
                    msg=(
                        '[full_real] Avvio search mission: resta in attesa '
                        'della initial pose.'
                    )
                ),
                search,
            ],
            condition=IfCondition(LaunchConfiguration('include_search')),
        ),

        TimerAction(
            period=16.0,
            actions=[
                LogInfo(
                    msg='[full_real] Se RViz mostra mappa e TF, imposta ora la 2D Pose Estimate. '
                        'La missione partira dopo la posa iniziale.'
                ),
            ],
        ),
    ])
