# Copyright 2026
#
# Start map-based localization on the real robot.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


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
    DeclareLaunchArgument('map', default_value=_default_diem_map(),
                          description='Full path to the map yaml file'),
    DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('tb4_project'),
            'config',
            'localization_timeout.yaml'
        ]),
        description='Localization parameters file'),
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')

    localization_launch = PathJoinSubstitution([
        pkg_turtlebot4_navigation,
        'launch',
        'localization.launch.py'
    ])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', LaunchConfiguration('map')),
            ('params', LaunchConfiguration('params')),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization)
    return ld
