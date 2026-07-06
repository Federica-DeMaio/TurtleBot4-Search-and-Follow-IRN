# Copyright 2026
#
# Start the Nav2 navigation stack on the real robot.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation clock'),
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('tb4_project'),
            'config',
            'nav2_timeout.yaml'
        ]),
        description='Nav2 parameters file'),
]


def launch_setup(context, *args, **kwargs):
    pkg_tb4_project = get_package_share_directory('tb4_project')

    nav2_launch = PathJoinSubstitution([
        pkg_tb4_project,
        'launch',
        'nav2_monitored_navigation.launch.py'
    ])

    namespace = LaunchConfiguration('namespace')
    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith('/'):
        namespace_str = '/' + namespace_str

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('namespace', namespace_str),
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('params_file', LaunchConfiguration('params_file')),
                ('use_composition', 'False'),
            ]
        ),
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
