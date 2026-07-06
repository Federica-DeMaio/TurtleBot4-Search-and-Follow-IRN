# Copyright 2026
#
# Launch the ArUco follow pipeline from irn_g01.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation clock'),
    DeclareLaunchArgument(
        'behavior_tree',
        default_value=PathJoinSubstitution([
            FindPackageShare('tb4_project'),
            'behavior_trees',
            'BT_follow.xml',
        ]),
        description='Behavior tree used by bt_navigator for ArUco follow goals'),
]


def generate_launch_description():
    compute_pose = Node(
        package='irn_g01',
        executable='compute_pose',
        name='compute_pose',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    follower = Node(
        package='irn_g01',
        executable='BT_FSM_follow',
        name='predictive_follower_fsm',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'behavior_tree': ParameterValue(
                LaunchConfiguration('behavior_tree'), value_type=str),
        }],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(compute_pose)
    ld.add_action(follower)
    return ld
