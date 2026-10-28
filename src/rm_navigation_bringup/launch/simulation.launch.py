"""Supported Jazzy/Harmonic simulation; one Nav2 chain and no hardware IO."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def require_jazzy(context):
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise RuntimeError('Unified simulation requires ROS 2 Jazzy / Gazebo Harmonic; see docs/operations/simulation.md')
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('localization_source', default_value='truth', choices=['truth', 'lio_health_fixture']),
        DeclareLaunchArgument('navigation_policy', default_value='baseline', choices=['baseline', 'event_driven']),
        DeclareLaunchArgument('output', default_value='/tmp/rm2027_simulation'),
    ]
    return LaunchDescription(arguments + [
        OpaqueFunction(function=require_jazzy),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rm_new_car_simulation'), 'launch', 'baseline.launch.py',
            ])),
            launch_arguments={
                'field': 'rules2026_course',
                'spawn': '-3,-3,.002,0,0,0',
                'use_nav2': 'true',
                'publish_truth_tf': 'true',
                **{name: LaunchConfiguration(name) for name in ('headless', 'use_rviz', 'sensors', 'output', 'navigation_policy', 'localization_source')},
            }.items(),
        ),
    ])
