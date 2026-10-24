"""Opt-in competition consumer; no mocks, command publishers or authority."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_posture_constraints", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=PathJoinSubstitution([
            FindPackageShare("rm_nav_config"), "config", "nav2_old_car_2026_left_stvl.yaml",
        ])),
        Node(package="rm_posture_motion", executable="posture_profile_selector",
             condition=IfCondition(LaunchConfiguration("enable_posture_constraints")),
             output="screen", parameters=[{
                 "source_mode": "competition_v2",  # Fixed; no test topic/config argument.
                 "baseline_file": LaunchConfiguration("nav2_params"),
                 "profiles_file": PathJoinSubstitution([
                     FindPackageShare("rm_posture_motion"), "config", "profiles.yaml",
                 ]),
             }]),
    ])
