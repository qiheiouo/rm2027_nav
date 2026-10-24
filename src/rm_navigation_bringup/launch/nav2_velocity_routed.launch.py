"""Compatibility entry for the shared Nav2 velocity route."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rm_nav_config"), "launch", "nav2_velocity_routed.launch.py",
            ])), launch_arguments={name: LaunchConfiguration(name)
                                  for name in ("params_file", "use_sim_time", "autostart")}.items(),
        ),
    ])
