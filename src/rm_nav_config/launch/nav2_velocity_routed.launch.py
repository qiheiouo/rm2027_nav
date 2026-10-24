"""Humble Nav2 with one final velocity owner for generic/legacy simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        GroupAction(actions=[
            # Node-qualified rule affects behavior only. Controller and smoother
            # retain Humble's upstream input/output mappings without a cycle.
            SetRemap(src="behavior_server:cmd_vel", dst="cmd_vel_nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py",
                ])),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "use_composition": "False",
                }.items(),
            ),
        ]),
    ])
