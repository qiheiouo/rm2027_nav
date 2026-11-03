"""Navigation intent composition only; supplies no hardware or motion authority."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def include(package, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), "launch", filename,
            ])), launch_arguments=arguments.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument("enable_navigation_intents", default_value="false"),
        DeclareLaunchArgument("radar_target_input", default_value="serial_intent",
                              choices=["serial_intent", "track"]),
        DeclareLaunchArgument("mission_config", default_value=PathJoinSubstitution([
            FindPackageShare("rm_competition_mission"), "config", "mission_safe.yaml",
        ])),
        LogInfo(msg="2027 target arbitration candidate: Operator > Radar > Local; Mission disabled at startup; "
                "requires separately authorized Nav2/safety/V2 guard. No hardware is started."),
        include("rm_pursuit", "pursuit.launch.py", {
            "enable_pursuit_boundary": PythonExpression([
                "'", LaunchConfiguration("enable_navigation_intents"),
                "'.lower() in ['true','1','yes','on'] and '",
                LaunchConfiguration("radar_target_input"), "' == 'track'",
            ]),
            "radar_only": "true", "use_mock_target": "false",
        }),
        include("rm_competition_mission", "competition_mission.launch.py", {
            "enable_mission_node": LaunchConfiguration("enable_navigation_intents"),
            "mission_config": LaunchConfiguration("mission_config"),
            "radar_only": "true", "require_motion_guard": "true",
            "radar_target_input": LaunchConfiguration("radar_target_input"),
            "require_chassis_mode": "true", "startup_enabled": "false",
            "use_safety_mock": "false",
        }),
    ])
