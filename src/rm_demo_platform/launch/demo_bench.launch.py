"""Software fixture only. No driver, physical serial, or motion authorization."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    if LaunchConfiguration("enable_test").perform(context) != "true":
        return [LogInfo(msg="Demo inventory: zero nodes. enable_test:=true runs the isolated software bench only.")]
    share = Path(get_package_share_directory("rm_demo_platform"))
    parameters = str(share / "config" / "bench.yaml")
    duration = float(LaunchConfiguration("gimbal_duration_sec").perform(context))
    return [
        LogInfo(msg="SYNTHETIC GEOMETRY / NO HARDWARE: V2 mock + direct relative yaw + SE(3) adapter. No FAST-LIO or Nav2 acceptance."),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", parameters=[{
                 "robot_description": (share / "urdf" / "bench.urdf").read_text(),
                 "publish_frequency": 100.0, "ignore_timestamp": False,
             }]),
        Node(package="rm_serial_driver", executable="competition_v2_transport_node",
             name="serial_transport_node", parameters=[{
                 "dry_run": True, "motion_test_mode": False,
                 "required_remote_capabilities": 5,
                 "enable_timestamp_provenance": LaunchConfiguration("enable_timestamp_provenance").perform(context) == "true",
                 "provenance_session_id": "demo-software-bench",
             }]),
        Node(package="rm_serial_driver", executable="competition_v2_mock_lower_node",
             name="competition_v2_mock_lower_node", parameters=[{
                 "gimbal_yaw_rad": 0.25, "gimbal_rate_rad_s": 0.0,
                 "gimbal_publish_duration_sec": duration,
                 "publish_operator_target": False,
             }]),
        Node(package="rm_localization_adapters", executable="gimbal_state_adapter",
             name="gimbal_state_adapter", parameters=[{"use_input": True}]),
        Node(package="rm_localization_adapters", executable="map_odom_stub",
             name="map_odom_stub"),
        Node(package="rm_localization_adapters", executable="imu_frame_adapter",
             name="imu_frame_adapter"),
        Node(package="rm_localization_adapters", executable="fake_lio_odom_publisher",
             name="fake_lio_odom_publisher", parameters=[parameters]),
        Node(package="rm_localization_adapters", executable="lio_adapter",
             name="lio_adapter", parameters=[parameters]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_test", default_value="false", choices=["false", "true"]),
        DeclareLaunchArgument("enable_timestamp_provenance", default_value="false", choices=["false", "true"]),
        DeclareLaunchArgument("gimbal_duration_sec", default_value="-1.0",
                              description="Mock source dropout after this duration; negative keeps publishing."),
        OpaqueFunction(function=setup),
    ])
