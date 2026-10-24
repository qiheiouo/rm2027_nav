from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _gimbal_node(context):
    # An empty CLI value preserves the selected YAML; explicit CLI wins.
    overrides = {"use_sim_time": ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)}
    use_input = LaunchConfiguration("gimbal_use_input").perform(context).strip()
    topic = LaunchConfiguration("gimbal_input_topic").perform(context).strip()
    if use_input:
        if use_input.lower() not in {"true", "false"}:
            raise RuntimeError("gimbal_use_input must be empty, true or false")
        overrides["use_input"] = use_input.lower() == "true"
    if topic:
        overrides["input_topic"] = topic
    return [Node(
        package="rm_localization_adapters", executable="gimbal_state_adapter",
        name="gimbal_state_adapter", output="screen",
        parameters=[LaunchConfiguration("gimbal_state_adapter_config"), overrides],
    )]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_map_odom_stub = LaunchConfiguration("use_map_odom_stub")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    default_lio_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "lio_adapter.yaml",
    ])
    lio_adapter_config = LaunchConfiguration("lio_adapter_config")
    default_gimbal_state_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "gimbal_state_adapter.yaml",
    ])
    gimbal_state_adapter_config = LaunchConfiguration("gimbal_state_adapter_config")
    imu_frame_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "imu_frame_adapter.yaml",
    ])

    # Phase 1 skeleton only. FAST-LIO is intentionally not launched here.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_map_odom_stub", default_value="true"),
        DeclareLaunchArgument(
            "raw_odom_topic",
            default_value="/odometry/fast_lio_raw",
        ),
        DeclareLaunchArgument(
            "lio_adapter_config",
            default_value=default_lio_adapter_config,
        ),
        DeclareLaunchArgument(
            "gimbal_state_adapter_config",
            default_value=default_gimbal_state_adapter_config,
        ),
        DeclareLaunchArgument("gimbal_use_input", default_value=""),
        DeclareLaunchArgument("gimbal_input_topic", default_value=""),
        Node(
            condition=IfCondition(use_map_odom_stub),
            package="rm_localization_adapters",
            executable="map_odom_stub",
            name="map_odom_stub",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rm_localization_adapters",
            executable="lio_adapter",
            name="lio_adapter",
            output="screen",
            parameters=[
                lio_adapter_config,
                {
                    "use_sim_time": use_sim_time,
                    "raw_odom_topic": raw_odom_topic,
                },
            ],
        ),
        OpaqueFunction(function=_gimbal_node),
        Node(
            package="rm_localization_adapters",
            executable="imu_frame_adapter",
            name="imu_frame_adapter",
            output="screen",
            parameters=[
                imu_frame_adapter_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
