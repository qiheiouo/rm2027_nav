"""Opt-in localization boundary. Native epoch changes require group restart."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_spec = importlib.util.spec_from_file_location('rm_zlocalization_policy', Path(__file__).with_name('runtime_policy.py'))
policy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(policy)


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    # A disabled entry must work even when the optional backend is not installed.
    enabled = value('enabled').lower()
    if enabled not in ('true', 'false'):
        raise ValueError('enabled must be true or false')
    if enabled == 'false':
        return []
    upstream = policy.upstream_module(share('backend'))
    flag = lambda name: upstream.launch_bool(value(name))
    native = value('frontend') == 'zlio'
    if native and flag('start_fastlio'):
        raise ValueError('native ZLIO and FAST-LIO are mutually exclusive')
    parameters = policy.backend_parameters(
        share('backend'), value('config_file'), mode=value('mode'), prior_map=value('prior_map'),
        frontend=value('frontend'), descriptor=value('descriptor'), loop=flag('enable_loop'),
        visualization=flag('visualization') or flag('rviz'), world_cloud=flag('world_cloud'))
    sim = flag('use_sim_time')
    actions = [
        Node(package='backend', executable='backend_node', name='zlocalization_backend',
             parameters=[parameters, {'use_sim_time': sim}], output='screen',
             remappings=[('/tf', '/zlocalization/_quarantine/tf'),
                         ('/tf_static', '/zlocalization/_quarantine/tf_static'),
                         ('~/localization_pose', '/zlocalization/sensor_pose'),
                         ('~/localization_state', '/zlocalization/state'),
                         ('/localization/odometry', '/zlocalization/sensor_odometry')]),
        Node(package='rm_zlocalization_candidate', executable='candidate_adapter.py',
             parameters=[{'use_sim_time': sim, 'frontend': value('frontend'),
                          'raw_frame': 'zlio_odom' if native else 'odom',
                          'raw_child': 'lio_imu_link' if native else 'body'}], output='screen'),
        Node(package='rm_relocalization_bridge', executable='map_odom_from_global_pose',
             parameters=[{'use_sim_time': sim,
                          'upstream_valid_topic': '/localization/zlocalization_backend_valid',
                          'max_pose_odom_dt_sec': .001,
                          'max_global_pose_age_sec': .5}], output='screen'),
    ]
    if flag('diagnostics'):
        actions.append(Node(package='rm_zlocalization_candidate', executable='runtime_diagnostics.py',
            parameters=[{'use_sim_time': sim}], output='screen'))
    if flag('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='zlocalization_rviz',
            parameters=[{'use_sim_time': sim}],
            arguments=['-d', str(Path(share('backend')) / 'rviz/backend.rviz')],
            remappings=policy.rviz_remappings('/zlocalization', 'zlocalization_backend', value('mode'))))
    if flag('start_lio_adapter'):
        actions.append(Node(package='rm_localization_adapters', executable='lio_adapter',
            parameters=[str(Path(share('rm_localization_adapters')) / 'config/lio_adapter.yaml'),
                {'use_sim_time': sim, 'backend_child_frame_alias_enabled': True,
                 'expected_input_odom_frame': 'zlio_odom' if native else 'odom',
                 'raw_odom_parent_frame_mode': 'sensor_initial',
                 'sensor_initial_reference_mode': 'first_source_sample',
                 'twist_mode': 'finite_difference'}],
            remappings=[('/odometry/fast_lio_raw', '/zlocalization/zlio_raw')] if native else [],
            output='screen'))
    if native and flag('start_zlio'):
        native_params = policy.native_parameters(value('frontend_config'), value('imu_acceleration_scale'),
            input_type=value('input_type'), lidar_topic=value('lidar_topic'), imu_topic=value('imu_topic'),
            use_sim_time=sim, wait_for_start=flag('wait_for_start'),
            point_time_field=value('point_time_field'), point_time_scale=value('point_time_scale'),
            point_time_absolute=flag('point_time_absolute'), livox_use_timebase=flag('livox_use_timebase'))
        actions.append(Node(package='frontend', executable='zlio_node', parameters=[native_params],
            remappings=policy.native_remappings('/zlocalization'), output='screen'))
    if flag('start_fastlio'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('rm_lio_bringup')) / 'launch/fast_lio_multi_phase2a.launch.py')),
            launch_arguments={'use_backend': 'true', 'scan_publish_en': 'true',
                'zero_start_pose': 'false', 'update_method': 'bundle',
                'use_sim_time': value('use_sim_time')}.items()))
    return actions


def generate_launch_description():
    defaults = {
        'enabled': 'false',
        'config_file': str(Path(share('rm_zlocalization_candidate')) / 'config/z_localization_candidate.yaml'),
        'prior_map': '', 'mode': 'localization', 'use_sim_time': 'false',
        'start_lio_adapter': 'true', 'start_fastlio': 'false', 'frontend': 'legacy', 'start_zlio': 'true',
        'frontend_config': '', 'imu_acceleration_scale': '', 'input_type': 'livox',
        'lidar_topic': '/livox/left/lidar', 'imu_topic': '/livox/lio_imu',
        'wait_for_start': 'false', 'point_time_field': 'offset_time', 'point_time_scale': '1e-9',
        'point_time_absolute': 'false', 'livox_use_timebase': 'true',
        'descriptor': 'scan_context', 'enable_loop': 'false', 'visualization': 'false',
        'world_cloud': 'false', 'diagnostics': 'true', 'rviz': 'false',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
