"""Run the native upstream estimator without canonical TF or status output."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_spec = importlib.util.spec_from_file_location('rm_zlocalization_policy', Path(__file__).with_name('runtime_policy.py'))
policy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(policy)


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('enabled').lower() == 'false':
        return []
    if value('enabled').lower() != 'true':
        raise ValueError('enabled must be true or false')
    upstream = policy.upstream_module(share('backend'))
    flag = lambda name: upstream.launch_bool(value(name))
    sim = flag('use_sim_time')
    params = policy.backend_parameters(share('backend'), value('config_file'), mode=value('mode'),
        prior_map=value('prior_map'), frontend='zlio', descriptor=value('descriptor'),
        loop=flag('enable_loop'), visualization=flag('visualization') or flag('rviz'),
        world_cloud=flag('world_cloud'), shadow=True)
    native = policy.native_parameters(value('frontend_config'), value('imu_acceleration_scale'),
        input_type=value('input_type'), lidar_topic=value('lidar_topic'), imu_topic=value('imu_topic'),
        use_sim_time=sim, wait_for_start=flag('wait_for_start'),
        point_time_field=value('point_time_field'), point_time_scale=value('point_time_scale'),
        point_time_absolute=flag('point_time_absolute'), livox_use_timebase=flag('livox_use_timebase'))
    prefix = '/zlocalization/shadow'
    actions = [
        Node(package='frontend', executable='zlio_node', name='zlio_shadow', parameters=[native],
             remappings=policy.native_remappings(prefix), output='screen'),
        Node(package='backend', executable='backend_node', name='zlocalization_shadow_backend',
             parameters=[params, {'use_sim_time': sim}], output='screen',
             remappings=[('/tf', prefix + '/_quarantine/tf'),
                         ('/tf_static', prefix + '/_quarantine/tf_static'),
                         ('~/localization_pose', prefix + '/sensor_pose'),
                         ('~/localization_state', prefix + '/state'),
                         ('/localization/odometry', prefix + '/sensor_odometry')]),
        Node(package='rm_zlocalization_candidate', executable='runtime_diagnostics.py',
             name='zlio_shadow_diagnostics', parameters=[{'use_sim_time': sim,
                 'state_topic': prefix + '/state', 'diagnostics_topic': prefix + '/diagnostics',
                 'adapter_status_topic': ''}], output='screen'),
    ]
    if flag('visualization'):
        actions.append(Node(package='frontend', executable='zlio_visualizer',
            namespace='/zlocalization/shadow', parameters=[{'use_sim_time': sim}], output='screen'))
    if flag('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='zlio_shadow_rviz',
            parameters=[{'use_sim_time': sim}],
            arguments=['-d', str(Path(share('backend')) / 'rviz/backend.rviz')],
            remappings=policy.rviz_remappings(prefix, 'zlocalization_shadow_backend', value('mode'))))
    return actions


def generate_launch_description():
    defaults = {
        'enabled': 'false', 'frontend_config': '', 'imu_acceleration_scale': '',
        'config_file': str(Path(share('rm_zlocalization_candidate')) / 'config/z_localization_candidate.yaml'),
        'mode': 'mapping', 'prior_map': '', 'use_sim_time': 'true', 'input_type': 'livox',
        'lidar_topic': '/livox/left/lidar', 'imu_topic': '/livox/lio_imu',
        'wait_for_start': 'false', 'point_time_field': 'offset_time', 'point_time_scale': '1e-9',
        'point_time_absolute': 'false', 'livox_use_timebase': 'true',
        'descriptor': 'scan_context', 'enable_loop': 'false', 'visualization': 'false',
        'world_cloud': 'false', 'rviz': 'false',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
