"""Shared, testable constraints for the isolated upstream entry points."""
import importlib.util
import math
from pathlib import Path


def upstream_module(backend_share):
    spec = importlib.util.spec_from_file_location(
        'pinned_zlocalization_launch', Path(backend_share) / 'launch/backend.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def backend_parameters(backend_share, overlay, *, mode, prior_map='', frontend='legacy',
                       descriptor='scan_context', loop=False, visualization=False,
                       world_cloud=False, shadow=False):
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    if frontend not in ('legacy', 'zlio') or descriptor not in ('scan_context', 'gprobe'):
        raise ValueError('unsupported frontend or descriptor')
    if loop and mode == 'localization':
        raise ValueError('loop closure requires mapping; immutable prior owns global correction')
    upstream = upstream_module(backend_share)
    params = upstream.load_params(Path(backend_share) / 'config/backend.yaml')
    upstream.merge(params, upstream.load_params(overlay))
    # These are interface constraints, applied after the user overlay. Algorithm
    # parameters retain upstream semantics; a YAML cannot create a second owner.
    common = params.setdefault('common', {})
    common.update(frontend=frontend, descriptor_type=descriptor, map_frame='map',
                  odom_frame='zlio_odom' if frontend == 'zlio' else 'fast_lio_odom',
                  sensor_frame='lio_imu_link', input_cloud_in_local_frame=frontend == 'zlio')
    # The upstream strict reader declares only the selected descriptor schema.
    # The example config contains SC keys, which are invalid when choosing GP.
    common.pop('scan_context' if descriptor == 'gprobe' else 'gprobe', None)
    common['body'] = {'frame': 'lio_imu_link',
                      'rotation': dict(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
                      'offset': dict(x=0.0, y=0.0, z=0.0)}
    prefix = '/zlocalization/shadow' if shadow else '/zlocalization'
    # Only synchronized scan-end estimates enter the RM boundary. In particular,
    # native predictions must not race an unversioned scan-end pose/state pair.
    common['odom_topic'] = prefix + '/unused_prediction'
    common['prediction_topic'] = prefix + ('/frontend/odometry' if shadow else '/unused_native_prediction')
    common['sync_odom_topic'] = '/zlocalization/fastlio_odom'
    common['cloud_topic'] = '/zlocalization/cloud_registered'
    common['frame_topic'] = prefix + '/frontend/frame'
    common['session_topic'] = prefix + '/frontend/session'
    common['odom_cloud_sync'] = {'max_interval': 0.001}
    params['uwb']['enabled'] = False  # No RM UWB producer/frame/timing contract.
    params['loop']['enabled'] = bool(loop)
    params['prior_map']['enabled'] = mode == 'localization'
    if mode == 'localization':
        path = prior_map or params['prior_map'].get('directory', '')
        if not path or not Path(path).is_absolute() or not Path(path).is_dir():
            raise ValueError('localization requires an existing absolute prior-map bundle')
        params['prior_map']['directory'] = path
    params['trajectory']['save_on_shutdown'] = False
    params['services'] = {name: prefix + '/' + name for name in ('save_map', 'relocalize')}
    params['services']['reset'] = prefix + '/upstream_reset_request'
    params['reset']['service'] = (prefix + '/reset_internal' if frontend == 'zlio'
                                else '/zlocalization/unavailable_frontend_reset')
    params['output']['legacy'].update(enabled=False, publish_tf=False)
    params['output']['world_cloud'].update(enabled=bool(world_cloud), topic=prefix + '/world_cloud')
    params['visualization'].update(
        enabled=bool(visualization), publish_prior_map=bool(visualization and mode == 'localization'),
        publish_runtime_map=bool(visualization and mode == 'mapping'),
        publish_map_axes=bool(visualization), publish_paths=bool(visualization),
        publish_debug_registration=bool(visualization))
    # Flattened ROS keys could otherwise override nested ownership constraints.
    if any('.' in key for key in params):
        raise ValueError('RM overlays require nested YAML keys, not dotted ROS parameter keys')
    return params


def native_parameters(config, acceleration_scale, *, input_type='livox',
                      lidar_topic='/livox/left/lidar', imu_topic='/livox/lio_imu',
                      use_sim_time=False, wait_for_start=False, point_time_field='offset_time',
                      point_time_scale='1e-9', point_time_absolute=False, livox_use_timebase=True):
    if not config or not Path(config).is_absolute() or not Path(config).is_file():
        raise ValueError('ZLIO requires an explicit absolute frontend_config file')
    scale = float(acceleration_scale)
    timing = float(point_time_scale)
    if not math.isfinite(scale) or scale <= 0 or not math.isfinite(timing) or timing <= 0:
        raise ValueError('IMU acceleration and point-time scales must be finite and positive')
    if input_type not in ('livox', 'pointcloud2'):
        raise ValueError('input_type must be livox or pointcloud2')
    return dict(config_file=config, world_frame='zlio_odom', body_frame='lio_imu_link',
                input_type=input_type, lidar_topic=lidar_topic, imu_topic=imu_topic,
                imu_acceleration_scale=scale, publish_tf=False, coordinated_reset=True,
                reset_mode='warm', use_sim_time=use_sim_time, wait_for_start=wait_for_start,
                point_time_field=point_time_field, point_time_scale=timing,
                point_time_absolute=point_time_absolute, livox_use_timebase=livox_use_timebase)


def native_remappings(prefix):
    return [(name, prefix + '/' + name) for name in
            ('frontend/frame', 'frontend/odometry', 'frontend/session', 'odometry', 'lidar_odometry')
            ] + [('/localization_start', prefix + '/start'), ('stop', prefix + '/stop'),
                 ('/zlio/reset_internal', prefix + '/reset_internal'),
                 ('/tf', prefix + '/_quarantine/tf'), ('/tf_static', prefix + '/_quarantine/tf_static')]


def rviz_remappings(prefix, backend_name, mode):
    topics = ('map_axes', 'localization_status', 'uwb_position', 'optimized_path',
              'odom_path', 'loop_edges', 'debug/target', 'debug/aligned_source')
    return [('/backend_node/' + name, '/' + backend_name + '/' + name) for name in topics] + [
        ('/backend_node/localization_pose', prefix + '/sensor_pose'),
        ('/backend_node/rviz_map', '/' + backend_name +
         ('/prior_map' if mode == 'localization' else '/runtime_map')),
        ('/localization/lio_world_cloud', prefix + '/world_cloud'),
        ('/tf', prefix + '/_quarantine/tf'), ('/tf_static', prefix + '/_quarantine/tf_static')]
