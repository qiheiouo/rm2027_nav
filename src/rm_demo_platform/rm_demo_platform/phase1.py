"""Phase 1 manifests and recording preparation; no device access or motion API."""
import argparse
from datetime import datetime, timezone
import ipaddress
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import uuid

import yaml


class NotReady(ValueError):
    pass


SERIAL_PARAMETERS = ('device', 'baudrate', 'required_remote_capabilities',
    'chassis_rate_hz', 'posture_rate_hz', 'heartbeat_rate_hz', 'read_poll_rate_hz',
    'health_rate_hz', 'cmd_vel_timeout_sec', 'connection_timeout_sec', 'gimbal_timeout_sec',
    'chassis_heading_timeout_sec', 'posture_timeout_sec', 'max_vx', 'max_vy', 'max_wz')


def read(path):
    with Path(path).open() as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise NotReady(f'{path}: expected mapping')
    return value


def required(data, key):
    value = data.get(key)
    if value is None or value == '' or value == 'TODO':
        raise NotReady(f'required: {key}')
    return value


def required_text(data, key):
    value = required(data, key)
    if not isinstance(value, str) or not value.strip():
        raise NotReady(f'{key}: nonempty text required')
    return value


def confirmed(data, key):
    if data.get(key) is not True:
        raise NotReady(f'not confirmed: {key}')


def positive(data, key):
    value = required(data, key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise NotReady(f'{key}: expected finite positive number')
    return float(value)


def existing_file(data, key):
    path = Path(required(data, key))
    if not path.is_absolute() or not path.is_file():
        raise NotReady(f'{key}: absolute existing file required')
    return path


def validate(config_dir, stage):
    """Validate only a requested acquisition stage, before creating any ROS nodes."""
    root = Path(config_dir)
    if stage == 'inventory':
        return {}
    if stage not in ('serial', 'lidar', 'tf', 'fixed_lio'):
        raise NotReady('supported stages: inventory, serial, lidar, tf, fixed_lio; no motion stage')
    result = {}
    if stage in ('serial', 'tf', 'fixed_lio'):
        serial = read(root / 'demo_serial.yaml')
        if serial.get('profile') != 'competition_v2':
            raise NotReady('firmware must explicitly implement competition_v2; no profile guessing')
        for key in ('firmware_repository', 'firmware_revision'):
            required_text(serial, key)
        for key in ('wire_contract_confirmed', 'physical_motion_inhibit_confirmed', 'isolated_ros_domain_confirmed'):
            confirmed(serial, key)
        params = serial['parameters'].copy()
        if set(params) != set(SERIAL_PARAMETERS):
            raise NotReady('serial parameters must exactly match the manifest; no implicit node defaults/overrides')
        device = required(params, 'device')
        if not isinstance(device, str) or not device.startswith('/dev/'):
            raise NotReady('device: explicit /dev path required')
        # Constants here are supported transport encodings, not measured hardware values.
        if params.get('baudrate') not in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600):
            raise NotReady('baudrate: select a supported, firmware-confirmed baud rate')
        cap = required(params, 'required_remote_capabilities')
        if type(cap) is not int or cap < 0 or cap > 0xffffffff or cap & 37 != 37:
            raise NotReady('required capabilities must include chassis(1), relative gimbal(4), heading(32)')
        for key in params:
            if key not in ('device', 'baudrate', 'required_remote_capabilities'):
                params[key] = positive(params, key)
        result['serial'] = serial
        result['serial_params'] = params
    if stage in ('lidar', 'fixed_lio'):
        lidar = read(root / 'demo_mid360.yaml')
        for key in ('device_identity', 'firmware_revision'):
            required_text(lidar, key)
        confirmed(lidar, 'sdk_config_reviewed')
        positive(lidar, 'publish_freq_hz')
        for key in ('host_ip', 'lidar_ip'):
            addr = ipaddress.IPv4Address(required(lidar, key))
            if addr.is_unspecified or addr.is_loopback or addr.is_multicast:
                raise NotReady(f'{key}: real unicast address required')
        sdk = json.loads(existing_file(lidar, 'sdk_config_path').read_text())
        devices = sdk.get('lidar_configs', [])
        if len(devices) != 1 or devices[0].get('ip') != lidar['lidar_ip']:
            raise NotReady('SDK JSON must contain exactly the selected MID360 IP')
        host = sdk['MID360']['host_net_info']
        if any(host.get(k) != lidar['host_ip'] for k in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')):
            raise NotReady('SDK JSON host IPs disagree with manifest')
        # Raw clouds must remain in sensor frame; apply measured geometry in TF once.
        extrinsic = devices[0].get('extrinsic_parameter', {})
        if any(extrinsic.get(k) != 0 for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')):
            raise NotReady('SDK extrinsic must explicitly be identity; mechanical extrinsics belong to TF')
        result['lidar'] = lidar
    if stage in ('tf', 'fixed_lio'):
        cal = read(root / 'demo_calibration.yaml')
        confirmed(cal, 'measured')
        required(cal, 'revision')
        for edge in ('base_to_yaw_axis', 'gimbal_to_mid360', 'mid360_to_imu'):
            for key in ('xyz_m', 'rpy_rad'):
                vec = cal[edge].get(key)
                if not isinstance(vec, list) or len(vec) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in vec):
                    raise NotReady(f'{edge}.{key}: three measured finite values required')
        positive(result['serial'], 'gimbal_joint_publish_rate_hz')
        result['calibration'] = cal
    if stage == 'fixed_lio':
        lio = read(root / 'demo_lio.yaml')
        for key in ('enabled', 'mechanically_locked_gimbal_confirmed', 'raw_epoch_and_frame_reviewed', 'internal_imu_extrinsics_reviewed'):
            confirmed(lio, key)
        existing_file(lio, 'phase1b_evidence')
        confirmed(result['lidar'], 'raw_imu_frame_lineage_confirmed')
        backend = read(existing_file(lio, 'backend_config_path'))['/**']['ros__parameters']
        if backend['mapping'].get('zero_start_pose') is not False:
            raise NotReady('disable delayed backend zero-start; do not introduce a hidden raw epoch jump')
        if backend['common'].get('multi_lidar') is not False:
            raise NotReady('single physical MID360 required')
        for key in ('extrinsic_T', 'extrinsic_R'):
            values = backend['mapping'].get(key)
            if not isinstance(values, list) or len(values) != (3 if key.endswith('T') else 9) or any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
                raise NotReady(f'measured backend mapping.{key} required')
        adapter = read(existing_file(lio, 'adapter_config_path'))['lio_adapter']['ros__parameters']
        expected = dict(pose_conversion_mode='sensor_tf', use_tf_sensor_to_base=True,
                        use_latest_transform=False, allow_placeholder_fallback=False,
                        input_sensor_frame='lio_imu_link', base_frame='base_link',
                        odom_frame='odom', publish_tf=True, raw_odom_topic='/odometry/fast_lio_raw',
                        output_odom_topic='/odometry/lio', backend_child_frame_alias_enabled=True,
                        backend_child_frame_alias_source='body', backend_child_frame_alias_target='lio_imu_link')
        for key, value in expected.items():
            if adapter.get(key) != value:
                raise NotReady(f'lio_adapter.{key} must be {value!r}')
        if adapter.get('raw_odom_parent_frame_mode') not in ('canonical_odom', 'sensor_initial'):
            raise NotReady('explicit raw_odom_parent_frame_mode required; review tilted startup frame evidence')
        if adapter['raw_odom_parent_frame_mode'] == 'sensor_initial' and adapter.get('sensor_initial_reference_mode') != 'first_source_sample':
            raise NotReady('sensor_initial requires first_source_sample; no implicit fixed mount')
        result['lio'] = lio
    return result


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()


def prepare(repo, config_dir, output, operator, phase, acq_dir=None):
    repo, config_dir, output = Path(repo).resolve(), Path(config_dir).resolve(), Path(output).resolve()
    acq = Path(acq_dir) if acq_dir else repo / 'src/rm_navigation_bringup/config/localization_acquisition'
    # Reuse the ACQ schema, topic/type whitelist and QoS; Demo adds its own section.
    metadata = read(acq / 'session.template.yaml')
    demo = read(config_dir / 'demo_session.yaml')
    if demo.get('motion_authorization') is not False:
        raise NotReady('this preparation tool never accepts motion authorization')
    output.mkdir(parents=True, exist_ok=False)
    snapshot = output / 'config'
    snapshot.mkdir()
    for path in config_dir.glob('demo_*.yaml'):
        shutil.copy2(path, snapshot / path.name)
    # Freeze external configuration inputs alongside the manifest, not just a path.
    for name, keys in [('demo_mid360.yaml', ['sdk_config_path']), ('demo_lio.yaml', ['backend_config_path', 'adapter_config_path', 'phase1b_evidence'])]:
        item = read(snapshot / name)
        for key in keys:
            if item.get(key):
                source = existing_file(item, key)
                target = snapshot / f'{key}{source.suffix}'
                shutil.copy2(source, target)
                item[key] = str(target)
        (snapshot / name).write_text(yaml.safe_dump(item, sort_keys=False, allow_unicode=True))
    session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:12]
    metadata.update(session_id=session_id, role='demo_phase1', bag_directory=str(output / 'bag'))
    metadata['software'].update(branch=git(repo, 'branch', '--show-current'),
                                rm_instrumentation_revision=git(repo, 'rev-parse', 'HEAD'),
                                submodules=git(repo, 'submodule', 'status'),
                                dirty_status=git(repo, 'status', '--short'),
                                livox_revision=git(repo / 'src/livox_ros_driver2_humble', 'rev-parse', 'HEAD'))
    # --binary preserves tracked staged+unstaged contents against HEAD. Untracked names are listed only.
    (output / 'dirty.patch').write_text(git(repo, 'diff', '--binary', 'HEAD') + '\n')
    subdiff = git(repo, 'submodule', 'foreach', '--quiet', 'git diff --binary HEAD')
    (output / 'submodule-dirty.patch').write_text(subdiff + '\n')
    serial, lidar, cal = (read(snapshot / name) for name in ('demo_serial.yaml', 'demo_mid360.yaml', 'demo_calibration.yaml'))
    metadata['devices']['serial'].update(profile=serial['profile'], device=serial['parameters']['device'],
        baudrate=serial['parameters']['baudrate'], mcu_firmware=serial['firmware_revision'],
        required_capabilities=serial['parameters']['required_remote_capabilities'])
    metadata['devices']['primary_mid360'].update(identity=lidar['device_identity'], firmware=lidar['firmware_revision'])
    demo.update(operator=operator, phase=phase, created_utc=datetime.now(timezone.utc).isoformat(),
                mechanical_calibration_revision=cal['revision'], result='NOT_RUN')
    demo['capture_environment'] = dict(hostname=platform.node(), platform=platform.platform(), kernel=platform.release())
    metadata['demo'] = demo
    metadata['runtime']['config_snapshots'] = sorted(str(p) for p in snapshot.iterdir())
    metadata['runtime']['actual_launch_commands'] = demo['launch_commands']
    metadata['runtime']['effective_parameters'] = 'PENDING: record ros2 param dump for every started node'
    topics = read(acq / 'topics.yaml')
    topics['required']['/motion/v2_guard/state'] = 'rm_competition_interfaces/msg/V2MotionGuardState'
    if phase in 'CDEF':
        topics['required'].update({'/livox/lio_imu': 'sensor_msgs/msg/Imu', '/odometry/fast_lio_raw':'nav_msgs/msg/Odometry', '/odometry/lio':'nav_msgs/msg/Odometry'})
    attitude = demo['chassis_attitude']
    if attitude.get('topic') and attitude.get('type') and attitude.get('contract_revision'):
        topics['required'][attitude['topic']] = attitude['type']
    else:
        demo['chassis_attitude_status'] = 'BLOCKED_PENDING_STM_CONTRACT'
    (output / 'session.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True))
    (output / 'topics.yaml').write_text(yaml.safe_dump(topics, sort_keys=False))
    shutil.copy2(acq / 'qos.yaml', output / 'qos.yaml')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('preflight')
    check.add_argument('--config-dir', required=True)
    check.add_argument('--stage', default='inventory', choices=['inventory', 'serial', 'lidar', 'tf', 'fixed_lio'])
    new = sub.add_parser('prepare')
    for name in ('repo', 'config-dir', 'output', 'operator'):
        new.add_argument('--' + name, required=True)
    new.add_argument('--phase', choices=list('ABCDEF'), default='A')
    new.add_argument('--acq-dir')
    args = parser.parse_args()
    try:
        if args.command == 'preflight':
            validate(args.config_dir, args.stage)
            print(f'CONFIG_READY: {args.stage}; NOT hardware acceptance')
        else:
            print(prepare(args.repo, args.config_dir, args.output, args.operator, args.phase, args.acq_dir))
    except (NotReady, KeyError, ValueError, OSError, yaml.YAMLError) as error:
        parser.exit(2, f'NOT_READY: {error}\n')
