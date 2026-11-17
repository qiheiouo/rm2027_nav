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
    'chassis_heading_timeout_sec', 'chassis_attitude_timeout_sec', 'posture_timeout_sec',
    'max_vx', 'max_vy', 'max_wz')

RADAR_ONLY_PROFILE = 'radar_only_fixed_gimbal'
STM_PROFILE = 'competition_v2_dynamic_gimbal'
SUPPORTED_STAGES = ('inventory', 'serial', 'lidar', 'tf', 'fixed_lio', 'radar_fixed_lio')


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


def finite_vector(data, edge, key):
    value = data[edge].get(key)
    if (not isinstance(value, list) or len(value) != 3 or
            any(type(item) not in (int, float) or not math.isfinite(item) for item in value)):
        raise NotReady(f'{edge}.{key}: three finite SI values required')
    return [float(item) for item in value]


def rpy_to_matrix(rpy):
    """REP-103 fixed-axis roll/pitch/yaw to a row-major rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]


def quaternion_from_rpy(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def locked_gimbal_transforms(calibration, locked_angle, physical_installation='LEFT'):
    """Return the three static edges for the mechanically locked profile.

    The first rotation is the real SE(3) composition
    R(base<-axis) * Rz(locked_angle), not an RPY yaw addition.
    """
    base_q = multiply_quaternions(
        quaternion_from_rpy(finite_vector(calibration, 'base_to_yaw_axis', 'rpy_rad')),
        quaternion_from_rpy([0.0, 0.0, float(locked_angle)]),
    )
    result = [dict(parent='base_link', child='gimbal_yaw_link',
                   xyz=finite_vector(calibration, 'base_to_yaw_axis', 'xyz_m'), quaternion=base_q)]
    mount_edge = ('gimbal_to_mid360_left_cad_nominal'
                  if physical_installation == 'LEFT' else
                  'gimbal_to_mid360_right_cad_nominal')
    for edge_name, parent, child in (
            (mount_edge, 'gimbal_yaw_link', 'mid360_left_frame'),
            ('mid360_to_imu', 'mid360_left_frame', 'lio_imu_link')):
        result.append(dict(parent=parent, child=child,
                           xyz=finite_vector(calibration, edge_name, 'xyz_m'),
                           quaternion=quaternion_from_rpy(
                               finite_vector(calibration, edge_name, 'rpy_rad'))))
    return result


def _validate_calibration(root, radar_only=False):
    cal = read(root / 'demo_calibration.yaml')
    required_text(cal, 'revision')
    if radar_only:
        for edge in ('base_to_yaw_axis', 'gimbal_to_mid360_left_cad_nominal',
                     'gimbal_to_mid360_right_cad_nominal'):
            finite_vector(cal, edge, 'xyz_m')
            finite_vector(cal, edge, 'rpy_rad')
            confirmed(cal[edge], 'confirmed_for_phase1')
            if cal[edge].get('confidence') not in (
                    'CAD_NOMINAL', 'CAD_NOMINAL_RVIZ_VALIDATED', 'MEASURED', 'CALIBRATED'):
                raise NotReady(f'{edge}.confidence: explicit CAD/measured status required')
        # The external CAD nominal does not define the MID360 internal IMU transform.
        # Do not silently turn the driver's frame-id rewrite into an identity calibration.
        edge = cal['mid360_to_imu']
        if edge.get('confidence') == 'PENDING_INTERNAL_IMU_EXTRINSIC_CONFIRMATION':
            raise NotReady('PENDING_INTERNAL_IMU_EXTRINSIC_CONFIRMATION')
        confirmed(edge, 'confirmed_for_phase1')
        if edge.get('confidence') not in ('VENDOR_CONFIRMED', 'MEASURED', 'CALIBRATED'):
            raise NotReady('mid360_to_imu.confidence: vendor or measured confirmation required')
        finite_vector(cal, 'mid360_to_imu', 'xyz_m')
        finite_vector(cal, 'mid360_to_imu', 'rpy_rad')
    else:
        confirmed(cal, 'measured')
        for edge in ('base_to_yaw_axis', 'gimbal_to_mid360', 'mid360_to_imu'):
            finite_vector(cal, edge, 'xyz_m')
            finite_vector(cal, edge, 'rpy_rad')
    return cal


def _validate_radar_only_profile(root, calibration):
    profile = read(root / 'demo_radar_only.yaml')
    if profile.get('hardware_profile') != RADAR_ONLY_PROFILE:
        raise NotReady(f'hardware_profile must be {RADAR_ONLY_PROFILE}')
    if profile.get('physical_mid360_count') != 1:
        raise NotReady('radar-only profile requires exactly one physical MID360')
    if profile.get('selected_logical_role') != 'left' or profile.get('selected_frame') != 'mid360_left_frame':
        raise NotReady('radar-only profile must select the existing single-left interface')
    if profile.get('physical_installation') not in ('LEFT', 'RIGHT'):
        raise NotReady('physical_installation must explicitly select LEFT or RIGHT')
    excluded = profile.get('excluded_second_mid360', {})
    if excluded.get('enabled') is not False or excluded.get('status') != 'DAMAGED_EXCLUDED':
        raise NotReady('second MID360 must remain explicitly damaged/excluded')
    if profile.get('serial_applicability') != 'NOT_APPLICABLE':
        raise NotReady('serial must be NOT_APPLICABLE in radar-only profile')
    if profile.get('motion_authorization') is not False:
        raise NotReady('motion authorization must remain false')
    confirmed(profile, 'mechanically_locked_gimbal_confirmed')
    angle = profile.get('locked_gimbal_angle_rad')
    if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
        raise NotReady('locked_gimbal_angle_rad: finite measured radians required')
    required_text(profile, 'lock_method')
    if required_text(profile, 'calibration_revision') != calibration['revision']:
        raise NotReady('radar-only calibration_revision disagrees with demo_calibration.yaml')
    return profile


def _validate_lio(root, calibration=None, radar_only=False):
    lio = read(root / 'demo_lio.yaml')
    required_flags = ['enabled', 'raw_epoch_and_frame_reviewed', 'internal_imu_extrinsics_reviewed']
    if not radar_only:
        required_flags.append('mechanically_locked_gimbal_confirmed')
    for key in required_flags:
        confirmed(lio, key)
    existing_file(lio, 'phase1b_evidence')
    backend = read(existing_file(lio, 'backend_config_path'))['/**']['ros__parameters']
    if backend['mapping'].get('zero_start_pose') is not False:
        raise NotReady('disable delayed backend zero-start; do not introduce a hidden raw epoch jump')
    if backend['common'].get('multi_lidar') is not False:
        raise NotReady('single physical MID360 required')
    if backend['common'].get('lid_topic') != '/livox/left/lidar':
        raise NotReady('radar-only FAST-LIO must use the selected single MID360 topic')
    for key in ('extrinsic_T', 'extrinsic_R'):
        values = backend['mapping'].get(key)
        expected_size = 3 if key.endswith('T') else 9
        if (not isinstance(values, list) or len(values) != expected_size or
                any(type(x) not in (int, float) or not math.isfinite(x) for x in values)):
            raise NotReady(f'measured backend mapping.{key} required')
    if radar_only:
        # calibration stores parent lidar -> child IMU. FAST-LIO expects the
        # inverse: lidar pose with respect to IMU (T_imu_lidar).
        edge_t = finite_vector(calibration, 'mid360_to_imu', 'xyz_m')
        edge_r = rpy_to_matrix(finite_vector(calibration, 'mid360_to_imu', 'rpy_rad'))
        inverse_r = [edge_r[row + 3 * col] for row in range(3) for col in range(3)]
        inverse_t = [-sum(inverse_r[3 * row + col] * edge_t[col] for col in range(3))
                     for row in range(3)]
        for actual, expected in zip(backend['mapping']['extrinsic_T'], inverse_t):
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6):
                raise NotReady('FAST-LIO extrinsic_T must match inverse(mid360_to_imu)')
        for actual, expected in zip(backend['mapping']['extrinsic_R'], inverse_r):
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6):
                raise NotReady('FAST-LIO extrinsic_R must match inverse(mid360_to_imu)')
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
    return lio


def validate(config_dir, stage):
    """Validate only a requested acquisition stage, before creating any ROS nodes."""
    root = Path(config_dir)
    if stage == 'inventory':
        return {}
    if stage not in SUPPORTED_STAGES:
        raise NotReady(f'supported stages: {", ".join(SUPPORTED_STAGES)}; no motion stage')
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
        if type(cap) is not int or cap < 0 or cap > 0xffffffff or cap & 293 != 293:
            raise NotReady(
                'required capabilities must include chassis(1), relative gimbal(4), '
                'heading(32), chassis attitude(256)')
        for key in params:
            if key not in ('device', 'baudrate', 'required_remote_capabilities'):
                params[key] = positive(params, key)
        result['serial'] = serial
        result['serial_params'] = params
    if stage in ('lidar', 'fixed_lio', 'radar_fixed_lio'):
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
    if stage in ('tf', 'fixed_lio', 'radar_fixed_lio'):
        radar_only = stage == 'radar_fixed_lio'
        cal = _validate_calibration(root, radar_only=radar_only)
        if radar_only:
            result['radar_only'] = _validate_radar_only_profile(root, cal)
        else:
            positive(result['serial'], 'gimbal_joint_publish_rate_hz')
        result['calibration'] = cal
    if stage in ('fixed_lio', 'radar_fixed_lio'):
        radar_only = stage == 'radar_fixed_lio'
        confirmed(result['lidar'], 'raw_imu_frame_lineage_confirmed')
        result['lio'] = _validate_lio(root, result['calibration'], radar_only=radar_only)
    return result


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()


def prepare(repo, config_dir, output, operator, phase, acq_dir=None, hardware_profile=STM_PROFILE):
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
    if hardware_profile not in (STM_PROFILE, RADAR_ONLY_PROFILE):
        raise NotReady(f'unsupported hardware_profile: {hardware_profile}')
    selected_configs = {'demo_session.yaml', 'demo_mid360.yaml', 'demo_calibration.yaml', 'demo_lio.yaml'}
    if hardware_profile == STM_PROFILE:
        selected_configs.add('demo_serial.yaml')
    else:
        selected_configs.add('demo_radar_only.yaml')
    for path in config_dir.glob('demo_*.yaml'):
        if path.name not in selected_configs:
            continue
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
    lidar, cal = (read(snapshot / name) for name in ('demo_mid360.yaml', 'demo_calibration.yaml'))
    if hardware_profile == STM_PROFILE:
        serial = read(snapshot / 'demo_serial.yaml')
        metadata['devices']['serial'].update(profile=serial['profile'], device=serial['parameters']['device'],
            baudrate=serial['parameters']['baudrate'], mcu_firmware=serial['firmware_revision'],
            required_capabilities=serial['parameters']['required_remote_capabilities'])
    else:
        radar = read(snapshot / 'demo_radar_only.yaml')
        metadata['devices']['serial'] = dict(applicability='NOT_APPLICABLE',
                                            reason=RADAR_ONLY_PROFILE)
        metadata['hardware'].update(
            actual_mechanical_dof='FIXED_GIMBAL_FOR_SESSION',
            mechanically_locked_gimbal_confirmed=radar.get('mechanically_locked_gimbal_confirmed'),
            locked_gimbal_angle_rad=radar.get('locked_gimbal_angle_rad'),
            lock_method=radar.get('lock_method'),
            calibration_revision=radar.get('calibration_revision'),
            dynamic_tf_status='PREPARED / BLOCKED_BY_STM')
    metadata['devices']['primary_mid360'].update(
        role='left_logical_single_physical_sensor', identity=lidar['device_identity'],
        firmware=lidar['firmware_revision'],
        physical_installation=(radar.get('physical_installation')
                               if hardware_profile == RADAR_ONLY_PROFILE else
                               lidar.get('physical_installation')),
        calibration_revision=cal['revision'])
    demo.update(operator=operator, phase=phase, created_utc=datetime.now(timezone.utc).isoformat(),
                hardware_profile=hardware_profile, mechanical_calibration_revision=cal['revision'],
                motion_authorization=False, result='NOT_RUN')
    demo['capture_environment'] = dict(hostname=platform.node(), platform=platform.platform(), kernel=platform.release())
    metadata['demo'] = demo
    metadata['runtime']['config_snapshots'] = sorted(str(p) for p in snapshot.iterdir())
    metadata['runtime']['actual_launch_commands'] = demo['launch_commands']
    metadata['runtime']['effective_parameters'] = 'PENDING: record ros2 param dump for every started node'
    topics = read(acq / 'topics.yaml')
    if hardware_profile == STM_PROFILE:
        topics['required']['/motion/v2_guard/state'] = 'rm_competition_interfaces/msg/V2MotionGuardState'
    else:
        for topic in ('/gimbal/state', '/chassis/heading', '/serial/connection_state',
                      '/joint_states', '/serial/v2/timestamp_provenance'):
            topics['required'].pop(topic, None)
    if phase in 'CDEF' or hardware_profile == RADAR_ONLY_PROFILE:
        topics['required'].update({'/livox/lio_imu': 'sensor_msgs/msg/Imu', '/odometry/fast_lio_raw':'nav_msgs/msg/Odometry', '/odometry/lio':'nav_msgs/msg/Odometry'})
    if hardware_profile == STM_PROFILE:
        attitude = demo['chassis_attitude']
        if (attitude.get('topic') and attitude.get('type') and attitude.get('status_topic') and
                attitude.get('status_type') and attitude.get('contract_revision')):
            topics['required'][attitude['topic']] = attitude['type']
            topics['required'][attitude['status_topic']] = attitude['status_type']
            demo['chassis_attitude_status'] = 'HOST_READY_STM_FIRMWARE_PENDING'
        else:
            demo['chassis_attitude_status'] = 'BLOCKED_PENDING_STM_CONTRACT'
    else:
        demo['chassis_attitude_status'] = 'NOT_APPLICABLE_RADAR_ONLY'
    (output / 'session.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True))
    (output / 'topics.yaml').write_text(yaml.safe_dump(topics, sort_keys=False))
    shutil.copy2(acq / 'qos.yaml', output / 'qos.yaml')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('preflight')
    check.add_argument('--config-dir', required=True)
    check.add_argument('--stage', default='inventory', choices=SUPPORTED_STAGES)
    new = sub.add_parser('prepare')
    for name in ('repo', 'config-dir', 'output', 'operator'):
        new.add_argument('--' + name, required=True)
    new.add_argument('--phase', choices=list('ABCDEF'), default='A')
    new.add_argument('--acq-dir')
    new.add_argument('--hardware-profile', choices=[STM_PROFILE, RADAR_ONLY_PROFILE], default=STM_PROFILE)
    args = parser.parse_args()
    try:
        if args.command == 'preflight':
            validate(args.config_dir, args.stage)
            print(f'CONFIG_READY: {args.stage}; NOT hardware acceptance')
        else:
            print(prepare(args.repo, args.config_dir, args.output, args.operator, args.phase,
                          args.acq_dir, args.hardware_profile))
    except (NotReady, KeyError, ValueError, OSError, yaml.YAMLError) as error:
        parser.exit(2, f'NOT_READY: {error}\n')
