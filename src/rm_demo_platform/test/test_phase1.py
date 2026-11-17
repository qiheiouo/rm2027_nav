"""No devices: reject unknown physical inputs, preserve measurements and capture evidence."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace as NS

import pytest
import yaml

from rm_demo_platform.phase1 import (
    RADAR_ONLY_PROFILE,
    NotReady,
    locked_gimbal_transforms,
    prepare,
    read,
    validate,
)
from rm_demo_platform.observe import StreamStats

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]


def save(path, data):
    path.write_text(yaml.safe_dump(data))


@pytest.fixture
def config(tmp_path):
    dest = tmp_path / 'config'
    shutil.copytree(ROOT / 'config', dest)
    return dest


def valid_serial(config):
    # Synthetic acceptance fixture only; never sent to a device.
    path = config / 'demo_serial.yaml'
    item = read(path)
    item.update(firmware_repository='fixture', firmware_revision='fixture', wire_contract_confirmed=True,
                physical_motion_inhibit_confirmed=True, isolated_ros_domain_confirmed=True,
                gimbal_joint_publish_rate_hz=50.0)
    item['parameters'] = {k:1.0 for k in item['parameters']}
    item['parameters'].update(device='/dev/NOT_A_REAL_DEVICE',baudrate=115200,required_remote_capabilities=293)
    save(path,item)
    return item


def load_launch():
    spec = importlib.util.spec_from_file_location('demo_hardware', ROOT / 'launch/demo_hardware.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_launch_and_hardware_opt_in():
    from launch import LaunchContext
    from launch.actions import LogInfo
    launch = load_launch()
    context = LaunchContext()
    context.launch_configurations.update(stage='inventory', enable_hardware_io='false', session_dir='')
    assert all(isinstance(action, LogInfo) for action in launch.setup(context))
    context.launch_configurations['stage'] = 'serial'
    with pytest.raises(NotReady, match='enable_hardware_io'):
        launch.setup(context)


@pytest.mark.parametrize('stage', ['serial','lidar','tf','fixed_lio','radar_fixed_lio'])
def test_missing_inputs_reject_before_io(config, stage):
    with pytest.raises(NotReady):
        validate(config, stage)


def test_serial_missing_parameter_and_bypass_rejected(config):
    item = valid_serial(config)
    assert validate(config, 'serial')['serial_params']['device'] == '/dev/NOT_A_REAL_DEVICE'
    item['parameters'].pop('max_vx')
    save(config / 'demo_serial.yaml',item)
    with pytest.raises(NotReady, match='implicit'):
        validate(config,'serial')
    item = valid_serial(config)
    item['parameters']['motion_test_mode'] = True
    save(config / 'demo_serial.yaml',item)
    with pytest.raises(NotReady, match='implicit'):
        validate(config,'serial')
    item = valid_serial(config)
    item['physical_motion_inhibit_confirmed'] = False
    save(config / 'demo_serial.yaml',item)
    with pytest.raises(NotReady, match='inhibit'):
        validate(config,'serial')


def test_measured_model_retains_full_rpy_and_refuses_null(config):
    import xacro
    valid_serial(config)
    with pytest.raises(NotReady):
        validate(config,'tf')
    path = config / 'demo_calibration.yaml'
    cal = read(path)
    cal.update(measured=True,revision='synthetic-unit-fixture')
    for edge in ('base_to_yaw_axis','gimbal_to_mid360','mid360_to_imu'):
        cal[edge] = dict(xyz_m=[0.01,0.02,0.03],rpy_rad=[0.1,-0.2,0.3])
    save(path,cal)
    validate(config,'tf')
    xml = xacro.process_file(str(ROOT / 'urdf/demo_measured.urdf.xacro'), mappings={'calibration_file':str(path)}).toxml()
    assert 'rpy="0.1 -0.2 0.3"' in xml
    assert 'robot_radius' not in xml


def test_session_snapshots_and_no_false_pass(config,tmp_path):
    output = tmp_path / 'session'
    prepare(REPO, config, output, 'fixture', 'A')
    meta = read(output / 'session.yaml')
    assert meta['software']['rm_instrumentation_revision'] == subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip()
    assert meta['demo']['motion_authorization'] is False
    assert meta['demo']['result'] == 'NOT_RUN'
    assert meta['demo']['chassis_attitude_status'] == 'HOST_READY_STM_FIRMWARE_PENDING'
    assert (output / 'dirty.patch').is_file()
    assert '/motion/v2_guard/state' in read(output / 'topics.yaml')['required']
    assert read(output / 'topics.yaml')['required']['/chassis/imu'] == 'sensor_msgs/msg/Imu'
    assert read(output / 'topics.yaml')['required']['/chassis/attitude_status'] == \
        'rm_competition_interfaces/msg/ChassisAttitudeStatus'
    original = (output / 'config/demo_serial.yaml').read_text()
    valid_serial(config)
    assert (output / 'config/demo_serial.yaml').read_text() == original
    with pytest.raises(FileExistsError):
        prepare(REPO,config,output,'fixture','A')


def test_sequence_wrap_reset_and_time_regression():
    stats = StreamStats()
    def sample(seq, stamp, boot=1, reset=0):
        return NS(sample_sequence=seq,source_boot_id=boot,reset_counter=reset,valid=True,online=True,
                  header=NS(frame_id='base_link',stamp=NS(sec=stamp,nanosec=0)))
    stats.update(sample(0xffffffff,3),1.0)
    stats.update(sample(0,4),1.1)
    stats.update(sample(3,2),1.3)
    stats.update(sample(0,5,boot=2),1.4)
    stats.update(sample(0,6,boot=2,reset=1),1.5)
    report = stats.summary()
    assert report['observed_sequence_gaps'] == 2
    assert report['nonmonotonic_stamps'] == 1
    assert report['backward_sequences'] == 0
    assert len(report['events']) == 4


def test_passive_observer_records_real_ros_bag(tmp_path):
    # Reuse existing ROS messages, no fake hardware or command publications.
    import rclpy
    from std_msgs.msg import String
    from rm_demo_platform.observe import observe
    session = tmp_path / 'observe'
    session.mkdir()
    save(session / 'session.yaml', dict(bag_directory=str(session / 'bag'),software={},runtime={},session_evidence={}))
    save(session / 'topics.yaml', dict(required={'/demo_observer_fixture':'std_msgs/msg/String'}))
    save(session / 'qos.yaml', {})
    # Independent process lets the observer own its own rclpy lifecycle.
    publisher = subprocess.Popen(['python3','-c',
        'import rclpy,time; from std_msgs.msg import String; rclpy.init(); n=rclpy.create_node("fixture_publisher"); p=n.create_publisher(String,"/demo_observer_fixture",10); n.create_timer(0.05,lambda:p.publish(String(data="observation"))); rclpy.spin(n)'])
    try:
        report = observe(session,4.0,True)
        assert report['topics']['/demo_observer_fixture']['count'] > 0
        assert report['bag_metadata_present']
        assert report['recorder_exit_code'] == 0
        assert report['hardware_go'] is False
        assert not report['missing_topics']
    finally:
        publisher.terminate()
        publisher.wait(timeout=5)


def valid_lidar(config, tmp_path):
    sdk = tmp_path / 'sdk.json'
    import json
    sdk.write_text(json.dumps(dict(MID360=dict(host_net_info={k:'192.0.2.2' for k in
        ('cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip')}),
        lidar_configs=[dict(ip='192.0.2.3',extrinsic_parameter={k:0 for k in ('x','y','z','roll','pitch','yaw')})])))
    item = read(config / 'demo_mid360.yaml')
    item.update(device_identity='fixture',firmware_revision='fixture',host_ip='192.0.2.2',lidar_ip='192.0.2.3',
                sdk_config_path=str(sdk),sdk_config_reviewed=True,publish_freq_hz=10.0,raw_imu_frame_lineage_confirmed=True)
    save(config / 'demo_mid360.yaml',item)
    return item


def valid_radar_only(config, tmp_path):
    valid_lidar(config, tmp_path)
    cal = read(config / 'demo_calibration.yaml')
    cal['mid360_to_imu'] = dict(
        xyz_m=[0.01, -0.02, 0.03], rpy_rad=[0.0, 0.0, 0.0],
        source='synthetic-unit-fixture', confidence='MEASURED', confirmed_for_phase1=True)
    save(config / 'demo_calibration.yaml', cal)
    profile = read(config / 'demo_radar_only.yaml')
    profile.update(mechanically_locked_gimbal_confirmed=True,
                   locked_gimbal_angle_rad=0.25,
                   lock_method='synthetic-unit-fixture',
                   physical_installation='LEFT')
    save(config / 'demo_radar_only.yaml', profile)
    backend = read(REPO / 'src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml')
    backend['/**']['ros__parameters']['mapping'].update(
        zero_start_pose=False,
        extrinsic_T=[-0.01, 0.02, -0.03],
        extrinsic_R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    adapter = read(REPO / 'src/rm_lio_bringup/config/lio_adapter_fast_lio_multi.yaml')
    adapter['lio_adapter']['ros__parameters']['raw_odom_parent_frame_mode'] = 'canonical_odom'
    save(tmp_path / 'backend.yaml', backend)
    save(tmp_path / 'adapter.yaml', adapter)
    (tmp_path / 'evidence.txt').write_text('synthetic unit fixture; not hardware evidence')
    lio = read(config / 'demo_lio.yaml')
    lio.update(enabled=True, raw_epoch_and_frame_reviewed=True,
               internal_imu_extrinsics_reviewed=True,
               backend_config_path=str(tmp_path / 'backend.yaml'),
               adapter_config_path=str(tmp_path / 'adapter.yaml'),
               phase1b_evidence=str(tmp_path / 'evidence.txt'))
    save(config / 'demo_lio.yaml', lio)
    return backend


def test_lidar_manifest_cannot_hide_ip_or_extra_transform(config,tmp_path):
    import json
    item = valid_lidar(config,tmp_path)
    validate(config,'lidar')
    item['host_ip'] = '192.0.2.4'
    save(config / 'demo_mid360.yaml',item)
    with pytest.raises(NotReady,match='host IPs'):
        validate(config,'lidar')
    item = valid_lidar(config,tmp_path)
    sdk = Path(item['sdk_config_path'])
    data = json.loads(sdk.read_text())
    data['lidar_configs'][0]['extrinsic_parameter']['roll'] = 0.2
    sdk.write_text(json.dumps(data))
    with pytest.raises(NotReady,match='extrinsic'):
        validate(config,'lidar')
    item = valid_lidar(config,tmp_path)
    sdk = Path(item['sdk_config_path'])
    data = json.loads(sdk.read_text())
    data['lidar_configs'].append(dict(ip='192.0.2.4', extrinsic_parameter={
        key: 0 for key in ('x','y','z','roll','pitch','yaw')}))
    sdk.write_text(json.dumps(data))
    with pytest.raises(NotReady, match='exactly the selected'):
        validate(config, 'lidar')


def test_fixed_lio_rejects_epoch_jump_and_yaw_only_conversion(config,tmp_path):
    valid_serial(config)
    valid_lidar(config,tmp_path)
    cal = read(config / 'demo_calibration.yaml')
    cal.update(measured=True,revision='fixture')
    for edge in ('base_to_yaw_axis','gimbal_to_mid360','mid360_to_imu'):
        cal[edge] = dict(xyz_m=[0.01,0.02,0.03],rpy_rad=[0.1,0.2,0.3])
    save(config / 'demo_calibration.yaml',cal)
    backend = read(REPO / 'src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml')
    adapter = read(REPO / 'src/rm_lio_bringup/config/lio_adapter_fast_lio_multi.yaml')
    adapter['lio_adapter']['ros__parameters']['raw_odom_parent_frame_mode'] = 'canonical_odom'
    save(tmp_path / 'backend.yaml',backend)
    save(tmp_path / 'adapter.yaml',adapter)
    (tmp_path / 'safety.txt').write_text('synthetic validation fixture; no physical acceptance')
    lio = read(config / 'demo_lio.yaml')
    lio.update(enabled=True,mechanically_locked_gimbal_confirmed=True,raw_epoch_and_frame_reviewed=True,
               internal_imu_extrinsics_reviewed=True,backend_config_path=str(tmp_path / 'backend.yaml'),
               adapter_config_path=str(tmp_path / 'adapter.yaml'),phase1b_evidence=str(tmp_path / 'safety.txt'))
    save(config / 'demo_lio.yaml',lio)
    with pytest.raises(NotReady,match='zero-start'):
        validate(config,'fixed_lio')
    backend['/**']['ros__parameters']['mapping']['zero_start_pose'] = False
    save(tmp_path / 'backend.yaml',backend)
    validate(config,'fixed_lio')
    adapter['lio_adapter']['ros__parameters']['pose_conversion_mode'] = 'chassis_heading_fusion'
    save(tmp_path / 'adapter.yaml',adapter)
    with pytest.raises(NotReady,match='sensor_tf'):
        validate(config,'fixed_lio')


def test_radar_only_needs_no_serial_and_rejects_second_sensor(config, tmp_path):
    valid_radar_only(config, tmp_path)
    (config / 'demo_serial.yaml').unlink()
    result = validate(config, 'radar_fixed_lio')
    assert 'serial' not in result
    assert result['radar_only']['hardware_profile'] == RADAR_ONLY_PROFILE
    assert result['lio']['enabled'] is True
    profile = read(config / 'demo_radar_only.yaml')
    profile['physical_mid360_count'] = 2
    save(config / 'demo_radar_only.yaml', profile)
    with pytest.raises(NotReady, match='exactly one'):
        validate(config, 'radar_fixed_lio')


def test_radar_only_requires_lock_and_internal_imu_confirmation(config, tmp_path):
    valid_radar_only(config, tmp_path)
    profile = read(config / 'demo_radar_only.yaml')
    profile['mechanically_locked_gimbal_confirmed'] = False
    save(config / 'demo_radar_only.yaml', profile)
    with pytest.raises(NotReady, match='mechanically_locked'):
        validate(config, 'radar_fixed_lio')
    profile['mechanically_locked_gimbal_confirmed'] = True
    save(config / 'demo_radar_only.yaml', profile)
    cal = read(config / 'demo_calibration.yaml')
    cal['mid360_to_imu'].update(xyz_m=None, rpy_rad=None,
        confidence='PENDING_INTERNAL_IMU_EXTRINSIC_CONFIRMATION', confirmed_for_phase1=False)
    save(config / 'demo_calibration.yaml', cal)
    with pytest.raises(NotReady, match='PENDING_INTERNAL_IMU'):
        validate(config, 'radar_fixed_lio')


def test_radar_only_rejects_backend_internal_extrinsic_mismatch(config, tmp_path):
    valid_radar_only(config, tmp_path)
    backend_path = tmp_path / 'backend.yaml'
    backend = read(backend_path)
    backend['/**']['ros__parameters']['mapping']['extrinsic_T'] = [0.0, 0.0, 0.0]
    save(backend_path, backend)
    with pytest.raises(NotReady, match='inverse'):
        validate(config, 'radar_fixed_lio')
    valid_radar_only(config, tmp_path)
    backend = read(backend_path)
    backend['/**']['ros__parameters']['common']['multi_lidar'] = True
    save(backend_path, backend)
    with pytest.raises(NotReady, match='single physical'):
        validate(config, 'radar_fixed_lio')


def test_locked_gimbal_uses_se3_rotation_composition():
    calibration = {
        'base_to_yaw_axis': {'xyz_m':[0.0, 0.0, 0.1], 'rpy_rad':[1.5707963267948966, 0.0, 0.0]},
        'gimbal_to_mid360_left_cad_nominal': {'xyz_m':[0.0, 0.1, 0.1], 'rpy_rad':[0.0, 0.0, 0.0]},
        'gimbal_to_mid360_right_cad_nominal': {'xyz_m':[0.0, -0.1, 0.1], 'rpy_rad':[0.0, 0.0, 0.0]},
        'mid360_to_imu': {'xyz_m':[0.0, 0.0, 0.0], 'rpy_rad':[0.0, 0.0, 0.0]},
    }
    transforms = locked_gimbal_transforms(calibration, 1.5707963267948966)
    assert transforms[0]['parent'] == 'base_link'
    assert transforms[0]['child'] == 'gimbal_yaw_link'
    assert transforms[0]['quaternion'] == pytest.approx((0.5, -0.5, 0.5, 0.5))
    assert locked_gimbal_transforms(calibration, 0.0, 'RIGHT')[1]['xyz'][1] == -0.1


def test_repository_cad_nominal_values_match_user_geometry():
    cal = read(ROOT / 'config/demo_calibration.yaml')
    assert cal['ground_to_lowest_circular_plate_bottom_m'] == 0.043
    assert cal['base_to_yaw_axis']['xyz_m'] == [0.0, 0.0, 0.103]
    assert cal['gimbal_to_mid360_left_cad_nominal']['xyz_m'] == [0.0, 0.132028, 0.1]
    assert cal['gimbal_to_mid360_left_cad_nominal']['rpy_rad'] == [0.0, 0.523599, 1.570796]
    assert cal['gimbal_to_mid360_right_cad_nominal']['xyz_m'] == [0.0, -0.132028, 0.1]
    assert cal['gimbal_to_mid360_right_cad_nominal']['rpy_rad'] == [0.0, -0.523599, 1.570796]
    assert cal['mid360_to_imu']['xyz_m'] == [0.011, 0.02329, -0.04412]
    assert cal['mid360_to_imu']['rpy_rad'] == [0.0, 0.0, 0.0]
    assert cal['mid360_to_imu']['confidence'] == 'VENDOR_CONFIRMED'
    assert cal['mid360_to_imu']['confirmed_for_phase1'] is True
    mapping = read(REPO / 'src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml')['/**']['ros__parameters']['mapping']
    assert mapping['extrinsic_T'] == [-0.011, -0.02329, 0.04412]
    assert mapping['extrinsic_R'] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def test_radar_only_session_has_no_serial_or_stm_requirements(config, tmp_path):
    valid_radar_only(config, tmp_path)
    (config / 'demo_serial.yaml').unlink()
    output = tmp_path / 'radar_session'
    prepare(REPO, config, output, 'fixture', 'C', hardware_profile=RADAR_ONLY_PROFILE)
    meta = read(output / 'session.yaml')
    topics = read(output / 'topics.yaml')['required']
    assert meta['demo']['hardware_profile'] == RADAR_ONLY_PROFILE
    assert meta['demo']['motion_authorization'] is False
    assert meta['devices']['serial']['applicability'] == 'NOT_APPLICABLE'
    assert meta['hardware']['mechanically_locked_gimbal_confirmed'] is True
    assert not (output / 'config/demo_serial.yaml').exists()
    assert '/gimbal/state' not in topics
    assert '/chassis/imu' not in topics
    assert '/chassis/attitude_status' not in topics
    assert '/motion/v2_guard/state' not in topics
    assert '/serial/connection_state' not in topics
    assert topics['/odometry/lio'] == 'nav_msgs/msg/Odometry'


def test_radar_only_launch_has_one_driver_static_tf_and_no_serial(config, tmp_path, monkeypatch):
    valid_radar_only(config, tmp_path)
    output = tmp_path / 'launch_session'
    prepare(REPO, config, output, 'fixture', 'C', hardware_profile=RADAR_ONLY_PROFILE)
    launch = load_launch()
    created = []
    def fake_node(**kwargs):
        created.append(kwargs)
        return NS(kind='node', kwargs=kwargs)
    monkeypatch.setattr(launch, 'Node', fake_node)
    monkeypatch.setattr(launch, 'IncludeLaunchDescription',
                        lambda *args, **kwargs: NS(kind='include', args=args, kwargs=kwargs))
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda package: str(
        ROOT if package == 'rm_demo_platform' else REPO / 'src' / package))
    from launch import LaunchContext
    context = LaunchContext()
    context.launch_configurations.update(stage='radar_fixed_lio', enable_hardware_io='true',
                                         session_dir=str(output))
    actions = launch.setup(context)
    packages = [item['package'] for item in created]
    assert packages.count('livox_ros_driver2') == 1
    assert packages.count('tf2_ros') == 3
    assert 'rm_serial_driver' not in packages
    assert 'robot_state_publisher' not in packages
    assert all(item.get('executable') != 'gimbal_state_adapter' for item in created)
    assert any(getattr(action, 'kind', None) == 'include' for action in actions)


def test_container_runner_never_maps_serial_for_radar_only():
    script = (ROOT / 'scripts/run_phase1_container.sh').read_text()
    assert 'if [ "$stage" = serial ] || [ "$stage" = tf ] || [ "$stage" = fixed_lio ]' in script
    assert '[ "$stage" != lidar ]' not in script


def test_hardware_launch_real_default_exits_without_nodes():
    result = subprocess.run(['ros2','launch','rm_demo_platform','demo_hardware.launch.py'],
                            capture_output=True,text=True,timeout=10)
    assert result.returncode == 0
    assert 'zero nodes' in result.stdout
    assert 'process started' not in result.stdout
