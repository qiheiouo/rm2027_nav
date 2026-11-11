"""No devices: reject unknown physical inputs, preserve measurements and capture evidence."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace as NS

import pytest
import yaml

from rm_demo_platform.phase1 import NotReady, prepare, read, validate
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
    item['parameters'].update(device='/dev/NOT_A_REAL_DEVICE',baudrate=115200,required_remote_capabilities=37)
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


@pytest.mark.parametrize('stage', ['serial','lidar','tf','fixed_lio'])
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
    assert meta['demo']['chassis_attitude_status'] == 'BLOCKED_PENDING_STM_CONTRACT'
    assert (output / 'dirty.patch').is_file()
    assert '/motion/v2_guard/state' in read(output / 'topics.yaml')['required']
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


def test_hardware_launch_real_default_exits_without_nodes():
    result = subprocess.run(['ros2','launch','rm_demo_platform','demo_hardware.launch.py'],
                            capture_output=True,text=True,timeout=10)
    assert result.returncode == 0
    assert 'zero nodes' in result.stdout
    assert 'process started' not in result.stdout
