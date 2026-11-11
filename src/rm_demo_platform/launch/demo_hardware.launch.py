"""Measured, explicit Phase 1 acquisition. Defaults to inventory (zero nodes)."""
from pathlib import Path
from datetime import datetime, timezone
import json
import os
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rm_demo_platform.phase1 import NotReady, read, required, validate


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    stage = arg('stage')
    if stage == 'inventory':
        return [LogInfo(msg='Demo inventory: zero nodes, no hardware IO, no motion authority.')]
    if arg('enable_hardware_io') != 'true':
        raise NotReady('enable_hardware_io:=true required for explicit physical acquisition')
    session = Path(arg('session_dir'))
    if not session.is_absolute():
        raise NotReady('session_dir must be an absolute prepared session path')
    meta = read(session / 'session.yaml')
    session_id = required(meta, 'session_id')
    if not isinstance(session_id, str) or not 1 <= len(session_id.encode()) <= 128:
        raise NotReady('session_id must contain 1..128 bytes')
    if meta['demo'].get('motion_authorization') is not False:
        raise NotReady('this launch does not implement real motion authorization')
    cfg = validate(session / 'config', stage)
    share = Path(get_package_share_directory('rm_demo_platform'))
    actions = []
    # Expand measured geometry BEFORE returning any device-start actions.
    if 'calibration' in cfg:
        import xacro
        model = xacro.process_file(str(share / 'urdf/demo_measured.urdf.xacro'),
            mappings={'calibration_file': str(session / 'config/demo_calibration.yaml')}).toxml()
        actions += [Node(package='robot_state_publisher', executable='robot_state_publisher',
                         name='robot_state_publisher', parameters=[{'robot_description': ParameterValue(model, value_type=str)}]),
                    Node(package='rm_localization_adapters', executable='gimbal_state_adapter',
                         name='gimbal_state_adapter', parameters=[{'use_input':True,
                         'publish_rate_hz':float(cfg['serial']['gimbal_joint_publish_rate_hz']),
                         'stale_timeout_sec':cfg['serial_params']['gimbal_timeout_sec']}])]
    if 'serial' in cfg:
        params = cfg['serial_params'] | {'dry_run':False, 'motion_test_mode':False,
            'enable_timestamp_provenance':True, 'provenance_session_id':session_id,
            # Posture uses a separate actuator path. Quarantine it per session.
            'posture_request_topic':f'/demo/inhibited_{session_id.replace("-", "_")}/posture_request'}
        actions.append(Node(package='rm_serial_driver', executable='competition_v2_transport_node',
                            name='serial_transport_node', parameters=[params]))
    if 'lidar' in cfg:
        lidar = cfg['lidar']
        actions.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_left_driver', parameters=[{'xfer_format':4, 'multi_topic':0, 'data_src':0,
            'publish_freq':float(lidar['publish_freq_hz']), 'output_data_type':0,
            'frame_id':'mid360_left_frame', 'lvx_file_path':'',
            'user_config_path':lidar['sdk_config_path'], 'cmdline_input_bd_code':ParameterValue(lidar['device_identity'], value_type=str),
            'enable_timestamp_provenance':True, 'provenance_session_id':session_id,
            'provenance_device_identity':ParameterValue(lidar['device_identity'], value_type=str), 'provenance_sensor_role':'left'}],
            remappings=[('/livox/lidar','/livox/left/lidar'),
                        ('/livox/lidar/pointcloud','/livox/left/pointcloud'),
                        ('/livox/imu','/livox/lio_imu_raw'),
                        ('/livox/timestamp_provenance','/livox/left/timestamp_provenance')]))
    if 'lio' in cfg:
        lio = cfg['lio']
        actions += [Node(package='rm_localization_adapters', executable='imu_frame_adapter',
                         name='imu_frame_adapter'),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
                        get_package_share_directory('rm_lio_bringup')) / 'launch/fast_lio_multi_phase2a.launch.py')),
                        launch_arguments={'use_backend':'true','sensor_mode':'single','selected_side':'left',
                                          'config_file':lio['backend_config_path'], 'publish_tf_results':'false'}.items()),
                    Node(package='rm_localization_adapters', executable='lio_adapter', name='lio_adapter',
                         parameters=[lio['adapter_config_path']])]
    # Separate files avoid races when serial/TF and LiDAR start in different containers.
    record = dict(stage=stage, started_utc=datetime.now(timezone.utc).isoformat(),
                  session_id=session_id, motion_authorization=False,
                  container_image=os.environ.get('RM_DEMO_CONTAINER_IMAGE'),
                  container_image_id=os.environ.get('RM_DEMO_CONTAINER_IMAGE_ID'),
                  ros_distro=os.environ.get('ROS_DISTRO'), domain_id=os.environ.get('ROS_DOMAIN_ID'),
                  launch_arguments=dict(stage=stage, enable_hardware_io=True, session_dir=str(session)))
    (session / f'launch_{stage}_{uuid.uuid4().hex[:8]}.json').write_text(json.dumps(record, indent=2))
    return [LogInfo(msg=f'Phase 1 acquisition: {stage}. Serial sends heartbeat and disabled zero commands; no Nav2 or motion grant.')] + actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='inventory', choices=['inventory','serial','lidar','tf','fixed_lio']),
        DeclareLaunchArgument('enable_hardware_io', default_value='false', choices=['false','true']),
        DeclareLaunchArgument('session_dir', default_value=''),
        OpaqueFunction(function=setup),
    ])
