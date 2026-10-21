"""Both adapters are shadow; baseline truth remains the sole public TF owner."""
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    scene=Path(LaunchConfiguration('scene').perform(context));m=json.loads((scene/'manifest.json').read_text())
    center=m['gimbal_center_m'];home=[a+b for a,b in zip(center,m['sensor_offset_m'])]
    rp=json.loads(LaunchConfiguration('startup_roll_pitch').perform(context))
    params=dict(use_sim_time=True,raw_odom_topic='/shadow/input/raw',output_odom_topic='/shadow/planar/odom',
        expected_input_odom_frame='sensor_epoch_1',odom_frame='shadow_planar_odom',base_frame='shadow_planar_base',
        input_sensor_frame='sim_lidar_link',publish_tf=False,use_tf_sensor_to_base=False,
        pose_conversion_mode='chassis_heading_fusion',raw_odom_parent_frame_mode='sensor_initial',
        twist_mode='finite_difference',**{'heading_fusion.heading_topic':'/shadow/input/heading',
        'heading_fusion.derived_gimbal_topic':'/shadow/planar/gimbal','heading_fusion.initial_alignment_confirmed':True,
        'heading_fusion.initial_gimbal_yaw_rad':0.,'twist_estimator.max_linear_speed':100.,'twist_estimator.max_angular_speed':100.})
    for i,k in enumerate('xyz'):
        params['heading_fusion.gimbal_center_in_base.'+k]=float(center[i])
        params['heading_fusion.initial_base_to_sensor.'+k]=float(home[i])
    paths=[str(Path(get_package_prefix(p))/'lib') for p in ['rm_new_car_simulation','rm_full_base_shadow_validation']]
    return [ExecuteProcess(cmd=['gz','sim','-r','-s','-v','2',str(scene/'world.sdf')],
              additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH':os.pathsep.join(paths)+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')},output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(scene/'bridge.yaml'),'use_sim_time':True}],output='screen'),
        Node(package='rm_new_car_simulation',executable='simulation_tf.py',parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(scene/'robot.urdf').read_text(),'use_sim_time':True}],output='screen'),
        Node(package='rm_full_base_shadow_validation',executable='ingress.py',parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='rm_localization_adapters',executable='lio_adapter',name='planar_shadow',parameters=[params],output='screen'),
        Node(package='rm_full_base_shadow_validation',executable='candidate.py',parameters=[{'use_sim_time':True,
             'manifest':str(scene/'manifest.json'),'startup_roll_pitch':rp,
             'startup_home_confirmed':True,
             'missing_startup':LaunchConfiguration('missing_startup').perform(context)=='true'}],output='screen')]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scene'),DeclareLaunchArgument('startup_roll_pitch',default_value='[0.0,0.0]'),
        DeclareLaunchArgument('missing_startup',default_value='false'),OpaqueFunction(function=setup)])
