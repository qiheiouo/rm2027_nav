"""One isolated entry for CAD geometry, omni contacts, field/map and dynamic TF."""
import copy
import xml.etree.ElementTree as ET
import importlib.util
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def setup(context):
    root=Path(get_package_share_directory('rm_new_car_simulation'))
    script=Path(get_package_prefix('rm_new_car_simulation'))/'lib/rm_new_car_simulation/baseline_scene.py'
    spec=importlib.util.spec_from_file_location('baseline_scene',script);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    field=LaunchConfiguration('field').perform(context);lidar=LaunchConfiguration('lidar').perform(context)
    profile=LaunchConfiguration('profile').perform(context)
    sensors=LaunchConfiguration('sensors').perform(context)=='true'
    event_policy=LaunchConfiguration('navigation_policy').perform(context)=='event_driven'
    rebase_fixture=LaunchConfiguration('localization_source').perform(context)=='rebase_fixture'
    delay_fixture=LaunchConfiguration('rebase_delay_fixture').perform(context)
    if delay_fixture != 'none' and not rebase_fixture:
        raise RuntimeError('delay fixture is only allowed in rebase validation')
    correction_fixture=rebase_fixture or LaunchConfiguration('localization_source').perform(context)=='correction_gate_fixture'
    health_fixture=correction_fixture or LaunchConfiguration('localization_source').perform(context)=='lio_health_fixture'
    if health_fixture and not event_policy:
        raise RuntimeError('lio_health_fixture requires event_driven Mission integration')
    nav_odom='/odometry/lio' if health_fixture else '/odometry/simulation'
    if event_policy and (not sensors or LaunchConfiguration('use_nav2').perform(context)!='true'
                         or LaunchConfiguration('publish_truth_tf').perform(context)!='true'):
        raise RuntimeError('event_driven requires sensors, Nav2 and the explicit simulation oracle')
    output=Path(LaunchConfiguration('output').perform(context))
    requested_spawn=LaunchConfiguration('spawn').perform(context)
    spawn=([-10,0,.002,0,0,0] if field=='competition2025_reference' else [0,0,.002,0,0,0]) if requested_spawn=='auto' else list(map(float,requested_spawn.split(',')))
    if len(spawn)!=6: raise RuntimeError('spawn requires x,y,z,roll,pitch,yaw in metres and radians')
    manifest=m.generate(root,output,field,lidar,profile,sensors,spawn,
                        LaunchConfiguration('physics_config').perform(context) or None,
                        LaunchConfiguration('tunnel_section').perform(context),
                        LaunchConfiguration('field_tolerance').perform(context),
                        float(LaunchConfiguration('ground_bank_deg').perform(context)))
    if event_policy:
        # Co-located horizontal range fixture for costmap integration. This is
        # NOT a claim about a real MID360 sampling pattern or hardware coverage.
        world=ET.parse(output/'world.sdf')
        link=world.find(".//sensor[@name='mid360_geometry_proxy']/..")
        sensor=copy.deepcopy(link.find("sensor[@name='mid360_geometry_proxy']"))
        sensor.set('name','navigation_planar_fixture')
        sensor.find('topic').text='/simulation/navigation_scan'
        scan=sensor.find('lidar/scan')
        scan.remove(scan.find('vertical'))
        scan.find('horizontal/samples').text='360'
        link.append(sensor)
        world.write(output/'world.sdf')
    bridge=[]
    pairs=[('/model/new_car/world_odometry','/simulation/ground_truth/odom','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
      ('/model/new_car/sensor_world_odometry','/simulation/sensor_truth','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
      ('/model/new_car/joint_state','/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS'),
      ('/model/new_car/cmd_vel','/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ'),
      ('/model/new_car/gimbal_velocity','/simulation/gimbal_velocity','std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ'),
      ('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS')]
    if sensors:
        pairs += [('/model/new_car/lidar/points','/points/simulation_raw','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked','GZ_TO_ROS')]
    if event_policy:
        pairs.append(('/simulation/navigation_scan','/simulation/navigation_scan',
                      'sensor_msgs/msg/LaserScan','gz.msgs.LaserScan','GZ_TO_ROS'))
    for name in m.NAMES:
        pairs.append((f'/model/new_car/contact/{name}',f'/simulation/contact/{name}',
                      'ros_gz_interfaces/msg/Contacts','gz.msgs.Contacts','GZ_TO_ROS'))
    for gz,ros,rt,gt,d in pairs:
        bridge.append(dict(gz_topic_name=gz,ros_topic_name=ros,ros_type_name=rt,gz_type_name=gt,direction=d))
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    args=['gz','sim','-r','-v','2',str(output/'world.sdf')]
    if LaunchConfiguration('headless').perform(context)=='true': args+=['-s']
    if sensors: args+=['--headless-rendering']
    env={'GZ_SIM_SYSTEM_PLUGIN_PATH':str(Path(get_package_prefix('rm_new_car_simulation'))/'lib')+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')}
    nodes=[LogInfo(msg=f'field_revision={manifest["revision"]}; CAD provisional; physics contacts; full-pose simulation oracle; no hardware'),
      ExecuteProcess(cmd=args,additional_env=env,output='screen'),
      Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(output/'bridge.yaml'),'use_sim_time':True}],output='screen'),
      Node(package='rm_new_car_simulation',executable='simulation_tf.py',parameters=[{'use_sim_time':True,
        'publish_truth_tf':not health_fixture and LaunchConfiguration('publish_truth_tf').perform(context)=='true',
        'publish_localization_fixture':event_policy and not health_fixture}],output='screen'),
      Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(output/'robot.urdf').read_text(),
        'use_sim_time':True,'publish_frequency':100.}],output='screen')]
    if health_fixture:
        # Source fixture is supplied by validate_lio_health_navigation.py. Without
        # it the real adapter/bridge stay invalid; there is no truth-valid fallback.
        for package, executable in [('rm_localization_adapters','lio_adapter'),
                                    ('rm_relocalization_bridge','map_odom_from_global_pose')]:
            nodes.append(Node(package=package, executable=executable, name=executable,
                parameters=[str(root/'config/lio_health_fixture.yaml'),
                    *([str(root/'config/correction_gate_fixture.yaml')] if correction_fixture else []),
                    {'use_sim_time':True}, *([{'rebase.enabled':True}] if rebase_fixture and executable=='map_odom_from_global_pose' else [])], output='screen'))
    if correction_fixture:
        nodes.append(Node(package='rm_relocalization_bridge', executable='global_pose_gate_node',
            name='global_pose_gate', parameters=[{'use_sim_time':True}], output='screen'))
    rviz_config=root/'config/simulation.rviz'
    if event_policy:
        rviz=yaml.safe_load(rviz_config.read_text())
        rviz['Visualization Manager']['Tools']=[tool for tool in rviz['Visualization Manager']['Tools']
                                                if 'Goal' not in tool['Class']]
        rviz_config=output/'mission.rviz';rviz_config.write_text(yaml.safe_dump(rviz))
    nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'))
    if LaunchConfiguration('use_nav2').perform(context)=='true':
        config=Path(get_package_share_directory('rm_nav_config'))/'config/nav2_phase1_5_mppi.yaml'
        data=yaml.safe_load(config.read_text())
        cad,_=m.load_config(root)
        for name in ['local_costmap','global_costmap']:
            cost=data[name][name]['ros__parameters'];cost['footprint']=str(manifest['navigation_footprint_m']);cost['footprint_padding']=0.
            cost['plugins']=['static_layer','inflation_layer']
            cost['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
            cost['global_frame']='map' if name=='global_costmap' else 'odom'
        data['global_costmap']['global_costmap']['ros__parameters']['rolling_window']=False
        for name,block in data.items():
            if 'ros__parameters' in block:
                block['ros__parameters']['use_sim_time']=True
                block['ros__parameters']['enable_stamped_cmd_vel']=False
                if 'odom_topic' in block['ros__parameters']: block['ros__parameters']['odom_topic']=nav_odom
        data['controller_server']['ros__parameters']['progress_checker_plugins']=['progress_checker']
        data['controller_server']['ros__parameters']['odom_topic']=nav_odom
        def update_plugins(tree):
            for key,value in tree.items():
                if isinstance(value,dict): update_plugins(value)
                elif key=='plugin' and isinstance(value,str) and value.startswith('nav2_'):
                    tree[key]=value.replace('/','::')
        update_plugins(data)
        if event_policy:
            delta=yaml.safe_load((root/'config/event_replanning.yaml').read_text())
            data['controller_server']['ros__parameters']['progress_checker']=delta['controller_server']['ros__parameters']['progress_checker']
            data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(
                Path(get_package_share_directory('rm_nav_config'))/'behavior_trees/event_driven_replanning.xml')
            for name in ('local_costmap','global_costmap'):
                cost=data[name][name]['ros__parameters']
                cost['plugins']=['static_layer','obstacle_layer','inflation_layer']
                cost['obstacle_layer']={'plugin':'nav2_costmap_2d::ObstacleLayer', 'enabled':True,
                    'observation_sources':'scan', 'scan':{
                    'topic':'/simulation/navigation_scan', 'data_type':'LaserScan',
                    'marking':True, 'clearing':True, 'inf_is_valid':True,
                    'observation_persistence':0.0, 'expected_update_rate':0.5,
                    # 0.5 m excludes the CAD body from the offset sensor. 8 m is
                    # the tested integration horizon, not a real sensor limit.
                    'obstacle_min_range':0.5, 'obstacle_max_range':8.0,
                    'raytrace_min_range':0.0, 'raytrace_max_range':10.0,
                    'max_obstacle_height':2.0}}
            if delay_fixture == 'planner':
                data['planner_server']['ros__parameters']['GridBased']['plugin']='rm_new_car_simulation::DelayedPlanner'
            elif delay_fixture == 'controller':
                data['controller_server']['ros__parameters']['FollowPath']['plugin']='rm_new_car_simulation::DelayedController'
            if rebase_fixture:
                data['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['plugin']='rm_nav_config::RebaseObstacleLayer'
            mission_root=Path(get_package_share_directory('rm_competition_mission'))
            nodes.append(Node(package='rm_competition_mission', executable='competition_mission_node',
                name='competition_mission_node', parameters=[str(root/'config/event_mission.yaml'),
                {'use_sim_time':True, 'tree_xml':str(mission_root/'trees/competition_default.xml')},
                *([{'rebase_child_actions':['/compute_path_to_pose','/follow_path','/wait','/spin']}] if rebase_fixture else [])], output='screen'))
        data['map_server']={'ros__parameters':{'use_sim_time':True,'yaml_filename':str(output/'field.yaml')}}
        params=output/'nav2.yaml';params.write_text(yaml.safe_dump(data))
        # Exactly one selected localization chain owns canonical TF.
        names=['map_server','controller_server','planner_server','behavior_server','velocity_smoother','bt_navigator']
        packages=['nav2_map_server','nav2_controller','nav2_planner','nav2_behaviors','nav2_velocity_smoother','nav2_bt_navigator']
        for name,package in zip(names,packages):
            remap=[('odom',nav_odom)]
            if event_policy and name=='bt_navigator':
                remap += [('goal_pose','/simulation/unused_direct_goal_pose')]
            if name in ('controller_server','behavior_server'):remap += [('cmd_vel','/cmd_vel_nav')]
            if name=='velocity_smoother':remap += [('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel')]
            nodes.append(Node(package=package,executable=name,name=name,parameters=[str(params)],
                remappings=remap,output='screen'))
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='new_car_navigation_lifecycle',
            parameters=[{'use_sim_time':True,'autostart':True,'node_names':names}],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('rebase_delay_fixture',default_value='none',choices=['none','planner','controller']),
      DeclareLaunchArgument('localization_source',default_value='truth',choices=['truth','lio_health_fixture','correction_gate_fixture','rebase_fixture']),
      DeclareLaunchArgument('navigation_policy',default_value='baseline',choices=['baseline','event_driven']),
      DeclareLaunchArgument('field',default_value='rules2026_course',choices=['flat','rules2026_course','dog_slope','banked_ground','competition2025_reference']),
      DeclareLaunchArgument('lidar',default_value='A',choices=['A','B']),
      DeclareLaunchArgument('profile',default_value='deformed',choices=['deformed','undeformed']),
      DeclareLaunchArgument('sensors',default_value='false'),
      DeclareLaunchArgument('headless',default_value='true'),
      DeclareLaunchArgument('use_rviz',default_value='false'),
      DeclareLaunchArgument('use_nav2',default_value='false'),
      DeclareLaunchArgument('publish_truth_tf',default_value='true'),
      DeclareLaunchArgument('physics_config',default_value=''),
      DeclareLaunchArgument('tunnel_section',default_value='B',choices=['A','B']),
      DeclareLaunchArgument('field_tolerance',default_value='nominal',choices=['nominal','lower_clearance']),
      DeclareLaunchArgument('ground_bank_deg',default_value='1.5'),
      DeclareLaunchArgument('spawn',default_value='auto'),
      DeclareLaunchArgument('output',default_value='/tmp/rm_new_car_simulation_baseline'),OpaqueFunction(function=setup)])
