"""Gazebo, physical robot adapters and a YAML registry of sandbox components."""
import os
from pathlib import Path
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from simulation_bringup.model import load_config, robot_xml, house_xml, bridges


def start(context):
    config = str(Path(LaunchConfiguration('config').perform(context)).resolve())
    cfg = load_config(config)
    gui = LaunchConfiguration('gui').perform(context)
    if gui != 'config':
        cfg['gazebo']['gui'] = gui == 'true'
    temporary = tempfile.TemporaryDirectory(prefix='pirobot-sim-')
    directory = Path(temporary.name)
    descriptor = Path(get_package_share_directory('robot_descriptor'))
    description = robot_xml(xacro.process_file(str(descriptor / 'descriptor/robot.urdf.xacro')).toxml(), cfg)
    # Absolute mesh URIs work in Gazebo without relying on a developer's directory.
    description = description.replace('package://robot_descriptor/', f'file://{descriptor}/')
    (directory / 'robot.urdf').write_text(description)
    if not cfg['gazebo']['world_file']:
        (directory / 'house.sdf').write_text(house_xml(cfg))
    bridge_config = bridges(cfg)
    cloud_bridge = [row for row in bridge_config if row['ros_type_name'] == 'sensor_msgs/msg/PointCloud2']
    image_bridges = [row for row in bridge_config if row not in cloud_bridge]
    (directory / 'bridge.yaml').write_text(yaml.safe_dump(image_bridges))
    (directory / 'cloud_bridge.yaml').write_text(yaml.safe_dump(cloud_bridge))
    world = cfg['gazebo']['world_file'] or str(directory / 'house.sdf')
    command = ['gz', 'sim', '-r', '-v', '3', '--seed', str(cfg['gazebo']['seed'])]
    if not cfg['gazebo']['gui']:
        command.append('-s')
    if cfg['gazebo']['headless_rendering']:
        command.append('--headless-rendering')
    command.append(world)
    resources = os.pathsep.join(filter(None, [str(Path(cfg['gazebo']['aws_house_dir']) / 'models'),
                                              os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))
    gazebo = ExecuteProcess(cmd=command, output='screen',
                            additional_env={'GZ_SIM_RESOURCE_PATH': resources})
    spawn = cfg['gazebo']['spawn']
    create = Node(package='ros_gz_sim', executable='create', arguments=[
        '-name', 'burger', '-file', str(directory / 'robot.urdf'),
        '-x', str(spawn[0]), '-y', str(spawn[1]), '-z', str(spawn[2]), '-Y', str(spawn[3])], output='screen')
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: temporary.cleanup() or [])])),
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
                             on_exit=[OpaqueFunction(function=lambda context: [] if context.is_shutdown else
                                       [EmitEvent(event=Shutdown(reason='Gazebo exited'))])])),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(description, value_type=str), 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(directory / 'bridge.yaml'), 'use_sim_time': True}], output='screen'),
        # Harmonic RGB-D cloud XYZ uses X-forward camera coordinates even when
        # the sensor labels it with optical_frame_id. Images remain optical.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_points_bridge',
             parameters=[{'config_file': str(directory / 'cloud_bridge.yaml'),
                          'override_frame_id': 'realsense_link', 'use_sim_time': True}], output='screen'),
        Node(package='simulation_bringup', executable='hardware_adapter',
             parameters=[{'config_file': config, 'use_sim_time': True}], output='screen'), create]
    if cfg['gazebo']['gui']:
        actions.append(ExecuteProcess(cmd=['python3', '-m', 'simulation_bringup.chase_camera'], output='screen'))
    if LaunchConfiguration('slam').perform(context) == 'true':
        slam_config = yaml.safe_load(Path(cfg['slam']['parameter_file']).read_text())
        slam_config['slam_toolbox']['ros__parameters'].update(
            use_sim_time=True, base_frame='base_link', odom_frame='odom',
            map_frame='map', scan_topic=cfg['topics']['scan'])
        slam_parameters = directory / 'slam.yaml'
        slam_parameters.write_text(yaml.safe_dump(slam_config))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py')),
            launch_arguments={'slam_params_file': str(slam_parameters),
                              'use_sim_time': 'true', 'autostart': 'true'}.items()))
    for key, item in cfg['components'].items():
        switch = 'joystick' if item.get('group') == 'joystick' else ('rviz' if key == 'rviz' else None)
        selected = LaunchConfiguration(switch).perform(context) if switch else 'config'
        enabled = item.get('enabled', True) and selected != 'false'
        if not enabled:
            continue
        parameters = dict(item.get('parameters', {}))
        if key == 'sensor_fusion':
            # Adapt the boundary only. Preserve all filter tuning from its own YAML.
            parameters.update(cmd_vel_topic=cfg['topics']['accepted'], imu_topic=cfg['topics']['imu'],
                              encoder_twist_topic=cfg['topics']['encoder'], encoder_input_type='twist',
                              odometry_topic='/odom', odom_frame_id='odom', base_frame_id='base_link', publish_tf=True)
        parameters['use_sim_time'] = True
        actions.append(Node(package=item['package'], executable=item['executable'],
                            name=item.get('name', key), namespace=item.get('namespace', ''),
                            parameters=[*item.get('parameter_files', []), parameters],
                            remappings=list(item.get('remappings', {}).items()),
                            arguments=item.get('arguments', []), output='screen'))
    return actions


def generate_launch_description():
    default = os.environ.get('PIROBOT_SIM_CONFIG', str(Path(get_package_share_directory('simulation_bringup')) / 'config/simulation.yaml'))
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default),
                              DeclareLaunchArgument('gui', default_value='config', choices=['config', 'true', 'false']),
                              DeclareLaunchArgument('rviz', default_value='config', choices=['config', 'true', 'false']),
                              DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
                              DeclareLaunchArgument('joystick', default_value='config', choices=['config', 'true', 'false']),
                              OpaqueFunction(function=start)])
