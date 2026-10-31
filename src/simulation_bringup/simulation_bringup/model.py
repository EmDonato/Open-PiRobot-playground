"""Generate simulation-only assets without changing the physical robot description."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load_config(path):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    for group, keys in [('sensors', ['lidar_hz', 'imu_hz', 'encoder_hz', 'rgbd_hz']),
                        ('motors', ['control_hz', 'wheel_radius', 'wheel_base', 'max_wheel_rad_s',
                                    'time_constant', 'torque_limit', 'command_timeout', 'acceleration_rad_s2']),
                        ('gazebo', ['step_size', 'real_time_factor'])]:
        for key in keys:
            value = cfg[group][key]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{group}.{key} must be finite and positive')
    for key in ['encoder_ticks_per_rev', 'width', 'height']:
        value = cfg['sensors'][key]
        if not isinstance(value, int) or value <= 0:
            raise ValueError(f'sensors.{key} must be a positive integer')
    for key in ['lidar_stddev', 'gyro_stddev', 'gyro_bias_stddev', 'gyro_dynamic_bias_stddev',
                'accel_stddev', 'encoder_stddev', 'camera_noise', 'reorder_window']:
        value = cfg['sensors'][key]
        if not math.isfinite(value) or value < 0:
            raise ValueError(f'sensors.{key} must be finite and nonnegative')
    if len(cfg['motors']['gains']) != 2 or not all(math.isfinite(x) and x > 0 for x in cfg['motors']['gains']):
        raise ValueError('motors.gains must contain two positive finite gains')
    if cfg['sensors']['encoder_hz'] > 1 / cfg['gazebo']['step_size']:
        raise ValueError('Encoder rate cannot exceed the physics update rate')
    if cfg['sensors']['encoder_hz'] + cfg['sensors']['imu_hz'] >= 1 / cfg['gazebo']['step_size']:
        raise ValueError('Combined IMU/encoder rate must be below the physics update rate')
    cfg['slam']['parameter_file'] = str((path.parent / cfg['slam']['parameter_file']).resolve())
    for component in cfg['components'].values():
        component['arguments'] = [str(arg).replace('{config_dir}', str(path.parent))
                                  for arg in component.get('arguments', [])]
        if 'source' in component:
            component['source'] = str((path.parent / component['source']).resolve())
        component['parameter_files'] = [str((path.parent / p).resolve())
                                        for p in component.get('parameter_files', [])]
    cfg['gazebo']['aws_house_dir'] = str((path.parent / cfg['gazebo']['aws_house_dir']).resolve())
    if cfg['gazebo']['world_file']:
        cfg['gazebo']['world_file'] = str((path.parent / cfg['gazebo']['world_file']).resolve())
    return cfg


def tag(parent, element, value=None, **attrs):
    child = ET.SubElement(parent, element, attrs)
    if value is not None:
        child.text = str(value)
    return child


def plugin(parent, kind):
    # CamelCase system names have kebab-case shared libraries.
    import re
    filename = re.sub(r'(?<!^)(?=[A-Z])', '-', kind).lower()
    return tag(parent, 'plugin', filename=f'gz-sim-{filename}-system',
               name=f'gz::sim::systems::{kind}')


def robot_xml(description, cfg):
    root = ET.fromstring(description)
    s, m = cfg['sensors'], cfg['motors']
    # Remove the blueprint's ideal DiffDrive, old IMU/lidar and pose publishers.
    for gazebo in root.findall('gazebo'):
        for child in list(gazebo):
            if child.tag in ('sensor', 'plugin'):
                gazebo.remove(child)
    # Same wheel geometry as the supplied description, configurable for calibration.
    for side, sign in [('left', 1), ('right', -1)]:
        joint = root.find(f"joint[@name='{side}_wheel_joint']")
        joint.find('origin').set('xyz', f"0 {sign*m['wheel_base']/2} {m['wheel_radius']}")
        tag(joint, 'dynamics', damping=str(m['joint_damping']), friction='0')
        root.find(f"link[@name='{side}_wheel']/collision/geometry/cylinder").set('radius', str(m['wheel_radius']))
        gz = tag(root, 'gazebo', reference=f'{side}_wheel')
        tag(gz, 'mu1', m['wheel_friction']); tag(gz, 'mu2', m['wheel_friction'])
        ctrl = plugin(tag(root, 'gazebo'), 'JointController')
        for key, value in {'joint_name': f'{side}_wheel_joint',
                           'topic': f'/sim/{side}/cmd', 'use_force_commands': 'true',
                           'p_gain': m['p_gain'], 'cmd_max': m['torque_limit'],
                           'cmd_min': -m['torque_limit']}.items():
            tag(ctrl, key, value)
    state = plugin(tag(root, 'gazebo'), 'JointStatePublisher')
    tag(state, 'topic', '/sim/joints')
    for side in ['left', 'right']:
        tag(state, 'joint_name', f'{side}_wheel_joint')
    truth = plugin(tag(root, 'gazebo'), 'OdometryPublisher')
    for key, value in {'odom_topic': '/sim/ground_truth', 'odom_frame': 'world',
                       'robot_base_frame': 'base_link', 'dimensions': 3,
                       'odom_publish_frequency': 50, 'tf_topic': '/sim/ground_truth_tf'}.items():
        tag(truth, key, value)
    # Fix the blueprint caster's penetration (bottom was 9 mm below the floor).
    root.find("joint[@name='caster_wheel_joint']/origin").set('xyz', '0.075 0 0.0054')
    # Physical LD06 mount is inverted; simulation uses an explicit upright scan frame.
    joint = tag(root, 'joint', name='sim_laser_joint', type='fixed')
    tag(joint, 'parent', link='base_link'); tag(joint, 'child', link='laser_scan')
    tag(joint, 'origin', xyz='0.075 0 0.2156', rpy='0 0 0')
    tag(root, 'link', name='laser_scan')
    sensor = tag(tag(root, 'gazebo', reference='laser_scan'), 'sensor', name='ld06', type='gpu_lidar')
    for k, v in {'topic': '/sim/scan', 'gz_frame_id': 'laser_scan', 'update_rate': s['lidar_hz'], 'always_on': 'true'}.items():
        tag(sensor, k, v)
    lidar = tag(sensor, 'lidar')
    horizontal = tag(tag(lidar, 'scan'), 'horizontal')
    for k, v in {'samples': 720, 'resolution': 1, 'min_angle': -math.pi, 'max_angle': math.pi}.items():
        tag(horizontal, k, v)
    ranges = tag(lidar, 'range')
    for k, v in {'min': 0.1, 'max': 12, 'resolution': 0.01}.items():
        tag(ranges, k, v)
    noise = tag(lidar, 'noise'); tag(noise, 'type', 'gaussian'); tag(noise, 'stddev', s['lidar_stddev'])
    sensor = tag(tag(root, 'gazebo', reference='imu'), 'sensor', name='bno055', type='imu')
    for k, v in {'topic': '/sim/imu', 'gz_frame_id': 'imu', 'update_rate': s['imu_hz'], 'always_on': 'true'}.items():
        tag(sensor, k, v)
    imu = tag(sensor, 'imu')
    for field, stddev in [('angular_velocity', s['gyro_stddev']), ('linear_acceleration', s['accel_stddev'])]:
        axes = tag(imu, field)
        for axis in 'xyz':
            noise = tag(tag(axes, axis), 'noise', type='gaussian')
            tag(noise, 'stddev', stddev)
            if field == 'angular_velocity':
                for key, value in {'bias_mean': s['gyro_bias'], 'bias_stddev': s['gyro_bias_stddev'],
                                   'dynamic_bias_stddev': s['gyro_dynamic_bias_stddev'],
                                   'dynamic_bias_correlation_time': s['gyro_correlation_time']}.items():
                    tag(noise, key, value)
    joint = tag(root, 'joint', name='sim_camera_optical_joint', type='fixed')
    tag(joint, 'parent', link='realsense_link'); tag(joint, 'child', link='camera_color_optical_frame')
    tag(joint, 'origin', xyz='0 0 0', rpy=f'{-math.pi/2} 0 {-math.pi/2}')
    tag(root, 'link', name='camera_color_optical_frame')
    sensor = tag(tag(root, 'gazebo', reference='realsense_link'), 'sensor', name='d435i', type='rgbd_camera')
    for k, v in {'topic': '/sim/rgbd', 'gz_frame_id': 'realsense_link',
                 'update_rate': s['rgbd_hz'], 'always_on': 'true'}.items():
        tag(sensor, k, v)
    camera = tag(sensor, 'camera')
    tag(camera, 'horizontal_fov', s['camera_fov'])
    tag(camera, 'optical_frame_id', 'camera_color_optical_frame')
    im = tag(camera, 'image')
    for k, v in {'width': s['width'], 'height': s['height'], 'format': 'R8G8B8'}.items():
        tag(im, k, v)
    clip = tag(camera, 'clip'); tag(clip, 'near', s['camera_near']); tag(clip, 'far', s['camera_far'])
    noise = tag(camera, 'noise'); tag(noise, 'type', 'gaussian'); tag(noise, 'stddev', s['camera_noise'])
    return ET.tostring(root, encoding='unicode')


def house_xml(cfg):
    """Load the original AWS scene and add Gazebo Harmonic systems only."""
    assets = Path(cfg['gazebo']['aws_house_dir'])
    source = assets / 'worlds/small_house.world'
    if not source.is_file():
        raise FileNotFoundError(f'{source} missing; run ./scripts/sim.sh assets')
    root = ET.parse(source).getroot()
    world = root.find('world')
    world.set('name', 'house')
    # Classic GUI camera settings do not configure the Harmonic GUI.
    for element in list(world):
        if element.tag in ('physics', 'gui', 'plugin'):
            world.remove(element)
    physics = tag(world, 'physics', name='physics', type='ignored')
    tag(physics, 'max_step_size', cfg['gazebo']['step_size'])
    tag(physics, 'real_time_factor', cfg['gazebo']['real_time_factor'])
    for kind in ['Physics', 'UserCommands', 'SceneBroadcaster', 'Imu', 'Sensors']:
        system = plugin(world, kind)
        if kind == 'Sensors':
            tag(system, 'render_engine', 'ogre2')
    world.find('gravity').text = '0 0 -9.80665'
    # Classic accepts some invalid inertias in static AWS furniture; Harmonic
    # validates them even when static. Inline static includes and omit inertial
    # data only. Preserve meshes, materials, collisions and model placement.
    for instance in world.findall('model'):
        if instance.findtext('static', '').strip() != 'true':
            raise ValueError(f'AWS instance {instance.get("name")} must be static')
        for include in list(instance.findall('include')):
            uri = include.findtext('uri', '')
            if not uri.startswith('model://'):
                raise ValueError(f'Unexpected AWS include: {uri}')
            nested = ET.parse(assets / 'models' / uri.removeprefix('model://') / 'model.sdf').getroot().find('model')
            for link in nested.iter('link'):
                for inertial in list(link.findall('inertial')):
                    link.remove(inertial)
            static = nested.find('static')
            if static is None:
                tag(nested, 'static', 'true')
            else:
                static.text = 'true'
            instance.remove(include)
            instance.append(nested)
    return ET.tostring(root, encoding='unicode')


def bridges(cfg):
    t = cfg['topics']
    rows = [('clock', '/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'Clock'),
            ('truth', '/sim/ground_truth', '/sim/ground_truth', 'nav_msgs/msg/Odometry', 'Odometry'),
            ('joints', '/sim/joints', '/sim/joints', 'sensor_msgs/msg/JointState', 'Model'),
            ('imu', '/sim/imu', '/sim/imu_raw', 'sensor_msgs/msg/Imu', 'IMU'),
            ('scan', '/sim/scan', t['scan'], 'sensor_msgs/msg/LaserScan', 'LaserScan'),
            ('rgb', '/sim/rgbd/image', t['rgb'], 'sensor_msgs/msg/Image', 'Image'),
            ('depth', '/sim/rgbd/depth_image', t['depth'], 'sensor_msgs/msg/Image', 'Image'),
            ('info', '/sim/rgbd/camera_info', t['camera_info'], 'sensor_msgs/msg/CameraInfo', 'CameraInfo'),
            ('points', '/sim/rgbd/points', t['points'], 'sensor_msgs/msg/PointCloud2', 'PointCloudPacked')]
    result = [dict(gz_topic_name=gz, ros_topic_name=ros, ros_type_name=rt,
                   gz_type_name=f'gz.msgs.{gt}', direction='GZ_TO_ROS',
                   qos_profile='CLOCK' if key == 'clock' else 'SENSOR_DATA')
              for key, gz, ros, rt, gt in rows]
    for side in ['left', 'right']:
        result.append(dict(topic_name=f'/sim/{side}/cmd', ros_type_name='std_msgs/msg/Float64',
                           gz_type_name='gz.msgs.Double', direction='ROS_TO_GZ'))
    return result
