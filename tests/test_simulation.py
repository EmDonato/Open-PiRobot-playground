import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import xacro
from simulation_bringup.model import load_config, robot_xml, house_xml, bridges
from simulation_bringup.hardware import limit_targets

ROOT = Path(__file__).resolve().parents[1]


class SimulationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config(ROOT / 'config/simulation.yaml')
        cls.robot = ET.fromstring(robot_xml(xacro.process_file(str(
            ROOT / 'src/robot_descriptor/descriptor/robot.urdf.xacro')).toxml(), cls.cfg))

    def test_no_competing_odometry(self):
        plugins = self.robot.findall('.//plugin')
        self.assertFalse(any('DiffDrive' in p.get('name') or 'PosePublisher' in p.get('name') for p in plugins))
        self.assertFalse(any(b.get('ros_topic_name') in ['/odom', '/tf'] for b in bridges(self.cfg)))
        motors = [p for p in plugins if 'JointController' in p.get('name')]
        self.assertEqual(len(motors), 2)
        for motor in motors:
            self.assertEqual(motor.findtext('use_force_commands'), 'true')
            self.assertAlmostEqual(float(motor.findtext('cmd_max')), self.cfg['motors']['torque_limit'])

    def test_sensor_rates_and_frames(self):
        for name, rate in [('ld06', 'lidar_hz'), ('bno055', 'imu_hz'), ('d435i', 'rgbd_hz')]:
            sensor = self.robot.find(f".//sensor[@name='{name}']")
            self.assertEqual(float(sensor.findtext('update_rate')), self.cfg['sensors'][rate])
            self.assertIsNotNone(self.robot.find(f"link[@name='{sensor.findtext('gz_frame_id')}']"))
        self.assertEqual(self.robot.find("joint[@name='sim_laser_joint']/origin").get('rpy'), '0 0 0')

    def test_command_saturation_preserves_curvature(self):
        m = self.cfg['motors']
        left, right = limit_targets(10, 5, m)
        self.assertLessEqual(max(abs(left), abs(right)), m['max_wheel_rad_s'])
        self.assertAlmostEqual(left/right, (10-5*m['wheel_base']/2)/(10+5*m['wheel_base']/2))
        left, right = limit_targets(0, 1, m)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_aws_house_preserves_original_scene(self):
        original = ET.parse(Path(self.cfg['gazebo']['aws_house_dir']) / 'worlds/small_house.world').getroot()
        adapted = ET.fromstring(house_xml(self.cfg))
        self.assertEqual(len(adapted.findall('./world/model')), 87)
        self.assertEqual([ET.tostring(m) for m in original.findall('./world/light')],
                         [ET.tostring(m) for m in adapted.findall('./world/light')])
        for before, after in zip(original.findall('./world/model'), adapted.findall('./world/model')):
            self.assertEqual(before.get('name'), after.get('name'))
            self.assertEqual(before.findtext('pose'), after.findtext('pose'))
            uri = before.findtext('include/uri')
            model = Path(self.cfg['gazebo']['aws_house_dir']) / 'models' / uri.removeprefix('model://') / 'model.sdf'
            source = ET.parse(model).getroot().find('model')
            nested = after.find('model')
            for kind in ['collision', 'visual']:
                self.assertEqual([ET.tostring(e) for e in source.findall(f'.//{kind}')],
                                 [ET.tostring(e) for e in nested.findall(f'.//{kind}')])
            self.assertFalse(nested.findall('.//inertial'))
        self.assertEqual(len(adapted.findall('./world/plugin')), 5)

    def test_aws_texture_files_resolve(self):
        models = Path(self.cfg['gazebo']['aws_house_dir']) / 'models'
        images = 0
        world = ET.parse(Path(self.cfg['gazebo']['aws_house_dir']) / 'worlds/small_house.world')
        meshes = set()
        for include in world.findall('.//include/uri'):
            model = ET.parse(models / include.text.removeprefix('model://') / 'model.sdf')
            for uri in model.findall('.//visual/geometry/mesh/uri'):
                meshes.add(models / uri.text.removeprefix('model://'))
        for mesh in meshes:
            document = ET.parse(mesh)
            for entry in document.findall('.//{*}library_images/{*}image/{*}init_from'):
                path = mesh.parent / entry.text.strip()
                self.assertTrue(path.is_file(), f'Missing texture: {path}')
                images += 1
        self.assertGreater(images, 50)



if __name__ == '__main__':
    unittest.main()
