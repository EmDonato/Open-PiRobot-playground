"""Exercise terminal WASD or the full joystick pipeline against Gazebo physics."""
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import time
import math

import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from simulation_bringup.model import load_config

mode = sys.argv[1]
assert mode in ('keyboard', 'joystick')
root = Path(__file__).resolve().parents[1]
cfg = load_config(os.environ['PIROBOT_SIM_CONFIG'])
cfg['gazebo']['gui'] = False
cfg['components']['rviz']['enabled'] = False
# Synthetic SDL input replaces only the physical device; all command nodes remain.
cfg['components']['joystick']['enabled'] = False
rclpy.init()
node = rclpy.create_node('control_motion_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
state = {}
def truth(msg):
    state['position'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
def accepted(msg):
    state['speed'] = msg.linear.x
subscriptions = [node.create_subscription(Odometry, '/sim/ground_truth', truth, qos_profile_sensor_data),
                 node.create_subscription(Twist, cfg['topics']['accepted'], accepted, 10)]
publisher = node.create_publisher(Joy, '/joy', 10)
master, slave = pty.openpty()
os.set_blocking(master, False)
proc = None
log = bytearray()
def spin():
    rclpy.spin_once(node, timeout_sec=.02)
    try:
        log.extend(os.read(master, 65536))
    except (BlockingIOError, OSError):
        pass
try:
    with tempfile.TemporaryDirectory() as tmp:
        config = Path(tmp) / 'simulation.yaml'
        config.write_text(yaml.safe_dump(cfg))
        env = dict(os.environ, PIROBOT_SIM_CONFIG=str(config))
        proc = subprocess.Popen(['bash', str(root / 'scripts/sim.sh'), 'up', '--teleop', mode],
                                env=env, stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        deadline = time.monotonic() + 65
        while 'position' not in state and time.monotonic() < deadline and proc.poll() is None:
            spin()
        assert 'position' in state, 'No physics data; ' + log.decode(errors='replace')[-3000:]
        initial = state['position']
        peak = 0.
        deadline = time.monotonic() + 7
        while time.monotonic() < deadline:
            if mode == 'keyboard':
                os.write(master, b'w')
            else:
                msg = Joy()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.axes = [0., .7, 0., 0., 0., 0.]
                msg.buttons = [0] * 21
                msg.buttons[9] = 1
                publisher.publish(msg)
            spin()
            peak = max(peak, state.get('speed', 0.))
            time.sleep(.04)
        distance = math.dist(initial, state['position'])
        assert peak > .1 and distance > .05, f'{mode}: speed={peak}, physical distance={distance}'
        if mode == 'keyboard':
            os.write(master, b' ')
        # Release keyboard / stop sending Joy, exercising the motor watchdog too.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            spin()
        assert abs(state['speed']) < .001, f'{mode}: failed to stop: {state}'
        print(f'PASS {mode}: accepted speed {peak:.3f} m/s, physical movement {distance:.3f} m, stopped after release')
finally:
    if proc and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    os.close(master)
    os.close(slave)
    node.destroy_node()
    rclpy.shutdown()
