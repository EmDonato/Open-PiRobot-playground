"""Bounded live check: sensor streams, clock, EKF and physical wheel response."""
import collections
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from simulation_bringup.model import load_config


def main():
    cfg = load_config(os.environ['PIROBOT_SIM_CONFIG'])
    t = cfg['topics']
    log_path = Path(os.environ['ROS_LOG_DIR']).parent / 'smoke.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('simulation_probe')
    counts, samples, stamps = collections.Counter(), {}, collections.defaultdict(list)
    map_to_odom = []
    def receive(key, msg):
        if key == 'tf':
            map_to_odom.extend(t for t in msg.transforms
                               if t.header.frame_id == 'map' and t.child_frame_id == 'odom')
        counts[key] += 1
        samples[key] = msg
        if hasattr(msg, 'header'):
            stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    subscriptions = []
    for key, topic, kind in [('clock', '/clock', Clock), ('imu', t['imu'], Imu),
                             ('scan', t['scan'], LaserScan), ('rgb', t['rgb'], Image),
                             ('depth', t['depth'], Image), ('info', t['camera_info'], CameraInfo),
                             ('encoder', t['encoder'], TwistStamped), ('odom', '/odom', Odometry),
                             ('truth', '/sim/ground_truth', Odometry),
                             ('map', '/map', OccupancyGrid),
                             ('tf', '/tf', TFMessage)]:
        subscriptions.append(node.create_subscription(kind, topic,
                             lambda msg, k=key: receive(k, msg), qos_profile_sensor_data))
    pub = node.create_publisher(Twist, t['command'], 10)
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'simulation_bringup', 'simulation.launch.py',
                                        'config:=' + os.environ['PIROBOT_SIM_CONFIG'], 'gui:=false', 'rviz:=false', 'joystick:=false'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 65
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.1)
                if all(counts[k] >= 3 for k in ['clock', 'imu', 'scan', 'rgb', 'depth', 'info', 'encoder', 'odom', 'tf', 'truth', 'map']):
                    break
            missing = [k for k in ['clock', 'imu', 'scan', 'rgb', 'depth', 'info', 'encoder', 'odom', 'tf', 'truth', 'map'] if counts[k] < 3]
            if missing:
                raise AssertionError(f'Missing streams: {missing}; counts={dict(counts)}; log={log_path}')
            assert map_to_odom, 'SLAM did not publish map -> odom'
            grid = samples['map']
            assert grid.header.frame_id == 'map' and grid.info.width > 0
            assert any(value >= 0 for value in grid.data), 'SLAM map is entirely unknown'
            assert samples['scan'].header.frame_id == 'laser_scan'
            assert samples['rgb'].header.frame_id == 'camera_color_optical_frame'
            assert samples['depth'].header.frame_id == 'camera_color_optical_frame'
            assert samples['imu'].header.frame_id == 'imu'
            assert samples['odom'].child_frame_id == 'base_link'
            assert samples['depth'].encoding == '32FC1'
            # Keep a camera view for visual inspection of the actual world assets.
            rgb = samples['rgb']
            if rgb.encoding == 'rgb8':
                pixels = bytes(rgb.data)
                packed = b''.join(pixels[row*rgb.step:row*rgb.step+rgb.width*3]
                                  for row in range(rgb.height))
                (log_path.parent / 'smoke_camera.ppm').write_bytes(
                    f'P6\n{rgb.width} {rgb.height}\n255\n'.encode() + packed)
            initial = samples['odom'].pose.pose.position
            initial_truth = samples['truth'].pose.pose.position
            max_encoder = 0.0
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                pub.publish(Twist(linear=Vector3(x=.12)))
                rclpy.spin_once(node, timeout_sec=.05)
                max_encoder = max(max_encoder, abs(samples['encoder'].twist.linear.x))
            final = samples['odom'].pose.pose.position
            distance = math.hypot(final.x-initial.x, final.y-initial.y)
            assert max_encoder > .04, f'Wheels did not respond: {max_encoder}'
            assert distance > .05, f'EKF did not advance: {distance}'
            final_truth = samples['truth'].pose.pose.position
            truth_distance = math.hypot(final_truth.x-initial_truth.x, final_truth.y-initial_truth.y)
            assert truth_distance > .05, f'Robot did not physically move: {truth_distance}'
            assert abs(distance - truth_distance) < .2, f'EKF/physics mismatch: {distance} / {truth_distance}'
            # Command timeout must stop the wheels without another command publisher.
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert abs(samples['encoder'].twist.linear.x) < .02, 'Robot failed to stop'
            max_yaw_rate = 0.0
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                pub.publish(Twist(angular=Vector3(z=.3)))
                rclpy.spin_once(node, timeout_sec=.05)
                max_yaw_rate = max(max_yaw_rate, samples['imu'].angular_velocity.z)
            assert max_yaw_rate > .1, f'IMU yaw sign/rotation incorrect: {max_yaw_rate}'
            pub.publish(Twist())
            rates = {key: (len(values)-1)/(values[-1]-values[0]) for key, values in stamps.items()
                     if len(values) > 1 and values[-1] > values[0]}
            print(f'PASS streams={dict(counts)}, max_encoder={max_encoder:.3f} m/s, odom_distance={distance:.3f} m')
            print(f'Physical displacement={truth_distance:.3f} m; rates in simulation time={rates}')
            print(f'Runtime log: {log_path}')
    finally:
        pub.publish(Twist())
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
