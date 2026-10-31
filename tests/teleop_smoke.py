"""Exercise the real sandbox command pipeline using synthetic SDL Joy messages."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def main():
    rclpy.init()
    node = rclpy.create_node('teleop_probe')
    received = []
    sub = node.create_subscription(Twist, '/teleop_test/cmd_vel', lambda msg: received.append(msg), 10)
    publisher = node.create_publisher(Joy, '/teleop_test/joy', 10)
    common = ['--ros-args', '-r', '__ns:=/teleop_test']
    commands = [
        ['ros2', 'run', 'teleop_twist_joy', 'teleop_node', *common,
         '-p', 'enable_button:=9', '-p', 'axis_linear.x:=1', '-p', 'scale_linear.x:=1.0',
         '-p', 'axis_angular.yaw:=2', '-p', 'scale_angular.yaw:=1.0',
         '-p', 'publish_stamped_twist:=true', '-r', 'cmd_vel:=cmd_vel/raw'],
        ['ros2', 'run', 'joy_teleop_twist_ref', 'joy_vel_ref', *common,
         '-p', 'max_vel:=0.2', '-p', 'wheel_base:=0.2', '-r', 'cmd_vel:=stamped'],
        ['ros2', 'run', 'simulation_bringup', 'stamped_to_twist', *common,
         '-r', 'input:=stamped', '-r', 'output:=cmd_vel'],
    ]
    processes = []
    log_path = Path(os.environ['ROS_LOG_DIR']).parent / 'teleop_smoke.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with log_path.open('w') as log:
            processes = [subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True) for cmd in commands]
            deadline = time.monotonic() + 10
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            def check(linear, angular, enabled, expected_v, expected_w):
                received.clear()
                deadline = time.monotonic() + 4
                next_send = 0
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_send:
                        msg = Joy()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.axes = [0.0, linear, angular, 0.0, 0.0, 0.0]
                        msg.buttons = [0] * 15
                        msg.buttons[9] = int(enabled)
                        publisher.publish(msg)
                        next_send = now + .04
                    rclpy.spin_once(node, timeout_sec=.02)
                    if received and abs(received[-1].linear.x-expected_v) < .001 and abs(received[-1].angular.z-expected_w) < .001:
                        return
                raise AssertionError(f'Expected Twist v={expected_v}, w={expected_w}; received={received[-1:]}')
            check(.5, 0.0, True, .1, 0.0)
            check(0.0, .5, True, 0.0, 1.0)
            check(.5, .5, False, 0.0, 0.0)
            print('PASS Joy -> teleop TwistStamped -> sandbox wheel mixing -> Twist; forward, yaw, deadman release')
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
