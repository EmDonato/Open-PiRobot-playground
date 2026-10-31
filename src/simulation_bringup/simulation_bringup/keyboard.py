"""Terminal WASD control with repeated commands and a wall-clock deadman."""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .model import load_config


class KeyboardCommand:
    """Keep a key active through terminal autorepeat, then stop on inactivity."""
    def __init__(self, speed=0.2, turn=1.0, timeout=0.6):
        self.speed, self.turn, self.timeout = speed, turn, timeout
        self.key, self.updated = '', float('-inf')

    def press(self, key, now):
        self.key = key.lower() if key.lower() in 'wasd' else ''
        self.updated = now

    def velocity(self, now):
        if now - self.updated > self.timeout:
            return 0.0, 0.0
        return {'w': (self.speed, 0.0), 's': (-self.speed, 0.0),
                'a': (0.0, self.turn), 'd': (0.0, -self.turn)}.get(self.key, (0.0, 0.0))


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('WASD richiede un terminale interattivo.')
    rclpy.init(args=args)
    node = Node('keyboard_wasd')
    cfg = load_config(node.declare_parameter('config_file', '').value)
    publisher = node.create_publisher(Twist, cfg['topics']['command'], 10)
    teleop = cfg['teleop']['keyboard']
    controls = KeyboardCommand(teleop['linear_speed'], teleop['angular_speed'], teleop['release_timeout'])
    original = termios.tcgetattr(sys.stdin)
    print('WASD: W avanti, S indietro, A sinistra, D destra.\n'
          'Tieni premuto per muoverti; spazio ferma subito, Ctrl-C esce.\n'
          f'Il rilascio ferma entro {controls.timeout:g} s. Mantieni il focus su questo terminale.', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = sys.stdin.read(1)
                if not key or key == '\x03':
                    break
                controls.press(key, time.monotonic())
            linear, angular = controls.velocity(time.monotonic())
            message = Twist()
            message.linear.x, message.angular.z = linear, angular
            publisher.publish(message)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        if rclpy.ok():
            publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
