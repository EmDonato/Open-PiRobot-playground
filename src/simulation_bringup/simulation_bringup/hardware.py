"""Motor dynamics and encoder measurements at the physical driver's ROS boundary."""
import math
import random
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64
from robot_interfaces.srv import ControlCommand

from .model import load_config


def limit_targets(v, w, motors):
    targets = [(v - w * motors['wheel_base'] / 2) / motors['wheel_radius'],
               (v + w * motors['wheel_base'] / 2) / motors['wheel_radius']]
    scale = max(1.0, max(abs(x) for x in targets) / motors['max_wheel_rad_s'])
    return [x / scale for x in targets]


class HardwareAdapter(Node):
    def __init__(self):
        super().__init__('sim_hardware')
        cfg = load_config(self.declare_parameter('config_file', '').value)
        self.m, self.s, t = cfg['motors'], cfg['sensors'], cfg['topics']
        self.rng = random.Random(cfg['gazebo']['seed'])
        self.target = [0.0, 0.0]
        self.motor = [0.0, 0.0]
        self.armed = self.m['start_armed']
        self.command_time = None
        self.control_time = None
        self.encoder_previous = None
        self.queue = []
        self.sequence = 0
        self.last_sent = -1.0
        self.pub = [self.create_publisher(Float64, f'/sim/{side}/cmd', 10)
                    for side in ['left', 'right']]
        self.accepted = self.create_publisher(Twist, t['accepted'], 10)
        self.encoder = self.create_publisher(TwistStamped, t['encoder'], qos_profile_sensor_data)
        self.imu = self.create_publisher(Imu, t['imu'], qos_profile_sensor_data)
        self.joints = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.create_subscription(Twist, t['command'], self.command, 10)
        self.create_subscription(JointState, '/sim/joints', self.measure, qos_profile_sensor_data)
        self.create_subscription(Imu, '/sim/imu_raw', self.measure_imu, qos_profile_sensor_data)
        self.create_service(ControlCommand, t['control_service'], self.control)
        self.create_timer(1 / self.m['control_hz'], self.tick)
        self.create_timer(cfg['gazebo']['step_size'], self.flush_measurement)

    def flush_measurement(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # Release at most one sample per physics tick: avoid mixed-topic bursts
        # which a downstream single-threaded executor can process in reverse order.
        if self.queue and self.queue[0][0] <= now - self.s['reorder_window']:
            stamp, _, publisher, message = heapq.heappop(self.queue)
            if stamp >= self.last_sent:
                publisher.publish(message)
                self.last_sent = stamp

    def command(self, msg):
        if not all(math.isfinite(x) for x in [msg.linear.x, msg.angular.z]):
            self.target = [0.0, 0.0]
            self.command_time = None
            return
        self.target = limit_targets(msg.linear.x, msg.angular.z, self.m)
        self.command_time = self.get_clock().now().nanoseconds * 1e-9

    def control(self, request, response):
        if request.command == ControlCommand.Request.ARM_TOGGLE:
            self.armed = not self.armed
            self.get_logger().info("Motors ARMED" if self.armed else "Motors DISARMED")
        elif request.command == ControlCommand.Request.RESET:
            # Hardware reset clears motor references; it does not reset Gazebo/EKF time.
            self.motor = [0.0, 0.0]
        else:
            response.success = False
            return response
        self.target = [0.0, 0.0]
        self.command_time = None
        response.success = True
        return response

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0 if self.control_time is None else now - self.control_time
        self.control_time = now
        if dt <= 0:
            return
        live = self.armed and self.command_time is not None and 0 <= now - self.command_time <= self.m['command_timeout']
        targets = self.target if live else [0.0, 0.0]
        accepted = Twist()
        accepted.linear.x = sum(targets) * self.m['wheel_radius'] / 2
        accepted.angular.z = (targets[1] - targets[0]) * self.m['wheel_radius'] / self.m['wheel_base']
        self.accepted.publish(accepted)
        for i, target in enumerate(targets):
            target *= self.m['gains'][i]
            if abs(target) < self.m['deadband_rad_s']:
                target = 0.0
            change = (target - self.motor[i]) * (1 - math.exp(-dt / self.m['time_constant']))
            maximum = self.m['acceleration_rad_s2'] * dt
            self.motor[i] += max(-maximum, min(maximum, change))
            self.pub[i].publish(Float64(data=self.motor[i]))

    def measure(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            positions = [msg.position[msg.name.index(f'{side}_wheel_joint')]
                         for side in ['left', 'right']]
        except (ValueError, IndexError):
            return
        tick = 2 * math.pi / self.s['encoder_ticks_per_rev']
        positions = [round(p / tick) * tick for p in positions]
        if self.encoder_previous is None or stamp < self.encoder_previous[0]:
            self.encoder_previous = (stamp, positions)
            return
        dt = stamp - self.encoder_previous[0]
        if dt + 1e-9 < 1 / self.s['encoder_hz']:
            return
        speed = [(p - old) / dt + self.rng.gauss(0, self.s['encoder_stddev'])
                 for p, old in zip(positions, self.encoder_previous[1])]
        self.encoder_previous = (stamp, positions)
        measured = TwistStamped()
        measured.header = msg.header
        measured.header.frame_id = 'base_link'
        measured.twist.linear.x = sum(speed) * self.m['wheel_radius'] / 2
        measured.twist.angular.z = (speed[1] - speed[0]) * self.m['wheel_radius'] / self.m['wheel_base']
        self.enqueue(self.encoder, measured)
        joints = JointState(header=msg.header, name=['left_wheel_joint', 'right_wheel_joint'],
                            position=positions, velocity=speed)
        self.joints.publish(joints)

    def enqueue(self, publisher, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.sequence += 1
        heapq.heappush(self.queue, (stamp, self.sequence, publisher, message))
        # Bound memory even if a custom world stops /clock or publishes bad stamps.
        if len(self.queue) > 2000:
            heapq.heappop(self.queue)

    def measure_imu(self, message):
        message.orientation_covariance[0] = -1.0  # Match physical driver's unavailable orientation.
        for index in [0, 4, 8]:
            message.angular_velocity_covariance[index] = self.s['gyro_stddev'] ** 2
            message.linear_acceleration_covariance[index] = self.s['accel_stddev'] ** 2
        self.enqueue(self.imu, message)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
