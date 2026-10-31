"""Check live RGB-D geometry, frame conventions and packed colors against images."""
import math
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from simulation_bringup.model import load_config
import os


def main():
    cfg = load_config(os.environ['PIROBOT_SIM_CONFIG'])
    topics = cfg['topics']
    rclpy.init()
    node = rclpy.create_node('camera_alignment_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    messages = {}
    def receive(key, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        messages.setdefault(stamp, {})[key] = msg
        if len(messages) > 40:
            del messages[min(messages)]
    subscriptions = [node.create_subscription(kind, topics[key], lambda msg, k=key: receive(k, msg), qos_profile_sensor_data)
                     for key, kind in [('points', PointCloud2), ('rgb', Image), ('depth', Image), ('camera_info', CameraInfo)]]
    try:
        deadline = time.monotonic() + 20
        packet = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            ready = [data for data in messages.values() if len(data) == 4]
            if ready and buffer.can_transform('realsense_link', 'camera_color_optical_frame', rclpy.time.Time()):
                packet = ready[-1]
                break
        assert packet, 'No synchronized cloud, depth, RGB and camera info received'
        cloud, rgb, depth, info = [packet[k] for k in ['points', 'rgb', 'depth', 'camera_info']]
        assert cloud.header.frame_id == 'realsense_link', cloud.header.frame_id
        assert all(m.header.frame_id == 'camera_color_optical_frame' for m in [rgb, depth, info])
        assert rgb.encoding == 'rgb8' and depth.encoding == '32FC1'
        assert (cloud.width, cloud.height) == (rgb.width, rgb.height) == (depth.width, depth.height)
        fields = {f.name: f.offset for f in cloud.fields}
        endian = '>' if cloud.is_bigendian else '<'
        errors = []; colors = []; depths = []; ray_residuals = []; ray_limits = []
        for v in range(cloud.height // 4, cloud.height, cloud.height // 4):
            for u in range(cloud.width // 4, cloud.width, cloud.width // 4):
                base = v*cloud.row_step + u*cloud.point_step
                x,y,z = [struct.unpack_from(endian+'f', cloud.data, base+fields[k])[0] for k in 'xyz']
                if not all(math.isfinite(value) for value in [x,y,z]) or x <= 0:
                    continue
                d = struct.unpack_from(('>' if depth.is_bigendian else '<')+'f', depth.data, v*depth.step+u*4)[0]
                assert math.isfinite(d)
                depths.append(abs(x-d))
                projected_u = info.k[0]*(-y/x)+info.k[2]
                projected_v = info.k[4]*(-z/x)+info.k[5]
                errors.append(max(abs(projected_u-u), abs(projected_v-v)))
                ray_y = (u-info.k[2])/info.k[0]
                ray_z = (v-info.k[5])/info.k[4]
                ray_residuals.append(max(abs(y+ray_y*x), abs(z+ray_z*x)))
                # Harmonic's RGB-D Gaussian shader perturbs XYZ independently.
                # Test geometry in metres, with a noise-derived bound; a fixed
                # pixel threshold incorrectly rejects valid close-range noise.
                sigma = cfg['sensors']['camera_noise']
                ray_limits.append(6*sigma*(1+max(abs(ray_y),abs(ray_z))) + x/min(info.k[0],info.k[4]))
                packed = struct.unpack_from(endian+'I',cloud.data,base+fields['rgb'])[0]
                pc = [(packed>>shift)&255 for shift in (16,8,0)]
                im = rgb.data[v*rgb.step+u*3:v*rgb.step+u*3+3]
                colors.extend(abs(int(a)-int(b)) for a,b in zip(pc,im))
        assert len(errors) >= 3, 'Insufficient valid geometry'
        assert all(e < limit for e,limit in zip(ray_residuals,ray_limits)), f'Cloud ray geometry mismatch: {ray_residuals}'
        assert max(depths) < .03, f'Cloud/depth mismatch: {max(depths)} metres'
        assert sum(colors)/len(colors) < 8, f'Packed RGB mismatch: {colors}'
        transform = buffer.lookup_transform('realsense_link', 'camera_color_optical_frame', rclpy.time.Time()).transform
        q = transform.rotation
        # Rotate optical +Z into the camera body frame; it must be body +X.
        optical_z = [2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)]
        assert max(abs(a-b) for a,b in zip(optical_z,[1,0,0])) < 1e-6
        print(f'PASS cloud frame + optical TF; max noisy ray residual={max(ray_residuals):.6f}m, depth error={max(depths):.6f}m, mean RGB error={sum(colors)/len(colors):.2f}/255')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
