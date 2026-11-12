"""Read-only checks of the configured sensor inputs and navigation TF chain."""

import argparse
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import load_hardware


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware-config', default=(
        get_package_share_directory('tracer_navigation') + '/config/hardware.yaml'))
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--navigation', action='store_true', help='also require map -> odom')
    options = parser.parse_args(args)
    if not 0 < options.duration <= 60:
        parser.error('--duration must be between 0 and 60 seconds')
    hardware = load_hardware(options.hardware_config)
    frames, topics = hardware['frames'], hardware['topics']
    rclpy.init(args=[])
    node = rclpy.create_node('tracer_navigation_doctor')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    seen = {key: [0, None] for key in ('cloud', 'imu', 'odom')}

    def record(key, msg):
        seen[key][0] += 1
        seen[key][1] = msg

    for key, message_type in [('cloud', PointCloud2), ('imu', Imu), ('odom', Odometry)]:
        node.create_subscription(message_type, topics[key],
                                 lambda msg, key=key: record(key, msg),
                                 qos_profile_sensor_data)
    try:
        end = time.monotonic() + options.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        passed = True
        for key, (count, message) in seen.items():
            expected_frame = frames['odom'] if key == 'odom' else frames[key if key == 'imu' else 'lidar']
            ok = count > 0 and message.header.frame_id == expected_frame
            if key == 'odom' and message is not None:
                ok = ok and message.child_frame_id == frames['base']
            print(f'{"OK" if ok else "FAIL"}: {topics[key]}: '
                  f'{count} messages, expected frame {expected_frame}')
            passed &= ok
        edges = [(frames['odom'], frames['base']), (frames['base'], frames['lidar'])]
        if options.navigation:
            edges.append((frames['map'], frames['odom']))
        for parent, child in edges:
            try:
                transform = buffer.lookup_transform(parent, child, Time())
                stamp = transform.header.stamp
                print(f'OK: TF {parent} -> {child}, stamp={stamp.sec}.{stamp.nanosec:09d}')
            except TransformException as error:
                print(f'FAIL: TF {parent} -> {child}: {error}')
                passed = False
        subscribers = node.count_subscribers(topics['cmd_vel'])
        print(f'{topics["cmd_vel"]}: {subscribers} subscribers (Tracer should subscribe)')
        passed &= subscribers > 0
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
