#!/usr/bin/env python3
"""Check map -> odom and calibrated sensor TF using PCD-derived synthetic scans.

Runs the combined navigation launch without hardware in localhost domain 174.
This verifies configuration and transform arithmetic, not field accuracy.
"""

import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from smoke_navigation import SERVERS, spin_until, stop_process
from tracer_navigation.configuration import load_hardware


def main():
    os.environ['ROS_DOMAIN_ID'] = '174'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    share = Path(get_package_share_directory('tracer_navigation'))
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp':
        os.environ['CYCLONEDDS_URI'] = (share / 'config/cyclonedds.xml').as_uri()
    hardware = load_hardware(share / 'config/hardware.yaml')
    root = Path(tempfile.mkdtemp(prefix='tracer_localization_smoke_'))
    os.environ['ROS_LOG_DIR'] = str(root / 'logs')
    print(f'PCD-derived synthetic scan test artifacts: {root}', flush=True)
    with Path(hardware['maps']['pcd']).open('rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('unexpected PCD header')
            header.append(line.decode().strip())
            if line.startswith(b'DATA'):
                assert line.strip() == b'DATA binary'
                break
        assert 'FIELDS x y z intensity normal_x normal_y normal_z curvature' in header
        points = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 8)[:, :3]
    # Known map -> base pose and a different wheel odom -> base pose.
    map_base = np.array([1.0, 0.5, 0.0])
    map_yaw, wheel_yaw = 0.2, -0.1
    wheel_base = np.array([0.4, -0.2, 0.0])
    rotation = np.array([[math.cos(map_yaw), -math.sin(map_yaw), 0],
                         [math.sin(map_yaw), math.cos(map_yaw), 0], [0, 0, 1]])
    scan = (points - map_base) @ rotation
    extrinsic = hardware['sensor_transform']
    assert all(extrinsic[k] == 0 for k in ('roll', 'pitch', 'yaw'))
    scan -= np.array([extrinsic[k] for k in ('x', 'y', 'z')])
    scan = scan[np.isfinite(scan).all(axis=1) &
                (np.linalg.norm(scan[:, :2], axis=1) > 1.2) &
                (np.linalg.norm(scan[:, :2], axis=1) < 15)]
    scan = scan[::max(1, len(scan) // 5000)]
    assert len(scan) > 100
    packed = np.column_stack((scan, np.full(len(scan), 30))).astype('<f4').tobytes()
    log = (root / 'bringup.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'tracer_navigation', 'navigation.launch.py',
        'start_base:=false', 'start_lidar:=false', 'rviz:=false',
        'auto_initialize:=false', f'state_file:={root / "state.json"}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('tracer_localization_mock_hardware')
    broadcaster = TransformBroadcaster(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    healthy = []

    def on_alignment(msg):
        for status in msg.status:
            values = {value.key: value.value for value in status.values}
            if values.get('failure_category') == 'healthy':
                healthy.append(values)

    node.create_subscription(DiagnosticArray, '/alignment_status', on_alignment,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    tick_count = [0]

    def tick():
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = wheel_base[:2]
        odom.pose.pose.orientation.z = math.sin(wheel_yaw / 2)
        odom.pose.pose.orientation.w = math.cos(wheel_yaw / 2)
        odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = wheel_base[:2]
        transform.transform.rotation = odom.pose.pose.orientation
        broadcaster.sendTransform(transform)
        tick_count[0] += 1
        if tick_count[0] % 2 == 0:
            return
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, 'livox_frame'
        cloud.height, cloud.width = 1, len(scan)
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.point_step, cloud.row_step, cloud.is_dense = 16, len(scan) * 16, True
        cloud.data = packed
        cloud_pub.publish(cloud)
        imu = Imu()
        imu.header = cloud.header
        imu.linear_acceleration.z = 9.81
        imu_pub.publish(imu)

    timer = node.create_timer(0.05, tick)
    try:
        client = node.create_client(GetState, '/lidar_localization/get_state')
        assert spin_until(node, client.service_is_ready, 15)
        pending_state = [None]

        def localizer_active():
            if pending_state[0] is None:
                pending_state[0] = client.call_async(GetState.Request())
            elif pending_state[0].done():
                if pending_state[0].result().current_state.label == 'active':
                    return True
                pending_state[0] = None
            return False

        assert spin_until(node, localizer_active, 15), 'localizer did not activate'
        assert spin_until(node, lambda: buffer.can_transform('base_link', 'livox_frame', Time()), 8)
        transform = buffer.lookup_transform('base_link', 'livox_frame', Time())
        translation = transform.transform.translation
        assert np.allclose([translation.x, translation.y, translation.z], [0.252, 0.0, 0.236])
        for topic, message_type in [('/livox/lidar', 'sensor_msgs/msg/PointCloud2'),
                                     ('/livox/imu', 'sensor_msgs/msg/Imu')]:
            info = node.get_subscriptions_info_by_topic(topic)
            assert any(i.node_name == 'lidar_localization' and i.topic_type == message_type for i in info)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = node.get_clock().now().to_msg()
        initial.pose.pose.position.x, initial.pose.pose.position.y = map_base[:2]
        initial.pose.pose.orientation.z = math.sin(map_yaw / 2)
        initial.pose.pose.orientation.w = math.cos(map_yaw / 2)
        initial_pub.publish(initial)
        assert spin_until(node, lambda: len(healthy) >= 2, 25), 'no accepted NDT matches'
        assert buffer.can_transform('map', 'odom', Time())
        transform = buffer.lookup_transform('map', 'odom', Time())
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        expected_yaw = map_yaw - wheel_yaw
        r = np.array([[math.cos(expected_yaw), -math.sin(expected_yaw)],
                      [math.sin(expected_yaw), math.cos(expected_yaw)]])
        expected_translation = map_base[:2] - r @ wheel_base[:2]
        actual_translation = np.array([transform.transform.translation.x,
                                       transform.transform.translation.y])
        assert np.linalg.norm(expected_translation - actual_translation) < 0.3
        assert abs(yaw - expected_yaw) < 0.15
        clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in SERVERS}
        pending, active = {}, set()

        def all_active():
            for name, client in clients.items():
                if name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
                elif name in pending and pending[name].done():
                    if pending[name].result().current_state.label == 'active':
                        active.add(name)
                    else:
                        pending.pop(name)
            return len(active) == len(SERVERS)

        assert spin_until(node, all_active, 25), f'inactive Nav2 servers: {set(SERVERS) - active}'
        # Exercise the quickstart check after its 3 s timer and scoped includes.
        # Reaching active once can miss a delayed launch-wide shutdown.
        stable_until = time.monotonic() + 4.0
        while time.monotonic() < stable_until:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert process.poll() is None, f'bringup exited after activation; see {root}'
        report = {'test': 'PCD-derived synthetic scans, no physical accuracy claim',
                  'accepted_matches': len(healthy), 'sensor_translation': [translation.x, translation.y, translation.z],
                  'map_odom_xy': actual_translation.tolist(), 'expected_xy': expected_translation.tolist(),
                  'map_odom_yaw': yaw, 'expected_yaw': expected_yaw,
                  'active_nav2_servers': sorted(active)}
        (root / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        stop_process(process)
        log.close()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
