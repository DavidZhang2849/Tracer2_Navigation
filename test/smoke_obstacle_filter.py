#!/usr/bin/env python3
"""Verify floor/ceiling rejection through real Nav2 nodes without any hardware."""

import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import StaticTransformBroadcaster

from smoke_navigation import stop_process


def main():
    os.environ['ROS_DOMAIN_ID'] = '185'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    root = Path(tempfile.mkdtemp(prefix='tracer_obstacle_filter_verify_'))
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    (root / 'map.pgm').write_bytes(b'P5\n100 100\n255\n' + bytes([254]) * 10000)
    (root / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'Obstacle filtering test artifacts: {root}', flush=True)
    rclpy.init()
    node = rclpy.create_node('obstacle_filter_verifier')
    static = StaticTransformBroadcaster(node)
    transforms = []
    # A nonzero map Z exposes incorrect filtering in map instead of base_link.
    for parent, child, z in [('map', 'odom', 2.0), ('odom', 'base_link', 0.0),
                             ('base_link', 'livox_frame', 0.25)]:
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    static.sendTransform(transforms)
    publisher = node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
    received = {}
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key, message_type, topic, qos in [
        ('filtered', PointCloud2, '/navigation/obstacle_cloud', qos_profile_sensor_data),
        ('voxels', PointCloud2, '/local_costmap/voxel_marked_cloud', 10),
        ('local', OccupancyGrid, '/local_costmap/costmap', map_qos),
        ('global', OccupancyGrid, '/global_costmap/costmap', map_qos),
    ]:
        node.create_subscription(message_type, topic,
                                 lambda msg, key=key: received.update({key: msg}), qos)
    # Sensor Z + 0.25 = base Z: ground, floor noise, obstacle, robot top, ceilings.
    points = [(-1.5, -1.5, -0.25), (0.0, -1.5, -0.21), (1.2, 0.0, 0.25),
              (1.4, 0.4, 1.25), (1.2, 1.4, 1.50), (1.8, 1.6, 2.25)]

    def publish_cloud():
        cloud = PointCloud2(height=1, width=len(points), point_step=16,
                            row_step=16 * len(points), is_dense=True)
        cloud.header.stamp = node.get_clock().now().to_msg()
        cloud.header.frame_id = 'livox_frame'
        cloud.fields = [PointField(name=name, offset=i * 4,
                                   datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.data = b''.join(struct.pack('<ffff', *point, float(i))
                              for i, point in enumerate(points))
        publisher.publish(cloud)

    def cell(grid, x, y):
        mx = int((x - grid.info.origin.position.x) / grid.info.resolution)
        my = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if 0 <= mx < grid.info.width and 0 <= my < grid.info.height:
            return grid.data[my * grid.info.width + mx]
        return None

    def ready():
        if not all(key in received for key in ('filtered', 'voxels', 'local', 'global')):
            return False
        if received['filtered'].width != 2 or received['voxels'].width != 2:
            return False
        for key in ('local', 'global'):
            # The map resolution and rolling-window origin can round cell edges
            # differently. Each expected hit must have a lethal cell within one cell.
            resolution = received[key].info.resolution
            for i in (2, 3):
                costs = [cell(received[key], points[i][0] + dx * resolution,
                              points[i][1] + dy * resolution)
                         for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
                if 100 not in costs:
                    return False
            if any(cell(received[key], *points[i][:2]) != 0 for i in (0, 1, 4, 5)):
                return False
        return True

    timer = node.create_timer(0.1, publish_cloud)
    log = (root / 'nav2.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'tracer_navigation', 'nav2.launch.py',
        f'map_yaml:={root / "map.yaml"}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline and not ready():
            assert process.poll() is None, f'Nav2 exited; see {root / "nav2.log"}'
            rclpy.spin_once(node, timeout_sec=0.1)
        assert ready(), f'Filtering/costmap mismatch; received {list(received)}; see {root}'
        subscribers = node.get_subscriptions_info_by_topic('/navigation/obstacle_cloud')
        assert {'local_costmap', 'global_costmap'} <= {item.node_name for item in subscribers}
        cloud = received['filtered']
        assert cloud.header.frame_id == 'livox_frame'
        tags = point_cloud2.read_points(cloud, field_names=('intensity',))
        assert {int(point['intensity']) for point in tags} == {2, 3}
        report = {'result': 'PASS', 'input_points': len(points), 'filtered_points': cloud.width,
                  'occupied_voxels': received['voxels'].width, 'map_z_offset_m': 2.0,
                  'floor_and_ceiling_free_in_both_costmaps': True}
        (root / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        stop_process(process)
        log.close()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
