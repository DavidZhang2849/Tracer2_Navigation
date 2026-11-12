#!/usr/bin/env python3
"""Nav2 action-to-velocity acceptance test with an isolated synthetic robot.

Always uses a private localhost ROS domain and starts nav2.launch.py only.
No Tracer CAN node or Livox driver is started.
"""

import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


SERVERS = ['map_server', 'controller_server', 'smoother_server', 'planner_server',
           'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']


def spin_until(node, predicate, seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    # Deliberately overrides inherited production ROS settings.
    os.environ['ROS_DOMAIN_ID'] = '173'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp':
        os.environ['CYCLONEDDS_URI'] = (
            Path(get_package_share_directory('tracer_navigation')) /
            'config/cyclonedds.xml').as_uri()
    root = Path(tempfile.mkdtemp(prefix='tracer_nav2_smoke_'))
    os.environ['ROS_LOG_DIR'] = str(root / 'logs')
    pixels = bytearray([254] * 400 * 400)
    for i in range(400):
        for x, y in ((i, 0), (i, 399), (0, i), (399, i)):
            pixels[y * 400 + x] = 0
    (root / 'map.pgm').write_bytes(b'P5\n400 400\n255\n' + pixels)
    (root / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [-10.0, -10.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'Synthetic navigation test artifacts: {root}', flush=True)
    log = (root / 'navigation.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'tracer_navigation', 'nav2.launch.py',
        f'map_yaml:={root / "map.yaml"}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('tracer_mock_base')
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child, translation in [('map', 'odom', (0.0, 0.0, 0.0)),
                                        ('base_link', 'livox_frame', (0.252, 0.0, 0.236))]:
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.transform.translation.x, transform.transform.translation.y, \
            transform.transform.translation.z = translation
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    static.sendTransform(transforms)
    state = dict(x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0, commands=0,
                 last_command=0.0, last_tick=time.monotonic())

    def on_velocity(msg):
        state.update(v=msg.linear.x, w=msg.angular.z, last_command=time.monotonic())
        state['commands'] += abs(msg.linear.x) + abs(msg.angular.z) > 0.001

    node.create_subscription(Twist, '/cmd_vel', on_velocity, 10)

    def tick():
        now = time.monotonic()
        dt, state['last_tick'] = min(now - state['last_tick'], 0.1), now
        if now - state['last_command'] > 0.5:
            state.update(v=0.0, w=0.0)
        state['x'] += state['v'] * math.cos(state['yaw']) * dt
        state['y'] += state['v'] * math.sin(state['yaw']) * dt
        state['yaw'] += state['w'] * dt
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state['x'], state['y']
        odom.pose.pose.orientation.z = math.sin(state['yaw'] / 2)
        odom.pose.pose.orientation.w = math.cos(state['yaw'] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = state['v'], state['w']
        odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y = state['x'], state['y']
        tf.transform.rotation = odom.pose.pose.orientation
        broadcaster.sendTransform(tf)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, 'livox_frame'
        cloud.height, cloud.width = 1, 36
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.point_step, cloud.row_step, cloud.is_dense = 16, 16 * 36, True
        cloud.data = b''.join(struct.pack('<ffff', 8 * math.cos(i * math.pi / 18),
                                         8 * math.sin(i * math.pi / 18), 0.3, 20.0)
                              for i in range(36))
        cloud_pub.publish(cloud)
        imu = Imu()
        imu.header = cloud.header
        imu.linear_acceleration.z = 9.81
        imu.orientation_covariance[0] = -1.0
        imu_pub.publish(imu)

    timer = node.create_timer(0.05, tick)
    try:
        clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in SERVERS}
        pending, active = {}, set()

        def all_active():
            if process.poll() is not None:
                raise RuntimeError(f'Nav2 launch exited with {process.returncode}; see {root}')
            for name, client in clients.items():
                if name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
                elif name in pending and pending[name].done():
                    if pending[name].result().current_state.label == 'active':
                        active.add(name)
                    else:
                        pending.pop(name)
            return len(active) == len(SERVERS)

        assert spin_until(node, all_active, 45), f'Inactive servers: {set(SERVERS) - active}'
        print('All eight lifecycle servers active', flush=True)
        costmap_nodes = {i.node_name for i in
                         node.get_subscriptions_info_by_topic('/navigation/obstacle_cloud')}
        assert {'local_costmap', 'global_costmap'} <= costmap_nodes, costmap_nodes
        planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        assert planner.wait_for_server(timeout_sec=5)
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = 1.5
        goal.goal.pose.orientation.w = 1.0
        future = planner.send_goal_async(goal)
        assert spin_until(node, future.done, 10)
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        assert spin_until(node, result.done, 15)
        assert result.result().status == 4
        path_size = len(result.result().result.path.poses)
        assert path_size > 2
        print(f'ComputePathToPose succeeded: {path_size} path poses', flush=True)
        navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        assert navigator.wait_for_server(timeout_sec=5)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 1.5
        goal.pose.pose.orientation.w = 1.0
        future = navigator.send_goal_async(goal)
        assert spin_until(node, future.done, 10)
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        assert spin_until(node, result.done, 55), f'Navigation timed out: {state}'
        assert result.result().status == 4, f'Goal failed: {state}'
        assert state['commands'] > 0 and state['x'] > 1.0
        print('NavigateToPose succeeded; smoothed /cmd_vel reached the simulated base', flush=True)
        report = {'test': 'synthetic navigation, no CAN hardware', 'active_servers': sorted(active),
                  'path_poses': path_size, 'nonzero_commands': state['commands'],
                  'final_position': [state['x'], state['y']]}
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
