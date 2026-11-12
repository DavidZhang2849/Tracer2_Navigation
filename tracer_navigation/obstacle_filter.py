"""Filter navigation observations by height in the robot base frame."""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def filter_cloud(cloud, transform, min_height, max_height):
    """Keep original point records whose transformed base-frame Z is in range.

    Retaining the sensor frame, timestamp and complete records preserves the
    actual ray origin and fields such as intensity, time and Livox line/tag.
    """
    output = PointCloud2(
        header=cloud.header, height=1, width=0, fields=cloud.fields,
        is_bigendian=cloud.is_bigendian, point_step=cloud.point_step,
        row_step=0, is_dense=True,
    )
    if cloud.width == 0 or cloud.height == 0:
        return output
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    fields = {field.name: field for field in cloud.fields}
    xyz = [fields[name] for name in ('x', 'y', 'z')]
    if any(field.count != 1 or field.datatype not in formats for field in xyz):
        raise ValueError('PointCloud2 XYZ fields must be scalar float32 or float64')
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('PointCloud2 row_step is smaller than its point records')
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [formats[field.datatype] for field in xyz],
        'offsets': [field.offset for field in xyz],
        'itemsize': cloud.point_step,
    }).newbyteorder('>' if cloud.is_bigendian else '<')
    points = np.ndarray(
        (cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
        strides=(cloud.row_step, cloud.point_step),
    )
    rotation = transform.rotation
    qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    # Only the third row of the rotation matrix is needed to calculate height.
    height = (2.0 * (qx * qz - qw * qy) * points['x'] +
              2.0 * (qy * qz + qw * qx) * points['y'] +
              (1.0 - 2.0 * (qx * qx + qy * qy)) * points['z'] +
              transform.translation.z)
    keep = (np.isfinite(points['x']) & np.isfinite(points['y']) &
            np.isfinite(points['z']) & (height >= min_height) & (height <= max_height))
    records = np.ndarray(
        (cloud.height, cloud.width, cloud.point_step), dtype=np.uint8,
        buffer=cloud.data, strides=(cloud.row_step, cloud.point_step, 1),
    )
    output.width = int(np.count_nonzero(keep))
    output.row_step = output.width * cloud.point_step
    output.data = records[keep].tobytes()
    return output


class ObstacleHeightFilter(Node):
    def __init__(self):
        super().__init__('obstacle_height_filter')
        defaults = {
            'input_topic': '/livox/lidar',
            'output_topic': '/navigation/obstacle_cloud',
            'base_frame': 'base_link',
            'min_height': 0.08,
            'max_height': 1.50,
            'transform_timeout': 0.1,
        }
        self.declare_parameters('', list(defaults.items()))
        params = {name: self.get_parameter(name).value for name in defaults}
        self.base_frame = params['base_frame']
        self.min_height, self.max_height = params['min_height'], params['max_height']
        timeout = params['transform_timeout']
        if (not all(math.isfinite(value) for value in
                    (self.min_height, self.max_height, timeout)) or
                self.min_height >= self.max_height or timeout < 0):
            raise ValueError('height limits must be finite and increasing; timeout must be >= 0')
        if params['input_topic'] == params['output_topic']:
            raise ValueError('obstacle filter input and output topics must differ')
        self.timeout = Duration(seconds=timeout)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, params['output_topic'], qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, params['input_topic'], self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'{params["input_topic"]} -> {params["output_topic"]}: '
            f'{self.min_height:.3f} <= Z({self.base_frame}) <= {self.max_height:.3f} m')

    def on_cloud(self, cloud):
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp), timeout=self.timeout)
            output = filter_cloud(cloud, transform.transform, self.min_height, self.max_height)
        except (TransformException, KeyError, ValueError, TypeError) as error:
            self.get_logger().warning(
                f'Obstacle cloud skipped: {error}', throttle_duration_sec=5.0)
            return
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleHeightFilter()
    # The TF listener must progress while a cloud callback waits for its stamp.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
