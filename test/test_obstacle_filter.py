import math
from pathlib import Path
import struct

from geometry_msgs.msg import Transform
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from tracer_navigation.configuration import load_hardware, obstacle_filter_parameters
from tracer_navigation.obstacle_filter import filter_cloud

def cloud_of(points, *, bigendian=False, height=1, row_padding=0):
    width = len(points) // height
    cloud = PointCloud2(height=height, width=width, point_step=16,
                        row_step=width * 16 + row_padding, is_bigendian=bigendian)
    cloud.header.frame_id = 'livox_frame'
    cloud.header.stamp.sec = 123
    cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.fields.append(PointField(name='tag', offset=12, datatype=PointField.UINT32, count=1))
    order = '>' if bigendian else '<'
    records = [struct.pack(order + 'fffI', *point, i) for i, point in enumerate(points)]
    cloud.data = b''.join(b''.join(records[i:i + width]) + b'\xff' * row_padding
                          for i in range(0, len(records), width))
    return cloud


def tags(cloud):
    order = '>' if cloud.is_bigendian else '<'
    return [struct.unpack_from(order + 'I', cloud.data, offset + 12)[0]
            for offset in range(0, len(cloud.data), cloud.point_step)]


def test_height_is_relative_to_base_and_point_records_remain_in_sensor_frame():
    cloud = cloud_of([(1.0, 0.0, z) for z in (-0.5, -0.25, 0.0, 0.5, 1.0, 1.25, 1.26)])
    original = bytes(cloud.data)
    transform = Transform()
    transform.rotation.w = 1.0
    transform.translation.z = 0.25
    filtered = filter_cloud(cloud, transform, 0.08, 1.5)
    assert tags(filtered) == [2, 3, 4, 5]
    assert filtered.header == cloud.header
    assert filtered.fields == cloud.fields
    assert bytes(filtered.data) == original[2 * 16:6 * 16]
    assert bytes(cloud.data) == original


def test_tilted_lidar_uses_transformed_height_instead_of_sensor_z():
    cloud = cloud_of([(x, 0.0, 3.0) for x in (-2.0, -1.0, -0.25, 0.0, 0.25)])
    transform = Transform()
    transform.rotation.y = math.sqrt(0.5)
    transform.rotation.w = math.sqrt(0.5)
    transform.translation.z = 0.25
    filtered = filter_cloud(cloud, transform, 0.08, 1.5)
    assert tags(filtered) == [1, 2, 3]


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_cloud_padding_endianness_and_invalid_points(bigendian):
    cloud = cloud_of([(1.0, 0.0, 0.0), (1.0, 0.0, 0.08), (1.0, 0.0, 1.5),
                      (1.0, 0.0, 1.51), (float('nan'), 0.0, 0.5), (1.0, 0.0, float('inf'))],
                     bigendian=bigendian, height=2, row_padding=8)
    original = bytes(cloud.data)
    transform = Transform()
    transform.rotation.w = 1.0
    filtered = filter_cloud(cloud, transform, 0.08, 1.5)
    assert tags(filtered) == [1, 2]
    assert filtered.height == 1 and filtered.width == 2 and filtered.row_step == 32
    assert filtered.is_bigendian == bigendian
    assert filtered.is_dense
    assert bytes(cloud.data) == original


def test_empty_or_entirely_removed_cloud_still_has_a_valid_header():
    transform = Transform()
    transform.rotation.w = 1.0
    for cloud in [PointCloud2(), cloud_of([(1.0, 0.0, 0.0), (1.0, 0.0, 2.0)])]:
        filtered = filter_cloud(cloud, transform, 0.08, 1.5)
        assert filtered.width == 0 and filtered.row_step == 0 and not filtered.data
        assert filtered.header == cloud.header


@pytest.mark.parametrize('settings', [
    {'min_height': 1.5, 'max_height': 0.08},
    {'max_height': float('nan')},
    {'transform_timeout': -1.0},
])
def test_invalid_filter_configuration_is_rejected(settings):
    hardware = load_hardware(Path(__file__).resolve().parents[1] / 'config/hardware.yaml')
    hardware['obstacle_filter'].update(settings)
    with pytest.raises(ValueError, match='height limits'):
        obstacle_filter_parameters(hardware)
