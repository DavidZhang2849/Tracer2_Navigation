"""Configuration transformations shared by launch files and contract tests."""

from copy import deepcopy
from pathlib import Path
import json
import math
import tempfile

import yaml


def read_yaml(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def load_hardware(path):
    hardware = read_yaml(path)
    for key in ('frames', 'topics', 'base', 'sensor_transform', 'robot', 'maps'):
        if not isinstance(hardware.get(key), dict):
            raise ValueError(f'hardware configuration needs a {key} mapping')
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        if not math.isfinite(float(hardware['sensor_transform'][key])):
            raise ValueError(f'invalid sensor transform: {key}')
    if hardware['frames']['base'] == hardware['frames']['odom']:
        raise ValueError('base and odom must be different frames')
    if float(hardware['robot']['radius']) <= 0:
        raise ValueError('robot radius must be positive')
    return hardware


def validate_map_yaml(path):
    path = Path(path).expanduser().resolve(strict=True)
    metadata = read_yaml(path)
    image = Path(metadata['image'])
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError(f'map image does not exist: {image}')
    if float(metadata['resolution']) <= 0:
        raise ValueError('map resolution must be positive')
    return str(path)


def localization_parameters(template, hardware, map_path, use_sim_time=False):
    result = deepcopy(template)
    params = result['/**']['ros__parameters']
    frames = hardware['frames']
    params.update(
        map_path=str(Path(map_path).expanduser().resolve(strict=True)),
        global_frame_id=frames['map'], odom_frame_id=frames['odom'],
        base_frame_id=frames['base'], use_sim_time=use_sim_time,
        enable_map_odom_tf=True, use_odom_tf_prediction=True,
        # Wheel TF is the sole motion predictor; do not also integrate /twist.
        use_odom=False, use_twist_prediction=False,
        set_initial_pose=False, initial_pose_qw=1.0,
    )
    return result


def obstacle_filter_parameters(hardware, use_sim_time=False):
    settings = hardware.get('obstacle_filter', {})
    params = {
        'input_topic': hardware['topics']['cloud'],
        'output_topic': hardware['topics'].get('obstacle_cloud', '/navigation/obstacle_cloud'),
        'base_frame': hardware['frames']['base'],
        'min_height': float(settings.get('min_height', 0.08)),
        'max_height': float(settings.get('max_height', 1.50)),
        'transform_timeout': float(settings.get('transform_timeout', 0.1)),
        'use_sim_time': use_sim_time,
    }
    if (not all(math.isfinite(params[key]) for key in
                ('min_height', 'max_height', 'transform_timeout')) or
            params['min_height'] >= params['max_height'] or params['transform_timeout'] < 0):
        raise ValueError('invalid navigation obstacle height limits or transform timeout')
    if params['input_topic'] == params['output_topic']:
        raise ValueError('obstacle filter input and output topics must differ')
    return params


def nav2_parameters(template, hardware, use_sim_time=False):
    result = deepcopy(template)
    frames, topics = hardware['frames'], hardware['topics']
    obstacle_filter = obstacle_filter_parameters(hardware, use_sim_time)
    for node in result.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            node['ros__parameters']['use_sim_time'] = use_sim_time
    for key, global_frame in [('local_costmap', frames['odom']),
                              ('global_costmap', frames['map'])]:
        params = result[key][key]['ros__parameters']
        params.update(global_frame=global_frame, robot_base_frame=frames['base'],
                      use_sim_time=use_sim_time)
        robot = hardware['robot']
        if robot.get('footprint'):
            params['footprint'] = json.dumps(robot['footprint'])
            params.pop('robot_radius', None)
        else:
            params['robot_radius'] = float(robot['radius'])
            params.pop('footprint', None)
        for layer in ('voxel_layer', 'obstacle_layer'):
            if layer in params:
                params[layer]['pointcloud']['topic'] = obstacle_filter['output_topic']
    navigator = result['bt_navigator']['ros__parameters']
    navigator.update(global_frame=frames['map'], robot_base_frame=frames['base'],
                     odom_topic=topics['odom'])
    result['controller_server']['ros__parameters']['odom_topic'] = topics['odom']
    result['behavior_server']['ros__parameters'].update(
        global_frame=frames['odom'], robot_base_frame=frames['base'])
    result['velocity_smoother']['ros__parameters']['odom_topic'] = topics['odom']
    return result


def temporary_yaml(data, prefix):
    with tempfile.NamedTemporaryFile(mode='w', prefix=prefix, suffix='.yaml',
                                     encoding='utf-8', delete=False) as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
        return stream.name
