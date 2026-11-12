"""Launch composition; vendor algorithm code is used without hardware patches."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from .configuration import load_hardware, localization_parameters, nav2_parameters
from .configuration import obstacle_filter_parameters
from .configuration import read_yaml, temporary_yaml, validate_map_yaml


def share():
    return Path(get_package_share_directory('tracer_navigation'))


def value(context, key):
    return LaunchConfiguration(key).perform(context)


def boolean(context, key):
    text = value(context, key).lower()
    if text not in ('true', 'false'):
        raise ValueError(f'{key} must be true or false')
    return text == 'true'


def include(name, **arguments):
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share() / 'launch' / f'{name}.launch.py')),
        launch_arguments={key: str(val).lower() if isinstance(val, bool) else str(val)
                          for key, val in arguments.items()}.items(),
    )])


def sensor_tf(hardware):
    frames = hardware['frames']
    if frames['base'] == frames['lidar']:
        return []
    arguments = []
    for key, val in hardware['sensor_transform'].items():
        arguments.extend([f'--{key}', str(val)])
    arguments.extend(['--frame-id', frames['base'], '--child-frame-id', frames['lidar']])
    if frames['imu'] != frames['lidar']:
        raise ValueError('MID360 driver IMU header is livox_frame; keep imu/lidar frames equal')
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name='tracer_lidar_tf', arguments=arguments, output='screen')]


def description(component):
    root = share()
    arguments = {
        'hardware_config': str(root / 'config' / 'hardware.yaml'),
        'use_sim_time': 'false',
    }
    by_component = {
        'mid360': {'livox_config': str(root / 'config' / 'livox_mid360.json'),
                   'xfer_format': '0'},
        'base': {'port_name': ''},
        'mapping': {'start_base': 'false', 'start_lidar': 'true', 'rviz': 'true',
                    'fastlio_config': str(root / 'config' / 'fastlio_mid360.yaml'),
                    'map_output': str(Path.home() / 'maps' / 'fastlio_mapping.pcd')},
        'localization': {'map': '', 'map_yaml': '', 'rviz': 'true',
                         'localization_config': str(root / 'config' / 'localization.yaml'),
                         'auto_initialize': 'true', 'publish_sensor_tf': 'true',
                         'state_file': ''},
        'nav2': {'map_yaml': '', 'autostart': 'true',
                 'nav2_config': str(root / 'config' / 'nav2_humble.yaml')},
        'navigation': {'map': '', 'map_yaml': '', 'start_base': 'true',
                       'start_lidar': 'true', 'start_localization': 'true',
                       'rviz': 'true', 'autostart': 'true',
                       'auto_initialize': 'true', 'publish_sensor_tf': 'true',
                       'localization_config': str(root / 'config' / 'localization.yaml'),
                       'nav2_config': str(root / 'config' / 'nav2_humble.yaml'),
                       'state_file': ''},
    }
    arguments.update(by_component[component])
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=val) for key, val in arguments.items()],
        OpaqueFunction(function=actions, args=[component]),
    ])


def actions(context, component):
    hardware_path = value(context, 'hardware_config')
    hardware = load_hardware(hardware_path)
    frames, topics = hardware['frames'], hardware['topics']
    use_sim_time = boolean(context, 'use_sim_time')
    common = {'hardware_config': hardware_path, 'use_sim_time': use_sim_time}
    root = share()
    if component == 'base':
        base = hardware['base']
        return [Node(package='tracer_base', executable='tracer_base_node', name='tracer',
                     parameters=[{
                         'use_sim_time': use_sim_time,
                         'port_name': value(context, 'port_name') or base['port_name'],
                         'is_tracer_mini': bool(base['is_tracer_mini']),
                         'simulated_robot': False, 'control_rate': 50,
                         'odom_frame': frames['odom'], 'base_frame': frames['base'],
                         'odom_topic_name': topics['odom'],
                     }], remappings=[('/cmd_vel', topics['cmd_vel'])], output='screen')]
    if component == 'mid360':
        xfer_format = int(value(context, 'xfer_format'))
        if xfer_format not in (0, 1):
            raise ValueError('xfer_format must be 0 (PointCloud2) or 1 (CustomMsg)')
        if frames['lidar'] != 'livox_frame' or frames['imu'] != 'livox_frame':
            raise ValueError('this driver hardcodes the IMU header frame to livox_frame')
        return [Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen',
            parameters=[{'xfer_format': xfer_format, 'multi_topic': 0, 'data_src': 0,
                         'publish_freq': 10.0, 'output_data_type': 0,
                         'frame_id': frames['lidar'], 'use_sim_time': use_sim_time,
                         'user_config_path': value(context, 'livox_config'),
                         'lvx_file_path': '/home/livox/livox_test.lvx',
                         'cmdline_input_bd_code': 'livox0000000001'}],
            remappings=[('/livox/lidar', topics['cloud']), ('/livox/imu', topics['imu'])],
        )]
    if component == 'mapping':
        result = []
        if boolean(context, 'start_base'):
            result.append(include('base', **common))
        if boolean(context, 'start_lidar'):
            result.append(include('mid360', **common, xfer_format=1))
        result += sensor_tf(hardware)
        result.append(Node(
            package='fast_lio', executable='fastlio_mapping', output='screen',
            parameters=[value(context, 'fastlio_config'), {
                'use_sim_time': use_sim_time, 'common.lid_topic': topics['cloud'],
                'common.imu_topic': topics['imu'], 'preprocess.lidar_type': 1,
                'map_file_path': str(Path(value(context, 'map_output')).expanduser()),
            }],
        ))
        if boolean(context, 'rviz'):
            fastlio_share = Path(get_package_share_directory('fast_lio'))
            result.append(Node(package='rviz2', executable='rviz2',
                               arguments=['-d', str(fastlio_share / 'rviz' / 'fastlio.rviz')],
                               parameters=[{'use_sim_time': use_sim_time}], output='screen'))
        return result
    map_yaml = value(context, 'map_yaml') or hardware['maps']['yaml']
    map_yaml = validate_map_yaml(map_yaml)
    if component == 'localization':
        map_path = value(context, 'map') or hardware['maps']['pcd']
        params = localization_parameters(read_yaml(value(context, 'localization_config')),
                                         hardware, map_path, use_sim_time)
        generated = temporary_yaml(params, 'tracer_localization_')
        vendor_share = Path(get_package_share_directory('lidar_localization_ros2'))
        state_file = value(context, 'state_file') or str(
            Path.home() / '.local' / 'state' / 'tracer_navigation' /
            f'{Path(map_path).stem}.json')
        auto = boolean(context, 'auto_initialize')
        launch_arguments = {
            'profile': 'nav2', 'localization_param_dir': generated,
            'map_path': str(Path(map_path).expanduser().resolve()),
            'occupancy_yaml': map_yaml, 'pose_state_path': state_file,
            'cloud_topic': topics['cloud'], 'imu_topic': topics['imu'],
            'pose_topic': '/localization/pose_with_covariance',
            'global_frame_id': frames['map'], 'odom_frame_id': frames['odom'],
            'base_frame_id': frames['base'], 'lidar_frame_id': frames['lidar'],
            'imu_frame_id': frames['imu'], 'use_sim_time': str(use_sim_time).lower(),
            'publish_lidar_tf': 'false', 'publish_imu_tf': 'false',
            'restore_saved_pose': str(auto).lower(),
            'enable_global_initialization': str(auto).lower(),
            'enable_g3_recovery': str(auto).lower(), 'start_rviz': 'false',
            'g2_registration_sensor_height_m': str(
                hardware.get('localization', {}).get('global_sensor_height_m', 1.0)),
        }
        result = [GroupAction(actions=[
            # The localizer's optional PointCloud2 map input must not share
            # /map with Nav2's OccupancyGrid map server.
            SetRemap(src='/map', dst='/localization/map_cloud'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(vendor_share / 'launch' / 'quickstart.launch.py')),
                launch_arguments=launch_arguments.items(),
            ),
        ])]
        if boolean(context, 'publish_sensor_tf'):
            result += sensor_tf(hardware)
        if boolean(context, 'rviz'):
            rviz = read_yaml(root / 'rviz' / 'navigation.rviz')
            manager = rviz['Visualization Manager']
            manager['Global Options']['Fixed Frame'] = frames['map']
            for display in manager['Displays']:
                if display.get('Name') == 'MID360':
                    display['Topic']['Value'] = topics['cloud']
            result.append(Node(package='rviz2', executable='rviz2', output='screen',
                               arguments=['-d', temporary_yaml(rviz, 'tracer_rviz_')],
                               parameters=[{'use_sim_time': use_sim_time}]))
        return result
    if component == 'nav2':
        params = nav2_parameters(read_yaml(value(context, 'nav2_config')),
                                 hardware, use_sim_time)
        params['map_server']['ros__parameters']['yaml_filename'] = map_yaml
        generated = temporary_yaml(params, 'tracer_nav2_')
        # These are the installed Humble lifecycle servers. No AMCL or odom demo.
        servers = [
            ('nav2_map_server', 'map_server'),
            ('nav2_controller', 'controller_server'),
            ('nav2_smoother', 'smoother_server'),
            ('nav2_planner', 'planner_server'),
            ('nav2_behaviors', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
            ('nav2_waypoint_follower', 'waypoint_follower'),
            ('nav2_velocity_smoother', 'velocity_smoother'),
        ]
        filter_params = obstacle_filter_parameters(hardware, use_sim_time)
        result = [
            LogInfo(msg=f'Tracer Nav2 map={map_yaml}, cloud={filter_params["output_topic"]}, '
                        f'odom={topics["odom"]}, cmd_vel={topics["cmd_vel"]}'),
            Node(package='tracer_navigation', executable='obstacle_height_filter',
                 name='obstacle_height_filter', parameters=[filter_params], output='screen'),
        ]
        for package, executable in servers:
            remappings = []
            if executable in ('controller_server', 'behavior_server'):
                remappings = [('cmd_vel', '/cmd_vel_nav')]
            elif executable == 'velocity_smoother':
                remappings = [('cmd_vel', '/cmd_vel_nav'),
                              ('cmd_vel_smoothed', topics['cmd_vel'])]
            result.append(Node(package=package, executable=executable, name=executable,
                               parameters=[generated], remappings=remappings, output='screen'))
        local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
        voxel = local_costmap.get('voxel_layer', {})
        if ('voxel_layer' in local_costmap.get('plugins', []) and
                voxel.get('enabled', True) and voxel.get('publish_voxel_map', False)):
            # RViz displays PointCloud2; this node decodes nav2_msgs/VoxelGrid.
            result.append(Node(
                package='nav2_costmap_2d', executable='nav2_costmap_2d_cloud',
                namespace='local_costmap', name='voxel_visualizer', output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ))
        result.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': boolean(context, 'autostart'),
                         'node_names': [name for _, name in servers]}],
        ))
        return result
    if component == 'navigation':
        # Check maps before starting any hardware process.
        map_path = value(context, 'map') or hardware['maps']['pcd']
        Path(map_path).expanduser().resolve(strict=True)
        result = []
        if boolean(context, 'start_base'):
            result.append(include('base', **common))
        if boolean(context, 'start_lidar'):
            result.append(include('mid360', **common, xfer_format=0))
        if boolean(context, 'start_localization'):
            result.append(include(
                'localization', **common, map=map_path, map_yaml=map_yaml,
                rviz=boolean(context, 'rviz'),
                localization_config=value(context, 'localization_config'),
                auto_initialize=boolean(context, 'auto_initialize'),
                publish_sensor_tf=boolean(context, 'publish_sensor_tf'),
                state_file=value(context, 'state_file'),
            ))
        result.append(include('nav2', **common, map_yaml=map_yaml,
                              nav2_config=value(context, 'nav2_config'),
                              autostart=boolean(context, 'autostart')))
        return result
    raise ValueError(f'unknown component: {component}')
