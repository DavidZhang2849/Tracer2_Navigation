from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

from tracer_navigation.configuration import load_hardware, localization_parameters
from tracer_navigation.configuration import nav2_parameters, read_yaml, validate_map_yaml
from tracer_navigation.configuration import obstacle_filter_parameters


ROOT = Path(__file__).resolve().parents[1]


def test_manifest_is_recognized_as_an_ament_python_package():
    from catkin_pkg.package import parse_package

    package = parse_package(ROOT / 'package.xml')
    assert package.name == 'tracer_navigation'
    assert package.get_build_type() == 'ament_python'


def test_localization_uses_wheel_tf_and_never_assumes_the_origin():
    hardware = load_hardware(ROOT / 'config/hardware.yaml')
    template = read_yaml(ROOT / 'config/localization.yaml')
    unchanged = deepcopy(template)
    result = localization_parameters(template, hardware, hardware['maps']['pcd'])
    params = result['/**']['ros__parameters']
    assert params['enable_map_odom_tf'] is True
    assert params['use_odom_tf_prediction'] is True
    assert params['use_twist_prediction'] is False
    assert params['use_odom'] is False
    assert params['set_initial_pose'] is False
    assert params['initial_pose_qw'] == 1.0
    assert template == unchanged


def test_topic_and_frame_overrides_reach_every_navigation_consumer():
    hardware = load_hardware(ROOT / 'config/hardware.yaml')
    hardware['topics'].update(cloud='/test/cloud', obstacle_cloud='/test/obstacles',
                              odom='/test/odom')
    hardware['frames'].update(map='site', odom='wheel', base='chassis')
    hardware['robot']['footprint'] = [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]]
    nav = nav2_parameters(read_yaml(ROOT / 'config/nav2_humble.yaml'), hardware)
    for name, frame in [('local_costmap', 'wheel'), ('global_costmap', 'site')]:
        params = nav[name][name]['ros__parameters']
        assert params['global_frame'] == frame
        assert params['robot_base_frame'] == 'chassis'
        assert 'footprint' in params and 'robot_radius' not in params
        layer = 'voxel_layer' if name == 'local_costmap' else 'obstacle_layer'
        assert params[layer]['pointcloud']['topic'] == '/test/obstacles'
        assert params[layer]['pointcloud']['min_obstacle_height'] < 0
    filtered = obstacle_filter_parameters(hardware)
    assert filtered['input_topic'] == '/test/cloud'
    assert filtered['output_topic'] == '/test/obstacles'
    assert filtered['base_frame'] == 'chassis'
    assert filtered['min_height'] == 0.08
    assert filtered['max_height'] == 1.50
    for name in ('controller_server', 'velocity_smoother', 'bt_navigator'):
        assert nav[name]['ros__parameters']['odom_topic'] == '/test/odom'


def test_plugin_ids_match_installed_humble_descriptions():
    nav = read_yaml(ROOT / 'config/nav2_humble.yaml')
    for package, filename, plugin_ids in [
        ('nav2_navfn_planner', 'global_planner_plugin.xml', [
            nav['planner_server']['ros__parameters']['GridBased']['plugin']]),
        ('nav2_behaviors', 'behavior_plugin.xml', [
            nav['behavior_server']['ros__parameters'][key]['plugin']
            for key in nav['behavior_server']['ros__parameters']['behavior_plugins']]),
    ]:
        xml = ET.parse(Path(get_package_share_directory(package)) / filename)
        registered = {element.get('name', element.get('type'))
                      for element in xml.iter('class')}
        assert set(plugin_ids) <= registered
    assert nav['bt_navigator']['ros__parameters']['plugin_lib_names']


def test_missing_map_image_is_reported_before_starting_nodes(tmp_path):
    metadata = tmp_path / 'map.yaml'
    metadata.write_text('image: missing.pgm\nresolution: 0.1\n')
    with pytest.raises(ValueError, match='map image does not exist'):
        validate_map_yaml(metadata)


def test_driver_and_mapping_configuration_agree_on_custom_format():
    from launch import LaunchContext
    from tracer_navigation.launches import actions

    context = LaunchContext()
    context.launch_configurations.update(
        hardware_config=str(ROOT / 'config/hardware.yaml'), use_sim_time='false',
        livox_config=str(ROOT / 'config/livox_mid360.json'), xfer_format='1')
    nodes = actions(context, 'mid360')
    assert len(nodes) == 1
    fastlio = read_yaml(ROOT / 'config/fastlio_mid360.yaml')['/**']['ros__parameters']
    assert fastlio['preprocess']['lidar_type'] == 1
    context.launch_configurations['xfer_format'] = '2'
    with pytest.raises(ValueError, match='PointCloud2'):
        actions(context, 'mid360')


def test_nav2_only_does_not_create_another_localizer_or_base():
    from launch import LaunchContext
    from launch_ros.actions import Node
    from tracer_navigation.launches import actions

    context = LaunchContext()
    context.launch_configurations.update(
        hardware_config=str(ROOT / 'config/hardware.yaml'), use_sim_time='false',
        map_yaml='', autostart='false', nav2_config=str(ROOT / 'config/nav2_humble.yaml'))
    nodes = [action for action in actions(context, 'nav2') if isinstance(action, Node)]
    packages = {node.node_package for node in nodes}
    assert 'nav2_map_server' in packages
    assert 'nav2_controller' in packages
    assert not {'tracer_base', 'livox_ros_driver2', 'lidar_localization_ros2'} & packages
