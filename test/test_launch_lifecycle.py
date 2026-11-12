"""Exercise delayed launch actions without starting ROS or hardware processes."""

from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetLaunchConfiguration, TimerAction
import pytest

from tracer_navigation.configuration import load_hardware
from tracer_navigation.launches import description


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('component', ['localization', 'navigation'])
@pytest.mark.parametrize('run_check', [True, False])
def test_delayed_bringup_check_survives_scoped_includes(
        monkeypatch, tmp_path, component, run_check):
    commands, remaps = {}, {}
    original_timer_init = TimerAction.__init__

    def short_timer(self, *, period, **kwargs):
        original_timer_init(self, period=0.01, **kwargs)

    def record_process(self, context):
        # Node.execute still resolves its real parameters, name and remappings.
        # Stop just before creating a subprocess, so the test cannot reach CAN.
        self.process_description.prepare(context, self)
        name = '/' + self.node_name.rsplit('/', 1)[-1]
        commands[name] = self.process_description.final_cmd
        remaps[name] = self.expanded_remapping_rules or []

    monkeypatch.setattr(TimerAction, '__init__', short_timer)
    monkeypatch.setattr(ExecuteProcess, 'execute', record_process)
    hardware_path = ROOT / 'config/hardware.yaml'
    hardware = load_hardware(hardware_path)
    launch = LaunchDescription([
        SetLaunchConfiguration('hardware_config', str(hardware_path)),
        SetLaunchConfiguration('rviz', 'false'),
        SetLaunchConfiguration('start_base', 'false'),
        SetLaunchConfiguration('start_lidar', 'false'),
        SetLaunchConfiguration('autostart', 'false'),
        SetLaunchConfiguration('state_file', str(tmp_path / 'pose.json')),
    ])
    if not run_check:
        launch.add_action(SetLaunchConfiguration('run_bringup_check', 'false'))
    launch.add_entity(description(component))
    service = LaunchService(noninteractive=True)
    service.include_launch_description(launch)

    assert service.run() == 0
    assert '/lidar_localization' in commands
    assert ('/map', '/localization/map_cloud') in remaps['/lidar_localization']
    if component == 'navigation':
        assert '/map_server' in commands, sorted(commands)
        assert ('/map', '/localization/map_cloud') not in remaps['/map_server']
    check = commands.get('/quickstart_bringup_check')
    if run_check:
        assert check is not None
        for flag, expected in [
            ('--profile', 'nav2'),
            ('--cloud-topic', hardware['topics']['cloud']),
            ('--imu-topic', hardware['topics']['imu']),
            ('--lidar-frame', hardware['frames']['lidar']),
            ('--imu-frame', hardware['frames']['imu']),
            ('--global-frame', hardware['frames']['map']),
            ('--odom-frame', hardware['frames']['odom']),
            ('--base-frame', hardware['frames']['base']),
            ('--wait-for-localization-sec', '120.0'),
        ]:
            assert check[check.index(flag) + 1] == expected
        assert '--require-map-odom-tf' in check
        assert '--require-odom-base-tf' in check
    else:
        assert check is None
