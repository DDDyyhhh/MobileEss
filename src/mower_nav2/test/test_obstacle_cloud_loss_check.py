from pathlib import Path

import pytest

from mower_nav2.obstacle_cloud_loss_check import obstacle_cloud_loss_blocks
from mower_nav2.obstacle_cloud_fault import find_owned_obstacle_process


def sample():
    return dict(input_linear_x='.1', input_angular_z='0', output_linear_x='0',
                output_angular_z='0', command_age_sec='.05', costmap_age_sec='.05',
                odom_age_sec='.05', obstacle_age_sec='.35')


REASON = 'stale command, costmap, odometry, or obstacle cloud'


def test_only_cloud_loss_with_fresh_command_can_count():
    assert obstacle_cloud_loss_blocks(sample(), REASON)
    assert not obstacle_cloud_loss_blocks(sample(), 'predicted footprint cost 253 >= 253')


@pytest.mark.parametrize('key,value', [
    ('obstacle_age_sec', '.3'), ('obstacle_age_sec', 'nan'),
    ('command_age_sec', '.26'), ('command_age_sec', '.31'), ('odom_age_sec', '.31'),
    ('costmap_age_sec', '.31'), ('input_linear_x', '0'),
    ('output_linear_x', '.1'), ('output_angular_z', '.1'),
])
def test_other_loss_or_nonzero_output_does_not_pass(key, value):
    values = sample()
    values[key] = value
    assert not obstacle_cloud_loss_blocks(values, REASON)


def proc(root, pid, parent, args):
    directory = root / str(pid)
    directory.mkdir()
    (directory / 'cmdline').write_bytes('\0'.join(args).encode())
    (directory / 'status').write_text(f'Name: test\nPPid:\t{parent}\n')


EXE = '/home/cat/MobileEss_ws/install/mower_nav2/lib/mower_nav2/livox_obstacle_cloud'


def test_only_owned_descendant_selected(tmp_path):
    proc(tmp_path, 10, 1, ['timeout', 'ros2', 'run', 'mower_nav2', 'field_workflow'])
    proc(tmp_path, 11, 10, ['ros2', 'launch'])
    proc(tmp_path, 12, 11, ['python3', EXE])
    proc(tmp_path, 20, 1, ['python3', EXE])
    assert find_owned_obstacle_process(10, tmp_path) == 12


def test_ambiguous_owned_processes_rejected(tmp_path):
    proc(tmp_path, 10, 1, ['timeout', 'field_workflow'])
    proc(tmp_path, 11, 10, [EXE])
    proc(tmp_path, 12, 10, [EXE])
    with pytest.raises(RuntimeError):
        find_owned_obstacle_process(10, tmp_path)


def test_unrelated_root_or_shell_text_rejected(tmp_path):
    proc(tmp_path, 10, 1, ['bash', '-c', 'field_workflow'])
    proc(tmp_path, 11, 10, ['bash', '-c', f'echo {EXE}'])
    with pytest.raises(RuntimeError):
        find_owned_obstacle_process(10, tmp_path)
