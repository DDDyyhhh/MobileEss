"""Identify and stop only an obstacle-cloud process owned by a field trial."""
import os
from pathlib import Path
import signal


def find_owned_obstacle_process(root_pid, proc_root=Path('/proc')):
    def argv(pid):
        return (proc_root / str(pid) / 'cmdline').read_bytes().decode().split('\0')

    root_argv = argv(root_pid)
    if (root_pid <= 1 or Path(root_argv[0]).name != 'timeout'
            or 'field_workflow' not in root_argv):
        raise RuntimeError('Expected the bounded field_workflow root process')
    matches = []
    executable = '/home/cat/MobileEss_ws/install/mower_nav2/lib/mower_nav2/livox_obstacle_cloud'
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            if executable not in argv(pid)[:2]:
                continue
            ancestor = pid
            for _ in range(32):
                if ancestor == root_pid:
                    matches.append(pid)
                    break
                status = (proc_root / str(ancestor) / 'status').read_text()
                ancestor = int(next(line.split()[1] for line in status.splitlines()
                                    if line.startswith('PPid:')))
                if ancestor <= 1:
                    break
        except (OSError, ValueError, StopIteration):
            continue
    if len(matches) != 1:
        raise RuntimeError('Require exactly one owned obstacle-cloud process')
    return matches[0]


def open_owned_obstacle_process(root_pid):
    pid = find_owned_obstacle_process(root_pid)
    fd = os.pidfd_open(pid)
    try:
        if find_owned_obstacle_process(root_pid) != pid:
            raise RuntimeError('Obstacle-cloud process identity changed')
    except BaseException:
        os.close(fd)
        raise
    return pid, fd


def terminate_obstacle_cloud(fd):
    # A pidfd cannot accidentally signal a replacement process with a reused PID.
    signal.pidfd_send_signal(fd, signal.SIGTERM)
