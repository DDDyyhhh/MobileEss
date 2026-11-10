"""Board-only field stages. No automatic goals, map approval, or CAN unlock."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import platform
import signal
import socket
import struct
import subprocess
import time


def require_board():
    if platform.machine() not in ('aarch64', 'arm64'):
        raise SystemExit('ROS field commands run on RK3588 only')


def stop_processes(processes):
    """Stop owned launch groups even when the launch leader already exited."""
    def send(child, sig):
        try:
            os.killpg(child.pid, sig)
            return True
        except ProcessLookupError:
            return False

    for child in reversed(processes):
        send(child, signal.SIGINT)
    for child in reversed(processes):
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
        # A reaped ros2/launch leader does not prove its nodes have exited.
        if not send(child, 0):
            continue
        send(child, signal.SIGTERM)
        deadline = time.monotonic() + 5
        while send(child, 0) and time.monotonic() < deadline:
            time.sleep(0.05)
        if send(child, 0):
            send(child, signal.SIGKILL)
        child.wait(timeout=5)


def listen_can(interface, seconds):
    import time
    counts = {'0x005': 0, '0x507': 0, '0x50a': 0}
    samples = {}
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.bind((interface,))
        bus.settimeout(0.2)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                frame = bus.recv(16)
            except socket.timeout:
                continue
            can_id, dlc, data = struct.unpack('=IB3x8s', frame)
            if can_id & 0xE0000000 or dlc != 8:
                continue
            key = hex(can_id)
            if key in counts:
                counts[key] += 1
                samples[key] = data.hex()
        print(json.dumps({'seconds': seconds, 'counts': counts,
                          'last_payloads': samples}, indent=2))
        return 0 if counts['0x507'] and counts['0x50a'] and not counts['0x005'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['sensors', 'capture', 'save', 'convert',
                                         'observe', 'motion', 'can-listen'])
    parser.add_argument('--session', type=Path)
    parser.add_argument('--livox-config')
    parser.add_argument('--fast-lio-config')
    parser.add_argument('--nav2-params', type=Path)
    parser.add_argument('--cmd-vel-params', type=Path)
    parser.add_argument('--localizer-params', type=Path)
    parser.add_argument('--can-interface', default='can0')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--duration', type=float, default=0,
                        help='Automatically stop sensors after this many seconds (0: interactive)')
    parser.add_argument('--physical-ready', action='store_true',
                        help='Blade power isolated; stop controls tested; test area secured')
    parser.add_argument('--safety-map-approved', action='store_true',
                        help='Boundary, buffers, keep-out and speed zones reviewed')
    args = parser.parse_args()
    require_board()
    if args.duration < 0 or (args.duration and args.stage != 'sensors'):
        parser.error('--duration is nonnegative and only supported for sensors')
    if args.stage == 'can-listen':
        raise SystemExit(listen_can(args.can_interface, args.seconds))
    if args.stage == 'motion' and not (args.physical_ready and args.safety_map_approved):
        parser.error('motion requires --physical-ready and --safety-map-approved')
    if args.stage in ('sensors', 'capture', 'observe', 'motion'):
        active = subprocess.run(['ros2', 'node', 'list', '--no-daemon'],
                                check=True, capture_output=True, text=True, timeout=15)
        forbidden = {'/livox_lidar_publisher', '/laser_mapping', '/fast_lio_map_capture',
                     '/fixed_map_localizer', '/cmd_vel_can_bridge', '/controller_server'}
        overlap = forbidden.intersection(active.stdout.splitlines())
        if overlap:
            parser.error('stop existing field nodes first: ' + ', '.join(sorted(overlap)))
    if args.stage not in ('sensors',) and args.session is None:
        parser.error('--session is required')
    session = args.session.resolve() if args.session else None
    saved_config = session if session and args.stage in ('observe', 'motion') else Path('/home/cat/MobileEss_config')
    args.livox_config = args.livox_config or str(saved_config / 'livox.json')
    args.fast_lio_config = args.fast_lio_config or str(saved_config / 'mid360_real.yaml')
    if args.stage == 'capture':
        session.mkdir(parents=True, exist_ok=False)
        for source, name in ((args.livox_config, 'livox.json'),
                             (args.fast_lio_config, 'mid360_real.yaml')):
            (session / name).write_bytes(Path(source).read_bytes())
        (session / 'metadata.json').write_text(json.dumps({
            'created': datetime.now().astimezone().isoformat(), 'host': socket.gethostname(),
            'sensor_xyz_m': [0.17, 0.0, 0.48], 'sensor_rpy_rad': [0.0, 0.0, 0.0],
            'capture_frame': 'map_capture_odom', 'rtk_enabled': False,
            'map_reviewed': False, 'vehicle_tested': False,
        }, indent=2))
        (session / 'acceptance.md').write_text(
            '# Field acceptance\n\n| Check | Evidence / result |\n|---|---|\n'
            '| Mapping coverage and return-to-start overlap | Pending |\n'
            '| Boundary / keep-out / speed zones | Pending |\n'
            '| Localization accepted / TF single owners | Pending |\n'
            '| No control CAN in observation | Pending |\n'
            '| Wheels raised: directions / stop / 500 ms timeout | Pending |\n'
            '| Ground: goal / obstacle / blockage / sensor loss | Pending |\n')
    if args.stage == 'save':
        result = subprocess.run(['ros2', 'service', 'call', '/map_save', 'std_srvs/srv/Trigger', '{}'],
                                check=True, capture_output=True, text=True, timeout=60)
        print(result.stdout)
        if 'success=True' not in result.stdout and 'success: true' not in result.stdout:
            raise SystemExit('Map save did not report success; keep capture running')
        if not (session / 'map.pcd').is_file() or (session / 'map.pcd').stat().st_size == 0:
            raise SystemExit('map.pcd missing or empty; keep capture running')
        print('Map save succeeded and PCD exists. Capture may now be stopped.')
        return
    if args.stage == 'convert':
        from ament_index_python.packages import get_package_share_directory
        if (session / 'map.yaml').exists():
            raise SystemExit('Map already exists; use a separate review directory')
        script = Path(get_package_share_directory('mower_nav2')) / 'scripts/pcd_to_occupancy.py'
        subprocess.run(['python3', str(script), str(session / 'map.pcd'),
                        '--output-dir', str(session), '--resolution', '0.10',
                        '--z-min', '0.15', '--z-max', '0.80',
                        '--obstacle-height', '0.15', '--min-obstacle-points', '5'], check=True)
        return
    processes = []
    def stop(signum=None, frame=None):
        stop_processes(processes)
    signal.signal(signal.SIGTERM, stop)
    command = ['ros2', 'launch', 'mower_nav2', 'mid360_field.launch.py',
               f'stage:={args.stage}', f'livox_config:={args.livox_config}',
               f'fast_lio_config:={args.fast_lio_config}',
               f'can_interface:={args.can_interface}',
               f'physical_ready:={str(args.physical_ready).lower()}',
               f'safety_map_approved:={str(args.safety_map_approved).lower()}']
    for key, value in (('nav2_params', args.nav2_params), ('cmd_vel_params', args.cmd_vel_params), ('localizer_params', args.localizer_params)):
        if value is not None:
            if not value.is_file():
                parser.error(f'{key} must name a configuration file')
            command.append(f'{key}:={value.resolve()}')
    if session:
        command.extend([f'session_dir:={session}', f'map_dir:={session}'])
    try:
        started = time.monotonic()
        processes.append(subprocess.Popen(command, start_new_session=True))
        if args.stage == 'capture':
            processes.append(subprocess.Popen([
                'ros2', 'bag', 'record', '-o', str(session / 'bag'),
                '/livox/lidar', '/livox/imu', '/fast_lio/map_capture/odometry',
                '/cloud_registered', '/tf', '/tf_static'], start_new_session=True))
        while all(child.poll() is None for child in processes):
            if args.duration and time.monotonic() - started >= args.duration:
                return
            time.sleep(0.2)
        raise SystemExit(next(child.returncode for child in processes
                              if child.returncode is not None))
    except KeyboardInterrupt:
        pass
    finally:
        stop()


if __name__ == '__main__':
    main()
