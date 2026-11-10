"""Finite wheel-raised checks; never clears locks or starts navigation."""
import argparse
import json
from pathlib import Path
import platform
import signal
import socket
import struct
import time

from .protocol import parse_wheel_feedback

FRAME = struct.Struct('=IB3x8s')
WATCHDOG_API_EXIT_DEADLINE_S = 1.0
WATCHDOG_STOP_CONFIRM_DEADLINE_S = 1.5


def bench_motion_profile(action):
    if action == 'wheel-revolution':
        return 'forward', 5.0
    if action == 'watchdog':
        return 'forward', 2.0
    if action in ('forward', 'forward-02', 'forward-04', 'left'):
        return action, 2.0
    raise ValueError('Action has no bounded motion profile')


def bench_payload(mode, action, gear, bag):
    if mode not in (0, 1, 2) or action not in ('zero', 'forward', 'forward-02', 'forward-04', 'left'):
        raise ValueError('invalid bench mode/action')
    if gear not in range(1, 6) or bag not in (0, 1):
        raise ValueError('explicit valid auxiliary targets required')
    vx = 1 if mode == 2 and action == 'forward' else 0
    if mode == 2 and action == 'forward-02':
        vx = 2
    if mode == 2 and action == 'forward-04':
        vx = 4
    wz = 2 if mode == 2 and action == 'left' else 0
    return bytes((mode, vx, 0, wz, 0, gear, bag, 0))


def watchdog_evidence(events, last_tx_s):
    """Assess independent fresh telemetry after the last control frame.

    Zero must persist for three wheel frames, and API must remain exited.
    The deadline includes feedback and wheel deceleration; it is not the
    firmware's internal 500 ms timeout measurement.
    """
    status_events = [e for e in events if e['direction'] == 'rx' and e['can_id'] == '0x507']
    wheel_events = [e for e in events if e['direction'] == 'rx' and e['can_id'] == '0x50a']
    if any(e['direction'] == 'tx' for e in events):
        return {'watchdog_stopped': False, 'error_evidence': 'TX during silence'}
    exits = [e for e in status_events if bytes.fromhex(e['payload'])[4] == 0]
    zero_streak = 0
    settled = None
    for e in wheel_events:
        zero_streak = zero_streak + 1 if all(abs(v) <= 10 for v in parse_wheel_feedback(bytes.fromhex(e['payload']))) else 0
        if zero_streak >= 3 and settled is None:
            settled = e['t_s']
    exit_s = exits[0]['t_s'] if exits else None
    remaining_exited = bool(exits) and all(bytes.fromhex(e['payload'])[4] == 0 for e in status_events if e['t_s'] >= exit_s)
    final_zero = len(wheel_events) >= 3 and all(all(abs(v) <= 10 for v in parse_wheel_feedback(bytes.fromhex(e['payload']))) for e in wheel_events[-3:])
    passed = (exit_s is not None and settled is not None and
              0.45 <= exit_s-last_tx_s <= WATCHDOG_API_EXIT_DEADLINE_S and
              settled-last_tx_s <= WATCHDOG_STOP_CONFIRM_DEADLINE_S and remaining_exited and final_zero)
    return {'watchdog_stopped': passed,
            'api_exit_deadline_s': WATCHDOG_API_EXIT_DEADLINE_S,
            'stop_confirmation_deadline_s': WATCHDOG_STOP_CONFIRM_DEADLINE_S,
            'api_exit_after_last_tx_s': None if exit_s is None else exit_s-last_tx_s,
            'settled_after_last_tx_s': None if settled is None else settled-last_tx_s}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=('standby', 'forward', 'forward-02', 'forward-04', 'left', 'remote-stop', 'watchdog', 'locked-zero', 'wheel-revolution'), required=True)
    parser.add_argument('--lifter-gear', type=int, choices=range(1, 6), required=True)
    parser.add_argument('--wheels-raised', action='store_true')
    parser.add_argument('--blade-isolated', action='store_true')
    parser.add_argument('--operator-ready', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('Hardware bench runs only on RK3588')
    if not (args.wheels_raised and args.blade_isolated and args.operator_ready):
        parser.error('All physical bench conditions must be confirmed')
    if args.output.exists():
        parser.error('Use a new evidence file')
    def interrupt(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    events = []
    status = wheel = None
    last_status = last_wheel = -float('inf')
    start = time.monotonic()
    result = {'action': args.action, 'lifter_gear': args.lifter_gear,
              'blade_isolated': True, 'wheels_raised': True,
              'automatic_unlock': False, 'navigation_started': False}
    bag = 0
    transmitted = False
    running = False
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.bind(('can0',))
        bus.setblocking(False)
        def receive():
            nonlocal status, wheel, last_status, last_wheel
            for _ in range(128):
                try:
                    raw = bus.recv(16)
                except BlockingIOError:
                    break
                cid, dlc, data = FRAME.unpack(raw)
                now = time.monotonic()
                if cid == 0x005:
                    raise RuntimeError('Other 0x005 publisher observed; abort')
                if cid in (1, 2, 3, 4) and args.action in ('wheel-revolution', 'forward-02', 'forward-04'):
                    # Receive only: never issue commands to motor-driver IDs.
                    events.append({'t_s': now-start, 'direction': 'rx',
                                   'can_id': hex(cid), 'dlc': dlc,
                                   'payload': data[:dlc].hex()})
                    continue
                if dlc != 8 or cid not in (0x507, 0x50A):
                    continue
                events.append({'t_s': now-start, 'direction': 'rx',
                               'can_id': hex(cid), 'payload': data.hex()})
                if cid == 0x507:
                    status, last_status = data, now
                else:
                    wheel, last_wheel = parse_wheel_feedback(data), now
        def fresh():
            now = time.monotonic()
            if now-last_status > 0.25 or now-last_wheel > 0.25:
                raise RuntimeError('Status/wheel feedback stale; abort')
        def send(mode, action='zero'):
            nonlocal transmitted
            payload = bench_payload(mode, action, args.lifter_gear, bag)
            bus.send(FRAME.pack(0x005, 8, payload))
            transmitted = True
            events.append({'t_s': time.monotonic()-start, 'direction': 'tx',
                           'can_id': '0x5', 'payload': payload.hex()})
        def phase(mode, action, seconds, acknowledgement=None):
            nonlocal running
            begin = time.monotonic()
            next_tx = begin
            previous_tx = begin
            while time.monotonic()-begin < seconds:
                receive()
                fresh()
                now = time.monotonic()
                if running and status[4] != 2:
                    raise RuntimeError('STM32 exited API run; no reacquisition')
                if action != 'zero' and any(abs(v) > 500 for v in wheel):
                    raise RuntimeError('Unexpected wheel speed exceeds bench ceiling')
                if action in ('forward', 'forward-02', 'forward-04') and any(v < -20 for v in wheel):
                    raise RuntimeError('Negative forward wheel feedback; abort')
                if now >= next_tx:
                    if action != 'zero' and now-previous_tx > 0.12:
                        raise RuntimeError('Command scheduling gap; abort')
                    send(mode, action)
                    previous_tx = now
                    next_tx = now+0.05
                if acknowledgement is not None and status[4] == acknowledgement:
                    if acknowledgement == 2:
                        running = True
                    return
                time.sleep(0.005)
            if acknowledgement is not None:
                raise RuntimeError(f'No STM32 API {acknowledgement} acknowledgement')
        try:
            until = time.monotonic()+1.0
            while time.monotonic() < until:
                receive()
                time.sleep(0.005)
            fresh()
            if status[4] != 0 or any(abs(v) > 10 for v in wheel):
                raise RuntimeError('Preflight requires idle mode and stationary wheels')
            bag = status[7] & 1
            result['preserved_bag_target'] = bag
            if args.action == 'locked-zero':
                begin = time.monotonic()
                next_tx = begin
                while time.monotonic()-begin < 1.0:
                    receive()
                    fresh()
                    if status[4] != 0 or any(abs(v) > 10 for v in wheel):
                        raise RuntimeError('Locked zero probe restored API or wheels moved')
                    if time.monotonic() >= next_tx:
                        send(2, 'zero')
                        next_tx = time.monotonic()+0.05
                    time.sleep(0.005)
                result['run_only_resume_rejected'] = True
                result['stopped_feedback'] = list(wheel)
            else:
                phase(1, 'zero', 2.0, acknowledgement=1)
                result['standby_acknowledged'] = True
                if args.action != 'standby':
                    phase(2, 'zero', 2.0, acknowledgement=2)
                    result['run_acknowledged'] = True
                    phase(2, 'zero', 0.5)
                    motion_start = time.monotonic()-start
                    if args.action == 'remote-stop':
                        begin = time.monotonic()
                        next_tx = begin
                        previous_tx = begin
                        responded = [False] * 4
                        stop_seen = False
                        while time.monotonic()-begin < 8.0:
                            receive()
                            fresh()
                            now = time.monotonic()
                            responded = [old or abs(v) >= 15 for old, v in zip(responded, wheel)]
                            if status[4] != 2 or (all(responded) and all(abs(v) <= 10 for v in wheel)):
                                result['remote_stop_observed_s'] = now-start
                                result['remote_stop_api_mode'] = status[4]
                                stop_seen = True
                                break
                            if any(v < -20 or v > 500 for v in wheel):
                                raise RuntimeError('Unexpected forward wheel feedback; abort')
                            if now >= next_tx:
                                if now-previous_tx > 0.12:
                                    raise RuntimeError('Command scheduling gap; abort')
                                send(2, 'forward')
                                previous_tx = now
                                next_tx = now+0.05
                            time.sleep(0.005)
                        if not stop_seen or not all(responded):
                            raise RuntimeError('Remote stop missing or occurred before wheel response')
                        # Keep zero commands only after observed stop; never reacquire.
                        begin = time.monotonic()
                        next_tx = begin
                        while time.monotonic()-begin < 1.0:
                            receive()
                            fresh()
                            if time.monotonic() >= next_tx:
                                send(2, 'zero')
                                next_tx = time.monotonic()+0.05
                            time.sleep(0.005)
                        if any(abs(v) > 10 for v in wheel):
                            raise RuntimeError('Remote stop did not remain stationary')
                        result['all_wheels_responded'] = True
                        result['remote_stop_held'] = True
                    else:
                        result['motion_begin_t_s'] = motion_start
                        motion_action, motion_seconds = bench_motion_profile(args.action)
                        result['motion_duration_limit_s'] = motion_seconds
                        phase(2, motion_action, motion_seconds)
                        result['zero_phase_begin_t_s'] = time.monotonic()-start
                        samples = [parse_wheel_feedback(bytes.fromhex(e['payload']))
                                   for e in events if e['direction'] == 'rx' and
                                   e['can_id'] == '0x50a' and e['t_s'] >= motion_start]
                        result['motion_wheel_samples'] = [list(v) for v in samples]
                        response = bool(samples) and all(
                            max(abs(v[i]) for v in samples) >= 15 for i in range(4))
                        result['all_wheels_responded'] = response
                        if not response:
                            raise RuntimeError('API acknowledged but wheel response insufficient')
                        if args.action == 'watchdog':
                            last_tx_s = next(e['t_s'] for e in reversed(events) if e['direction'] == 'tx')
                            result['silence_after_tx_s'] = last_tx_s
                            begin = time.monotonic()
                            # Deliberately no TX for 2 seconds; feedback must remain fresh.
                            while time.monotonic()-begin < 2.0:
                                receive()
                                fresh()
                                if any(abs(v) > 500 for v in wheel):
                                    raise RuntimeError('Unexpected wheel speed during silence')
                                time.sleep(0.005)
                            silence_events = [e for e in events if e['t_s'] > last_tx_s]
                            result.update(watchdog_evidence(silence_events, last_tx_s))
                            if not result['watchdog_stopped']:
                                raise RuntimeError('Watchdog stop/mode exit not observed within deadline')
                            # Resume only RUN-with-zero, never STANDBY or unlock.
                            begin = time.monotonic()
                            next_tx = begin
                            while time.monotonic()-begin < 1.0:
                                receive()
                                fresh()
                                if status[4] != 0 or any(abs(v) > 10 for v in wheel):
                                    raise RuntimeError('RUN-only zero frames restored API or wheels moved')
                                if time.monotonic() >= next_tx:
                                    send(2, 'zero')
                                    next_tx = time.monotonic()+0.05
                                time.sleep(0.005)
                            result['run_only_resume_rejected'] = True
                        else:
                            phase(2, 'zero', 1.5)
                        if any(abs(v) > 10 for v in wheel):
                            raise RuntimeError('Wheels not stationary after stop')
                    result['stopped_feedback'] = list(wheel)
                    result['stop_observation_end_t_s'] = time.monotonic()-start
            result['completed'] = True
        except (Exception, KeyboardInterrupt) as exc:
            result['completed'] = False
            result['error'] = str(exc) or 'interrupted'
        finally:
            if transmitted:
                # Explicit zero + API exit; byte 7 stays zero. No retry takeover.
                for _ in range(6):
                    try:
                        send(0)
                    except OSError:
                        break
                    time.sleep(0.05)
                until = time.monotonic()+0.6
                while time.monotonic() < until:
                    try:
                        receive()
                    except RuntimeError:
                        break
                    time.sleep(0.005)
            result.update(final_mode=status[4] if status else None,
                          final_wheel_mm_s=list(wheel) if wheel else None,
                          elapsed_s=time.monotonic()-start, events=events)
            args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k != 'events'}, indent=2))
    return 0 if result.get('completed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
