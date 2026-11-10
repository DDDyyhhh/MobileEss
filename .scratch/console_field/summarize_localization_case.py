#!/usr/bin/env python3
"""Summarize an already completed field recording on RK3588, without ROS output."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import struct
import sys

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


case = sys.argv[1]
if not re.fullmatch(r'[A-E][1-4]', case):
    raise SystemExit('Expected case A1 through E4')
root = Path('/home/cat/MobileEss_data')
paths = json.loads((root / ('console-field-' + case + '-latest.json')).read_text())
result = json.loads(Path(paths['result']).read_text())
bag = Path(paths['bag'])
database = next(bag.glob('*.db3'))
with sqlite3.connect('file:' + str(database) + '?mode=ro', uri=True) as connection:
    topics = connection.execute('select id,name,type from topics').fetchall()
    counts = {name: connection.execute('select count(*) from messages where topic_id=?', (tid,)).fetchone()[0]
              for tid, name, _ in topics}
    diagnostic_id = next(tid for tid, name, _ in topics if name == '/fixed_map_localization/diagnostics')
    events = []
    for stamp, data in connection.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                                          (diagnostic_id,)):
        for status in deserialize_message(data, DiagnosticArray).status:
            if status.name == 'fixed_map_localization':
                values = {value.key: value.value for value in status.values}
                events.append(dict(stamp_ns=stamp, event=status.message,
                                   generation=values.get('generation'), accepted_updates=values.get('accepted_updates')))
transitions = []
for event in events:
    if not transitions or (event['event'], event['generation']) != (transitions[-1]['event'], transitions[-1]['generation']):
        transitions.append(event)
summary = {key: value for key, value in result.items() if key not in ('states', 'can_frames')}
summary.update(bag_topic_counts=counts, diagnostic_event_counts=dict(Counter(e['event'] for e in events)),
               diagnostic_transitions=transitions,
               bag_files={p.name: dict(bytes=p.stat().st_size, sha256=digest(p)) for p in sorted(bag.iterdir()) if p.is_file()},
               all_can_api_modes=sorted(set(bytes.fromhex(f['data'])[4] for f in result['can_frames'] if f['id'] == 0x507)),
               max_actual_wheel_mm_s=max((abs(v) for f in result['can_frames'] if f['id'] == 0x50A
                                          for v in struct.unpack('>hhhh', bytes.fromhex(f['data']))), default=None),
               case_harness_sha256=digest(Path('/tmp/console_field_case.py')))
if 'after' in result and result['before'].get('pose'):
    before, after = result['before']['pose'], result['after']['pose']
    summary.update(position_change_from_previous_match_m=math.dist(before[:2], after[:2]),
                   yaw_change_deg=math.degrees(math.atan2(math.sin(after[2]-before[2]), math.cos(after[2]-before[2]))))
output = root / ('console-field-' + case + '-summary-20261021.json')
output.write_text(json.dumps(summary, indent=2, ensure_ascii=False))
print(json.dumps(dict(summary=str(output), paths=paths, software_result=result['software_result'],
                      topic_counts=counts, diagnostic_event_counts=summary['diagnostic_event_counts'],
                      control_frames=result['control_frames'], all_can_api_modes=summary['all_can_api_modes'],
                      max_actual_wheel_mm_s=summary['max_actual_wheel_mm_s'],
                      bag_total_bytes=sum(p.stat().st_size for p in bag.iterdir() if p.is_file())), indent=2))
