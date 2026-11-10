#!/usr/bin/env python3
"""Host-only allowlisted source deployment, with immutable board backups."""
from pathlib import Path
import hashlib
import io
import json
import shlex
import subprocess
import tarfile


def main():
    root=Path(__file__).resolve().parents[3]
    paths=[p for p in (root/'src/mower_console').rglob('*') if p.is_file() and
           not any(v in p.parts for v in ('node_modules','__pycache__','.pytest_cache')) and
           not p.name.endswith('.local.yaml') and p.name != '.env']
    paths += [root/p for p in ('src/fast_lio/src/fixed_map_localizer.cpp',
        'src/mcu_can_bridge/mcu_can_bridge/cmd_vel_can_bridge.py',
        'src/mcu_can_bridge/mcu_can_bridge/motion_permit.py','src/mcu_can_bridge/package.xml',
        'src/mower_nav2/mower_nav2/collision_gate.py','src/mower_nav2/launch/mower_navigation.launch.py',
        'src/mcu_can_bridge/test/test_field_command_gate.py',
        'src/mcu_can_bridge/test/test_motion_permit_can.py','src/mower_nav2/test/test_collision_gate.py')]
    manifest={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w:gz') as t:
        for p in paths:t.add(p,arcname=str(p.relative_to(root)),recursive=False)
    source='''import sys,tarfile,io,json,hashlib,datetime,os
from pathlib import Path
for p in Path('/proc').iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid():continue
 try:args=[v.decode(errors='replace') for v in p.joinpath('cmdline').read_bytes().split(b'\\0') if v]
 except (OSError,ProcessLookupError):continue
 executable=[Path(v).name for v in args[:2]]
 assert not any(v in executable for v in ('cmd_vel_can_bridge','fixed_map_localizer','controller_server','console_server')),'Active vehicle process: '+p.name
 assert not any(Path(a).name=='ros2' and b=='launch' for a,b in zip(args,args[1:])),'Active launch: '+p.name
r=Path('/home/cat/MobileEss_ws');b=Path('/home/cat/MobileEss_data')/('console-deploy-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'));b.mkdir()
manifest=MANIFEST
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r:gz') as t:
 for m in t.getmembers():
  assert m.isfile() and m.name in manifest and '..' not in Path(m.name).parts
  data=t.extractfile(m).read();assert hashlib.sha256(data).hexdigest()==manifest[m.name]
  target=r/m.name;assert str(target.resolve()).startswith(str(r)+'/')
  if target.exists():
   previous=target.read_bytes()
   if previous==data:continue
   backup=b/m.name;backup.parent.mkdir(parents=True,exist_ok=True);backup.write_bytes(previous)
  target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
(b/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(files=len(manifest),backup=str(b))))
'''.replace('MANIFEST',repr(manifest))
    result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','cat@192.168.10.81',
                           'python3 -c '+shlex.quote(source)],input=buffer.getvalue(),capture_output=True)
    if result.returncode:
        print(result.stderr.decode()[-3000:])
        raise SystemExit(result.returncode)
    report={'manifest':manifest,'remote':json.loads(result.stdout)}
    Path('/tmp/mower-console-deployment.json').write_text(json.dumps(report,indent=2))
    print(result.stdout.decode())


if __name__=='__main__':main()
