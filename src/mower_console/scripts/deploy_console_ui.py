#!/usr/bin/env python3
"""Stage a frontend-only release; activation requires local sudo authentication.

Does not update ROS sources, calibration, maps, or motion configuration. The
generated activation script stops the existing observation service before
installing assets, builds only mower_console, and restores the frontend on a
failed build. Run without flags to prepare a reviewable release on the board.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import tarfile


ACTIVATE = r'''#!/usr/bin/env bash
set -euo pipefail
release_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
workspace=/home/cat/MobileEss_ws
cd "$release_dir"
python3 verify.py
sudo -v
sudo systemctl stop mower-console.service
rollback() {
  echo "Activation failed; restoring the previous frontend."
  python3 "$release_dir/apply.py" restore
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "$workspace"
  colcon build --packages-select mower_console --symlink-install
  sudo systemctl start mower-console.service
  exit 1
}
trap rollback ERR
python3 apply.py install
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$workspace"
colcon build --packages-select mower_console --symlink-install
sudo systemctl start mower-console.service
systemctl is-active --quiet mower-console.service
trap - ERR
echo "UI_ACTIVATED $release_dir"
'''

APPLY = r'''from pathlib import Path
import json, shutil, sys, tarfile
root=Path('/home/cat/MobileEss_ws/src/mower_console/web')
release=Path(__file__).resolve().parent
if sys.argv[1]=='install':
    if not (release/'previous-web.tar.gz').exists():
        with tarfile.open(release/'previous-web.tar.gz','w:gz') as t:
            for p in root.rglob('*'):
                if p.is_file() and 'node_modules' not in p.parts:
                    t.add(p,arcname=str(p.relative_to(root)),recursive=False)
    manifest=json.loads((release/'manifest.json').read_text())
    shutil.rmtree(root/'dist',ignore_errors=True)
    for name in manifest:
        target=root/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(release/'web'/name,target)
else:
    manifest=json.loads((release/'manifest.json').read_text())
    for name in manifest:
        p=root/name
        if p.is_file(): p.unlink()
    with tarfile.open(release/'previous-web.tar.gz') as t:
        for m in t.getmembers():
            if not m.isfile() or Path(m.name).is_absolute() or '..' in Path(m.name).parts:
                raise ValueError('invalid backup entry')
            p=root/m.name;p.parent.mkdir(parents=True,exist_ok=True)
            p.write_bytes(t.extractfile(m).read())
'''

VERIFY = r'''from pathlib import Path
import hashlib,json,os
release=Path(__file__).resolve().parent
root=Path('/home/cat/MobileEss_ws/src/mower_console/web')
manifest=json.loads((release/'manifest.json').read_text())
for name,digest in manifest.items():
    assert hashlib.sha256((release/'web'/name).read_bytes()).hexdigest()==digest,name
def snapshot():
    result={}
    for base,dirs,files in os.walk(root):
        dirs[:]=[d for d in dirs if d not in ('node_modules','.git')]
        for name in files:
            p=Path(base)/name
            result[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    return result
assert snapshot()==json.loads((release/'expected-board-web.json').read_text()),'Board frontend changed after staging; prepare a new release before activation.'
print('Frontend payload and board baseline verified.')
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='cat@192.168.10.81')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]/'web'
    manifest={}
    for base,dirs,files in os.walk(root):
        dirs[:]=[d for d in dirs if d not in ('node_modules','.git','.cache')]
        for name in files:
            if name.startswith('.env') or name.endswith('.local.yaml'):continue
            p=Path(base)/name
            manifest[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    if 'dist/index.html' not in manifest:
        raise SystemExit('Build the console frontend first.')
    content=io.BytesIO()
    with tarfile.open(fileobj=content,mode='w:gz') as archive:
        for name in sorted(manifest):archive.add(root/name,arcname='web/'+name,recursive=False)
        for name,value in {'activate.sh':ACTIVATE,'apply.py':APPLY,'verify.py':VERIFY,
                           'manifest.json':json.dumps(manifest,indent=2)}.items():
            raw=value.encode();info=tarfile.TarInfo(name);info.size=len(raw)
            archive.addfile(info,io.BytesIO(raw))
    digest=hashlib.sha256(content.getvalue()).hexdigest()
    remote=r'''import datetime,hashlib,io,json,os,sys,tarfile
from pathlib import Path
raw=sys.stdin.buffer.read()
assert hashlib.sha256(raw).hexdigest()==DIGEST
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
release=Path('/home/cat/MobileEss_data')/('console-ui-apple-'+stamp+'-'+DIGEST[:8])
release.mkdir(parents=True)
with tarfile.open(fileobj=io.BytesIO(raw),mode='r:gz') as t:
    for m in t.getmembers():
        assert m.isfile() and not Path(m.name).is_absolute() and '..' not in Path(m.name).parts
        p=release/m.name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes(t.extractfile(m).read())
root=Path('/home/cat/MobileEss_ws/src/mower_console/web')
snapshot={}
for base,dirs,files in os.walk(root):
    dirs[:]=[d for d in dirs if d not in ('node_modules','.git')]
    for name in files:
        p=Path(base)/name
        snapshot[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
(release/'expected-board-web.json').write_text(json.dumps(snapshot,indent=2))
print(json.dumps({'release':str(release),'files':len(snapshot),'activation':str(release/'activate.sh')}))
'''.replace('DIGEST',repr(digest))
    result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',args.host,
                           'python3 -c '+shlex.quote(remote)],input=content.getvalue(),capture_output=True)
    if result.returncode:raise SystemExit(result.stderr.decode())
    report=json.loads(result.stdout)
    print(json.dumps(report,indent=2))
    print('Activate in your terminal:')
    print('ssh -t '+shlex.quote(args.host)+' '+shlex.quote('bash '+shlex.quote(report['activation'])))


if __name__=='__main__':main()
