"""Create a separate static map with explicitly reviewed keep-out rectangles."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil

import numpy as np
import yaml
from mower_nav2.visibility_mapping import keepout_rectangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--zones-json', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('Map asset processing runs on RK3588')
    if args.output_dir.exists():
        parser.error('Output exists; preserve earlier map versions')
    zones = json.loads(args.zones_json.read_text())
    if zones.get('user_confirmed_keepouts') is not True:
        parser.error('User-confirmed keepouts required')
    config = yaml.safe_load((args.source/'map.yaml').read_text())
    image_path = args.source/config['image']
    with image_path.open('rb') as stream:
        if stream.readline().strip() != b'P5':
            parser.error('Expected binary PGM')
        width, height = map(int, stream.readline().split())
        if stream.readline().strip() != b'255':
            parser.error('Expected 8-bit PGM')
        grid = np.frombuffer(stream.read(), np.uint8).reshape(height, width)[::-1]
    output, mask = keepout_rectangles(grid, zones['rectangles_xy_m'],
        config['origin'][:2], config['resolution'], zones['digitization_margin_m'])
    out = args.output_dir
    out.mkdir(parents=True)
    (out/'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+output[::-1].tobytes())
    config['image'] = 'map.pgm'
    (out/'map.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    (out/'map.pcd').symlink_to((args.source/'map.pcd').resolve())
    for name in ('livox.json', 'mid360_real.yaml', 'metadata.json', 'acceptance.md'):
        shutil.copy2(args.source/name, out/name)
    record = dict(zones, source=str(args.source.resolve()),
        source_pgm_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),
        keepout_cells=int(mask.sum()), changed_cells=int((grid != output).sum()),
        outside_unchanged=bool(np.array_equal(grid[~mask], output[~mask])),
        map_reviewed=False, motion_approved=False)
    (out/'keepouts.json').write_text(json.dumps(record, indent=2))
    print(json.dumps(record, indent=2), flush=True)


if __name__ == '__main__':
    main()
