"""Create a separate candidate after explicit physical review of a startup area."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil

import numpy as np
import yaml
from mower_nav2.visibility_mapping import reviewed_free_disk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--center', type=float, nargs=2, required=True)
    parser.add_argument('--radius', type=float, default=0.65)
    parser.add_argument('--physical-clear-confirmed', action='store_true')
    parser.add_argument('--evidence', required=True)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('Run map asset processing on RK3588')
    if not args.physical_clear_confirmed:
        parser.error('Explicit physical review is required')
    if args.output_dir.exists():
        parser.error('Output already exists; preserve prior candidates')
    config = yaml.safe_load((args.source/'map.yaml').read_text())
    source_image = args.source/config['image']
    with source_image.open('rb') as stream:
        if stream.readline().strip() != b'P5':
            parser.error('Expected captured binary PGM')
        width, height = map(int, stream.readline().split())
        if stream.readline().strip() != b'255':
            parser.error('Expected 8-bit PGM')
        grid = np.frombuffer(stream.read(), np.uint8).reshape(height, width)[::-1]
    output, mask = reviewed_free_disk(grid, args.center, args.radius,
                                     config['origin'][:2], config['resolution'])
    root = args.output_dir
    root.mkdir(parents=True)
    (root/'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+output[::-1].tobytes())
    config['image'] = 'map.pgm'
    (root/'map.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    (root/'map.pcd').symlink_to((args.source/'map.pcd').resolve())
    for name in ('livox.json', 'mid360_real.yaml', 'metadata.json', 'acceptance.md'):
        shutil.copy2(args.source/name, root/name)
    record = {'source': str(args.source.resolve()), 'source_pgm_sha256': hashlib.sha256(source_image.read_bytes()).hexdigest(),
              'review_method': 'human_physical_confirmation', 'center_xy_m': args.center,
              'radius_m': args.radius, 'evidence': args.evidence,
              'review_cells': int(mask.sum()), 'changed_cells': int((grid!=output).sum()),
              'map_reviewed': False, 'motion_approved': False}
    (root/'local_revision.json').write_text(json.dumps(record, indent=2))
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    main()
