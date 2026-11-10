"""Read fixed map assets; never accepts browser-supplied file paths."""
import hashlib
from pathlib import Path
import re

import numpy as np
import yaml

from .geometry import Grid


class MapAssets:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        yaml_path = self.directory / 'map.yaml'
        cfg = yaml.safe_load(yaml_path.read_text())
        image = (self.directory / cfg['image']).resolve()
        if image.parent != self.directory:
            raise ValueError('地图图像必须位于地图目录内')
        raw = image.read_bytes()
        # P5 PGM with whitespace/comments in its header.
        match = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(?:#[^\n]*\n\s*)*(\d+)\s+(?:#[^\n]*\n\s*)*(\d+)\s', raw)
        if not match or int(match[3]) != 255:
            raise ValueError('地图需要8位P5 PGM')
        w, h = int(match[1]), int(match[2])
        pixels = np.frombuffer(raw[match.end():], dtype=np.uint8)
        if pixels.size != w*h:
            raise ValueError('PGM大小不匹配')
        values = pixels.reshape(h, w)[::-1].reshape(-1).astype(float)/255.
        probability = values if cfg.get('negate', 0) else 1.-values
        data = np.full(w*h, -1, dtype=np.int16)
        data[probability < cfg.get('free_thresh', .196)] = 0
        data[probability > cfg.get('occupied_thresh', .65)] = 100
        self.grid = Grid(w, h, float(cfg['resolution']), tuple(cfg['origin']), data.tolist())
        self.tracked_files = [yaml_path, image] + [self.directory/n for n in
                              ('map.pcd','livox.json','mid360_real.yaml') if (self.directory/n).is_file()]
        digest = hashlib.sha256()
        for path in self.tracked_files:
            digest.update(path.name.encode())
            with path.open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024), b''):
                    digest.update(block)
        self.revision = digest.hexdigest()
        self.fingerprints = {path:(path.stat().st_size,path.stat().st_mtime_ns) for path in self.tracked_files}
        self.description = dict(width=w, height=h, resolution=self.grid.resolution,
                                origin=list(self.grid.origin), data=self.grid.data,
                                revision=self.revision, frame='map')
        self._points = None

    def unchanged(self):
        try:
            return all((p.stat().st_size,p.stat().st_mtime_ns)==v for p,v in self.fingerprints.items())
        except OSError:
            return False

    def points(self):
        if self._points is None:
            self._points = read_pcd(self.directory/'map.pcd')
            self._points = downsample(self._points, .20, 100000).astype('<f4').tobytes()
        return self._points


def read_pcd(path):
    with Path(path).open('rb') as stream:
        header = {}
        for _ in range(100):
            line = stream.readline().decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *value = line.split()
            header[key] = value
            if key == 'DATA':
                break
        names = header['FIELDS']
        if header['DATA'] == ['ascii']:
            rows = np.atleast_2d(np.loadtxt(stream))
            return rows[:, [names.index(n) for n in ('x', 'y', 'z')]]
        if header['DATA'] != ['binary'] or any(v != '1' for v in header.get('COUNT', ['1'])):
            raise ValueError('三维地图需ASCII或binary PCD')
        dtype = np.dtype([(name, '<'+kind.lower()+size) for name, kind, size in
                          zip(names, header['TYPE'], header['SIZE'])])
        rows = np.frombuffer(stream.read(), dtype=dtype, count=int(header['POINTS'][0]))
        return np.column_stack([rows[n] for n in ('x', 'y', 'z')])


def downsample(points, voxel, limit):
    points = np.asarray(points).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    _, indices = np.unique(np.floor(points/voxel).astype(np.int32), axis=0, return_index=True)
    points = points[np.sort(indices)]
    return points[::max(1, int(np.ceil(len(points)/limit)))][:limit]
