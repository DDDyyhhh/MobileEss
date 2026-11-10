import json
import math
from pathlib import Path
from mower_console.geometry import split_path


def test_lattice_endpoint_resolution_and_direction_semantics():
    cfg=Path(__file__).resolve().parents[1]/'config/lattice_r4_10cm.json'
    data=json.loads(cfg.read_text())
    assert data['lattice_metadata']['grid_resolution']==.1
    assert data['lattice_metadata']['turning_radius']==4.
    kinds=set()
    for primitive in data['primitives']:
        heading=data['lattice_metadata']['heading_angles'][primitive['start_angle_index']]
        poses=[[0.,0.,heading]]+primitive['poses']
        for value in poses[-1][:2]:assert abs(value/.1-round(value/.1))<.001
        kinds.update(s['kind'] for s in split_path(poses))
    assert {'forward','rotate'} <= kinds
    for heading in range(16):
        assert any(p['start_angle_index']==heading and p['end_angle_index']==heading and
                   0<p['trajectory_length']<=.25 for p in data['primitives'])
