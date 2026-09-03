import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'clean_occupancy_components', ROOT / 'scripts' / 'clean_occupancy_components.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_small_components_are_removed_but_real_connected_structure_remains():
    image = np.full((8, 10), 254, dtype=np.uint8)
    image[1, 1] = 0
    image[2, 2] = 0
    image[5:7, 6:9] = 0
    cleaned, removed = MODULE.remove_small_occupied_components(
        image, min_component_size=3, connectivity=8)
    assert removed == 2
    assert cleaned[1, 1] == 254
    assert cleaned[2, 2] == 254
    assert np.all(cleaned[5:7, 6:9] == 0)


def test_unknown_and_free_pixels_are_not_modified():
    image = np.full((4, 4), 205, dtype=np.uint8)
    image[0, 0] = 0
    image[3, 3] = 254
    cleaned, _ = MODULE.remove_small_occupied_components(image, min_component_size=2)
    assert cleaned[0, 0] == 254
    assert cleaned[3, 3] == 254
    assert np.all(cleaned[image == 205] == 205)
