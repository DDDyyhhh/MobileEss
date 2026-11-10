import numpy as np
import pytest
from mower_nav2.visibility_mapping import keepout_rectangles


def test_intersecting_cells_blocked_with_world_origin_and_preserved_outside():
    grid = np.full((5, 5), 254, np.uint8)
    grid[4, 4] = 205
    output, mask = keepout_rectangles(grid, [[-0.75, 2.15, -0.55, 2.35]],
                                      [-1., 2.], 0.1)
    expected = np.zeros((5, 5), bool)
    expected[1:4, 2:5] = True
    np.testing.assert_array_equal(mask, expected)
    assert np.all(output[mask] == 0)
    np.testing.assert_array_equal(output[~mask], grid[~mask])
    assert grid[1, 2] == 254


def test_margin_expands_keepout_and_invalid_bounds_rejected():
    grid = np.full((5, 5), 205, np.uint8)
    _, mask = keepout_rectangles(grid, [[0.21, 0.21, 0.29, 0.29]], [0., 0.], 0.1, 0.05)
    assert mask.sum() == 9
    with pytest.raises(ValueError):
        keepout_rectangles(grid, [[1., 0., 0., 1.]], [0., 0.], 0.1)
