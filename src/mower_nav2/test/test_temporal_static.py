import numpy as np
from mower_nav2.visibility_mapping import TemporalStaticGrid


def mask(cell=None):
    result = np.zeros((5, 5), dtype=bool)
    if cell is not None:
        result[cell] = True
    return result


def test_person_return_clears_only_after_repeated_later_visibility():
    history = TemporalStaticGrid((5, 5))
    for _ in range(3):
        history.observe(mask((2, 2)), mask())
    assert history.result()[2, 2] == 0
    for _ in range(4):
        history.observe(mask(), mask((2, 2)))
    assert history.result()[2, 2] == 0
    history.observe(mask(), mask((2, 2)))
    assert history.result()[2, 2] == 254


def test_persistent_wall_wins_over_same_scan_free_ray():
    history = TemporalStaticGrid((5, 5))
    for _ in range(10):
        history.observe(mask((2, 2)), mask((2, 2)))
    assert history.result()[2, 2] == 0


def test_unseen_after_person_remains_blocked_not_filled_free():
    history = TemporalStaticGrid((5, 5))
    history.observe(mask((2, 2)), mask())
    for _ in range(10):
        history.observe(mask(), mask((1, 1)))
    assert history.result()[2, 2] == 205
    assert history.result()[4, 4] == 205


def test_new_obstacle_resets_old_free_evidence():
    history = TemporalStaticGrid((5, 5))
    for _ in range(8):
        history.observe(mask(), mask((2, 2)))
    history.observe(mask((2, 2)), mask((2, 2)))
    assert history.result()[2, 2] == 205
    history.observe(mask((2, 2)), mask())
    history.observe(mask((2, 2)), mask())
    assert history.result()[2, 2] == 0
