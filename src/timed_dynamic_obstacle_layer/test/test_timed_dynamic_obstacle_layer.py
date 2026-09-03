from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_plugin_exports_timed_dynamic_layer():
    xml = (ROOT / 'src' / 'timed_dynamic_obstacle_layer' /
           'timed_dynamic_obstacle_layer.xml').read_text()
    source = (ROOT / 'src' / 'timed_dynamic_obstacle_layer' / 'src' /
              'timed_dynamic_obstacle_layer.cpp').read_text()
    assert 'TimedDynamicObstacleLayer' in xml
    assert 'PLUGINLIB_EXPORT_CLASS' in source
    assert 'ttl_seconds' in source
    assert 'source_timeout' in source
    assert 'hasHealthyInput' in source
    assert 'resetMapToValue' in source
