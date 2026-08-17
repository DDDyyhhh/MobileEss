from pathlib import Path
import ast


ROOT = Path(__file__).resolve().parents[1]


def test_map_capture_launch_requires_explicit_pcd_destination_and_no_tf():
    launch = ROOT / 'launch' / 'map_capture.launch.py'
    text = launch.read_text()
    ast.parse(text)
    assert "DeclareLaunchArgument(\n            'map_file_path'" in text
    assert "'pcd_save.pcd_save_en': True" in text
    assert "'map_file_path': LaunchConfiguration('map_file_path')" in text
    assert "'publish.tf_en': False" in text
    assert "'common.odom_frame': 'map_capture_odom'" in text


def test_map_save_rejects_an_empty_destination():
    source = (ROOT / 'src' / 'laserMapping.cpp').read_text()
    callback = source[source.index('void map_save_callback'):]
    assert 'if (map_file_path.empty())' in callback
    assert 'Map save path is empty.' in callback
    assert 'if (ikdtree.validnum() == 0)' in callback
    assert 'Map is empty.' in callback
    assert 'save_to_pcd();' in callback


def test_map_save_flattens_the_accumulated_ikd_tree():
    source = (ROOT / 'src' / 'laserMapping.cpp').read_text()
    save = source[source.index('void save_to_pcd()'):source.index('void publish_odometry')]
    assert 'ikdtree.flatten(ikdtree.Root_Node, map_points, NOT_RECORD);' in save
    assert 'pcd_writer.writeBinary(map_file_path, map_cloud);' in save


def test_map_capture_documents_explicit_map_save_service():
    launch = ROOT / 'launch' / 'map_capture.launch.py'
    assert 'call /map_save before stopping it' in launch.read_text()
