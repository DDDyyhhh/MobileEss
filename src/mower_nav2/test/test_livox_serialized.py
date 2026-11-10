"""Binary reader equivalence and rejection tests; no hardware dependencies."""
import math
import struct
import pytest

from mower_nav2.livox_serialized import decode_livox_xyz
from mower_nav2.livox_costmap_evidence import filter_livox_costmap_evidence


def encoded(points, frame_id='livox_frame', endian='<'):
    body = bytearray(struct.pack(endian + 'iI', 17, 123456))
    name = frame_id.encode() + b'\0'
    body.extend(struct.pack(endian + 'I', len(name)) + name)
    body.extend(b'\0' * (-len(body) % 8))
    body.extend(struct.pack(endian + 'QIB3sI', 999, len(points), 1, b'\0'*3, len(points)))
    for i, point in enumerate(points):
        body.extend(b'\0' * (-len(body) % 4))
        body.extend(struct.pack(endian + 'IfffBBB', i, *point, 15, 16, 2))
    return (b'\0\1\0\0' if endian == '<' else b'\0\0\0\0') + body


@pytest.mark.parametrize('endian', ['<', '>'])
@pytest.mark.parametrize('name', ['', 'a', 'livox_frame', 'frame_with_17chars'])
@pytest.mark.parametrize('stride', [1, 4, 7])
def test_reader_preserves_header_xyz_and_original_stride(endian, name, stride):
    points = [(i*.25, -i*.5, .125) for i in range(13)]
    frame = decode_livox_xyz(encoded(points, name, endian), stride)
    assert (frame.frame_id, frame.stamp_sec, frame.stamp_nanosec) == (name, 17, 123456)
    assert frame.point_count == len(points)
    assert list(frame.points) == points[::stride]


def test_empty_cloud_and_nonfinite_returns_keep_original_filter_semantics():
    assert decode_livox_xyz(encoded([])).points == ()
    frame = decode_livox_xyz(encoded([(math.nan, 0., 0.)]), 1)
    assert filter_livox_costmap_evidence(frame.points, point_stride=1) == ([], [])


@pytest.mark.parametrize('cut', [1, 4, 12, 30])
def test_truncated_points_are_rejected(cut):
    with pytest.raises(ValueError):
        decode_livox_xyz(encoded([(1., 2., 3.)]*8)[:-cut])


def test_mismatched_counts_and_unsupported_encoding_are_rejected():
    data = bytearray(encoded([(1., 2., 3.)], 'a'))
    struct.pack_into('<I', data, 4+16+8, 7)
    with pytest.raises(ValueError, match='point_num'):
        decode_livox_xyz(data)
    with pytest.raises(ValueError, match='encapsulation'):
        decode_livox_xyz(b'\0\7'+bytes(data[2:]))


def test_raw_and_message_paths_produce_identical_obstacle_and_ground_evidence():
    points = [(x/8, y/8, -.5 + .01*x/8) for x in range(4, 25) for y in range(-8, 9)]
    points += [(1.01+i*.001, .21, .25) for i in range(48)]
    data = encoded(points)
    raw = decode_livox_xyz(data, 4)
    # Decode all points to match float32 rounding of ROS deserialization.
    original = decode_livox_xyz(data, 1).points
    expected = filter_livox_costmap_evidence(original, point_stride=4)
    actual = filter_livox_costmap_evidence(raw.points, point_stride=1)
    assert actual == expected
    assert actual[0] and actual[1]


def test_reader_matches_actual_ros_serializer_when_ros_is_available():
    serialization = pytest.importorskip('rclpy.serialization')
    msg = pytest.importorskip('livox_ros_driver2.msg')
    cloud = msg.CustomMsg()
    cloud.header.frame_id = 'livox_frame'
    cloud.header.stamp.sec = 123
    cloud.header.stamp.nanosec = 456
    cloud.timebase = 789
    for i in range(11):
        point = msg.CustomPoint()
        point.offset_time = i
        point.x, point.y, point.z = float(i), .5, -.25
        cloud.points.append(point)
    cloud.point_num = len(cloud.points)
    raw = decode_livox_xyz(serialization.serialize_message(cloud), 4)
    assert (raw.frame_id, raw.stamp_sec, raw.stamp_nanosec) == ('livox_frame', 123, 456)
    assert list(raw.points) == [(p.x, p.y, p.z) for p in cloud.points[::4]]


def test_node_paths_publish_identical_clouds_and_fail_closed_on_bad_data():
    serialization = pytest.importorskip('rclpy.serialization')
    msg = pytest.importorskip('livox_ros_driver2.msg')
    from builtin_interfaces.msg import Time
    from types import SimpleNamespace
    from mower_nav2.livox_obstacle_cloud import LivoxObstacleCloud
    cloud = msg.CustomMsg()
    cloud.header.frame_id = 'livox_frame'
    for x, y, z in [(1.+i*.001, .21, .25) for i in range(48)] + [
            (x/8, y/8, -.5) for x in range(4,25) for y in range(-8,9)]:
        point = msg.CustomPoint()
        point.x, point.y, point.z = float(x), float(y), float(z)
        cloud.points.append(point)
    cloud.point_num = len(cloud.points)
    node = object.__new__(LivoxObstacleCloud)
    node._params = dict(point_stride=4)
    node._stamp_with_receipt_time = True
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=10, nanosec=20)))
    outputs, errors = [], []
    node._publisher = SimpleNamespace(publish=outputs.append)
    node.get_logger = lambda: SimpleNamespace(error=errors.append)
    node._cloud_callback(cloud)
    node._serialized_callback(serialization.serialize_message(cloud))
    assert len(outputs) == 2 and outputs[0] == outputs[1]
    node._serialized_callback(b'bad')
    assert len(outputs) == 2 and errors
