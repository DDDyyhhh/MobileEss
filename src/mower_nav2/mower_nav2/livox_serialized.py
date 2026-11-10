"""Decode the XYZ samples used by obstacle filtering from Livox CDR v1.

FAST-LIO still receives the original CustomMsg. This reader skips unused point
fields and applies the same index stride before allocating Python point objects.
"""
from dataclasses import dataclass
import struct
from typing import NamedTuple


class XYZ(NamedTuple):
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class LivoxXYZFrame:
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    point_count: int
    points: tuple[XYZ, ...]


def decode_livox_xyz(data: bytes, point_stride: int = 4) -> LivoxXYZFrame:
    """Read the repository's CustomMsg/CustomPoint schema; reject bad lengths.

    CustomPoint has 19 data bytes and alignment 4, hence a 20-byte stride.
    Alignment is relative to the payload after the four-byte encapsulation.
    Both CDR v1 byte orders are supported; other encodings fail closed.
    """
    if not isinstance(point_stride, int) or point_stride < 1:
        raise ValueError('point_stride must be a positive integer')
    if len(data) < 16 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('Unsupported or truncated Livox CDR encapsulation')
    endian = '<' if data[1] == 1 else '>'
    sec, nanosec, frame_size = struct.unpack_from(endian + 'iII', data, 4)
    if nanosec >= 1_000_000_000 or not 1 <= frame_size <= 4096:
        raise ValueError('Invalid Livox header')
    position = 16 + frame_size
    if position > len(data) or data[position - 1] != 0:
        raise ValueError('Truncated or unterminated Livox frame_id')
    try:
        frame_id = data[16:position - 1].decode('utf-8')
    except UnicodeDecodeError as exc:
        raise ValueError('Invalid Livox frame_id encoding') from exc
    position = 4 + ((position - 4 + 7) // 8) * 8
    # timebase(8), point_num(4), lidar_id/rsvd(4), sequence length(4).
    if position + 20 > len(data):
        raise ValueError('Truncated Livox point header')
    point_count = struct.unpack_from(endian + 'I', data, position + 8)[0]
    sequence_count = struct.unpack_from(endian + 'I', data, position + 16)[0]
    position += 20
    if point_count != sequence_count:
        raise ValueError('Livox point_num does not match sequence length')
    required = position + (20 * point_count - 1 if point_count else 0)
    if not required <= len(data) <= required + 3:
        raise ValueError('Livox point data length does not match schema')
    xyz = struct.Struct(endian + 'fff')
    points = tuple(XYZ(*xyz.unpack_from(data, position + index * 20 + 4))
                   for index in range(0, point_count, point_stride))
    return LivoxXYZFrame(frame_id, sec, nanosec, point_count, points)
