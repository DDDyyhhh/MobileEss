"""Small parsers for UM982 GGA and dual-antenna heading output."""
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class GgaFix:
    latitude: float
    longitude: float
    altitude: float
    quality: int
    satellites: int
    hdop: float
    raw: str


@dataclass(frozen=True)
class HeadingFix:
    solution_status: str
    position_type: str
    baseline_m: float
    heading_deg: float
    pitch_deg: float
    heading_stddev_deg: float
    raw: str


def _coordinate(value: str, hemisphere: str, degree_digits: int) -> float:
    if not value or hemisphere not in ('N', 'S', 'E', 'W'):
        raise ValueError('invalid NMEA coordinate')
    degrees = int(value[:degree_digits])
    minutes = float(value[degree_digits:])
    result = degrees + minutes / 60.0
    return -result if hemisphere in ('S', 'W') else result


def _valid_checksum(line: str) -> bool:
    if not line.startswith('$') or '*' not in line:
        return False
    body, checksum = line[1:].split('*', 1)
    if len(checksum) < 2:
        return False
    value = 0
    for char in body:
        value ^= ord(char)
    try:
        return value == int(checksum[:2], 16)
    except ValueError:
        return False


def parse_gga(line: str) -> Optional[GgaFix]:
    """Parse a checksum-valid GGA sentence, returning None for other/invalid lines."""
    line = line.strip()
    if not _valid_checksum(line):
        return None
    body = line[1:].split('*', 1)[0]
    fields = body.split(',')
    if len(fields) < 10 or not fields[0].upper().endswith('GGA'):
        return None
    try:
        quality = int(fields[6] or '0')
        satellites = int(fields[7] or '0')
        hdop = float(fields[8])
        altitude = float(fields[9])
        latitude = _coordinate(fields[2], fields[3], 2)
        longitude = _coordinate(fields[4], fields[5], 3)
    except (TypeError, ValueError, IndexError):
        return None
    return GgaFix(latitude, longitude, altitude, quality, satellites, hdop, line)


def parse_uniheading(line: str) -> Optional[HeadingFix]:
    """Parse UM982 UNIHEADINGA ASCII output."""
    line = line.strip()
    if not line.startswith('#UNIHEADING') or ';' not in line:
        return None
    body = line.split('*', 1)[0]
    try:
        fields = body.split(';', 1)[1].split(',')
        if len(fields) < 8:
            return None
        return HeadingFix(
            solution_status=fields[0].strip().upper(),
            position_type=fields[1].strip().upper(),
            baseline_m=float(fields[2]),
            heading_deg=float(fields[3]),
            pitch_deg=float(fields[4]),
            heading_stddev_deg=float(fields[6]),
            raw=line,
        )
    except (TypeError, ValueError, IndexError):
        return None
