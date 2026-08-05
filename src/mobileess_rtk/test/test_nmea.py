import unittest

from mobileess_rtk.nmea import parse_gga


class TestNmea(unittest.TestCase):
    def test_gga_coordinates_and_quality(self):
        line = '$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42'
        fix = parse_gga(line)
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.latitude, 48.1173, places=4)
        self.assertAlmostEqual(fix.longitude, 11.5166667, places=4)
        self.assertEqual(fix.quality, 4)

    def test_south_west_coordinates(self):
        line = '$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48'
        fix = parse_gga(line)
        self.assertIsNotNone(fix)
        self.assertLess(fix.latitude, 0)
        self.assertLess(fix.longitude, 0)

    def test_invalid_sentence(self):
        self.assertIsNone(parse_gga('$GPGGA,invalid'))


if __name__ == '__main__':
    unittest.main()
