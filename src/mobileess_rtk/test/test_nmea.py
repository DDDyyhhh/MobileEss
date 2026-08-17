import unittest

from mobileess_rtk.nmea import parse_gga, parse_uniheading


class TestNmea(unittest.TestCase):
    def test_gga_coordinates_and_quality(self):
        line = '$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42'
        fix = parse_gga(line)
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.latitude, 48.1173, places=4)
        self.assertAlmostEqual(fix.longitude, 11.5166667, places=4)
        self.assertEqual(fix.quality, 4)
        self.assertAlmostEqual(fix.hdop, 0.9)

    def test_south_west_coordinates(self):
        line = '$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48'
        fix = parse_gga(line)
        self.assertIsNotNone(fix)
        self.assertLess(fix.latitude, 0)
        self.assertLess(fix.longitude, 0)

    def test_invalid_sentence(self):
        self.assertIsNone(parse_gga('$GPGGA,invalid'))

    def test_uniheading(self):
        line = '#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;SOL_COMPUTED,NARROW_INT,0.650,270.0000,0.0000,0.0000,0.1000,0.1000*00'
        fix = parse_uniheading(line)
        self.assertIsNotNone(fix)
        self.assertEqual(fix.solution_status, 'SOL_COMPUTED')
        self.assertAlmostEqual(fix.baseline_m, 0.65)
        self.assertAlmostEqual(fix.heading_deg, 270.0)


if __name__ == '__main__':
    unittest.main()
