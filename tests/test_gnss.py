#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''
tests for handling the GNSS message alongside GPS_RAW_INT and GPS2_RAW
'''

import math
import unittest

from pymavlink import mavextra
from pymavlink.dialects.v20 import all as mavlink2

HAVE_GNSS = hasattr(mavlink2, 'MAVLink_gnss_message')

# CMAC, Canberra
LAT_E7 = -353632621
LON_E7 = 1491652374


def gnss(instance=0, fix_type=3, lat=LAT_E7, lon=LON_E7, height_above_geoid=584.0, ground_speed=12.5, cog=9000,
         satellites_used=10, time_boot_ms=1000):
    return mavlink2.MAVLink_gnss_message(
        time_boot_ms=time_boot_ms,
        id=instance,
        fix_type=fix_type,
        lat=lat,
        lon=lon,
        height_above_geoid=height_above_geoid,
        height_above_ellipsoid=height_above_geoid + 20,
        datum=mavlink2.MAV_GNSS_DATUM_WGS84,
        geoid_model=mavlink2.MAV_GNSS_GEOID_MODEL_UNKNOWN,
        ground_speed=ground_speed,
        vertical_speed=0,
        cog=cog,
        satellites_used=satellites_used,
        yaw=65535,
        h_acc=0.5,
        v_acc=0.8,
        vel_acc=100,
        yaw_acc=65535,
    )


def gps_raw_int(fix_type=3, lat=LAT_E7, lon=LON_E7, alt_mm=584000, vel_cms=1250, cog=9000, satellites_visible=10,
                time_usec=1000000):
    return mavlink2.MAVLink_gps_raw_int_message(
        time_usec=time_usec,
        fix_type=fix_type,
        lat=lat,
        lon=lon,
        alt=alt_mm,
        eph=65535,
        epv=65535,
        vel=vel_cms,
        cog=cog,
        satellites_visible=satellites_visible,
    )


@unittest.skipUnless(HAVE_GNSS, "message definitions do not include GNSS")
class TestGNSSPosition(unittest.TestCase):
    def test_get_lat_lon_alt(self):
        (lat, lon, alt) = mavextra.get_lat_lon_alt(gnss())
        self.assertAlmostEqual(math.degrees(lat), LAT_E7 * 1.0e-7)
        self.assertAlmostEqual(math.degrees(lon), LON_E7 * 1.0e-7)
        self.assertAlmostEqual(alt, 584.0)

    def test_distance_two_matches_gps_raw_int(self):
        north_e7 = LAT_E7 + 1000  # about 11m north
        from_gnss = mavextra.distance_two(gnss(), gnss(lat=north_e7))
        from_raw = mavextra.distance_two(gps_raw_int(), gps_raw_int(lat=north_e7))
        self.assertIsNotNone(from_gnss)
        self.assertAlmostEqual(from_gnss, from_raw)


if __name__ == '__main__':
    unittest.main()
