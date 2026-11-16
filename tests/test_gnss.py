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


def from_system(msg, sysid=1):
    '''give msg a header, as if received from sysid'''
    msg.pack(mavlink2.MAVLink(None, srcSystem=sysid, srcComponent=1))
    return msg


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


@unittest.skipUnless(HAVE_GNSS, "message definitions do not include GNSS")
class TestGPSValues(unittest.TestCase):
    def test_gnss_matches_gps_raw_int(self):
        from_gnss = mavextra.gps_values(gnss())
        from_raw = mavextra.gps_values(gps_raw_int())
        for field in ('instance', 'fix_type', 'satellites'):
            self.assertEqual(getattr(from_gnss, field), getattr(from_raw, field), field)
        for field in ('lat', 'lon', 'alt', 'ground_speed', 'cog', 'time_usec'):
            self.assertAlmostEqual(getattr(from_gnss, field), getattr(from_raw, field), msg=field)

    def test_instances(self):
        self.assertEqual(mavextra.gps_values(gnss(instance=3)).instance, 3)
        gps2 = mavlink2.MAVLink_gps2_raw_message(time_usec=0, fix_type=3, lat=LAT_E7, lon=LON_E7, alt=0, eph=0, epv=0,
                                                 vel=0, cog=0, satellites_visible=0, dgps_numch=0, dgps_age=0)
        self.assertEqual(mavextra.gps_values(gps2).instance, 1)

    def test_unknown_values(self):
        values = mavextra.gps_values(gnss(height_above_geoid=float('nan'), ground_speed=float('nan'), cog=65535,
                                          satellites_used=255))
        self.assertIsNone(values.alt)
        self.assertIsNone(values.ground_speed)
        self.assertIsNone(values.cog)
        self.assertIsNone(values.satellites)
        values = mavextra.gps_values(gps_raw_int(vel_cms=65535, cog=65535, satellites_visible=255))
        self.assertIsNone(values.ground_speed)
        self.assertIsNone(values.cog)
        self.assertIsNone(values.satellites)

    def test_other_messages(self):
        self.assertIsNone(mavextra.gps_values(mavlink2.MAVLink_heartbeat_message(0, 0, 0, 0, 0, 3)))

    def test_velocity(self):
        from_gnss = mavextra.gps_velocity_old(gnss())
        from_raw = mavextra.gps_velocity_old(gps_raw_int())
        self.assertAlmostEqual(from_gnss.x, from_raw.x)
        self.assertAlmostEqual(from_gnss.y, from_raw.y)
        self.assertAlmostEqual(from_gnss.y, 12.5)


@unittest.skipUnless(HAVE_GNSS, "message definitions do not include GNSS")
class TestGPSMessageFilter(unittest.TestCase):
    def test_gps_raw_int_used_until_gnss(self):
        gps_filter = mavextra.GPSMessageFilter()
        self.assertTrue(gps_filter.use(from_system(gps_raw_int())))
        self.assertTrue(gps_filter.use(from_system(gnss())))
        self.assertFalse(gps_filter.use(from_system(gps_raw_int())))
        self.assertTrue(gps_filter.use(from_system(gnss(instance=1))))

    def test_per_system(self):
        gps_filter = mavextra.GPSMessageFilter()
        gps_filter.use(from_system(gnss(), sysid=1))
        self.assertTrue(gps_filter.use(from_system(gps_raw_int(), sysid=2)))

    def test_other_messages_pass(self):
        gps_filter = mavextra.GPSMessageFilter()
        gps_filter.use(from_system(gnss()))
        self.assertTrue(gps_filter.use(from_system(mavlink2.MAVLink_heartbeat_message(0, 0, 0, 0, 0, 3))))


if __name__ == '__main__':
    unittest.main()
