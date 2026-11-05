#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

'''
tests for the optional timeouts on mavutil's wait helpers
'''

import socket
import time
import unittest

from pymavlink import mavutil


def free_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class TestWaitTimeouts(unittest.TestCase):
    def setUp(self):
        port = free_udp_port()
        self.gcs = mavutil.mavlink_connection("udpin:127.0.0.1:%u" % port)
        self.vehicle = mavutil.mavlink_connection("udpout:127.0.0.1:%u" % port,
                                                  source_system=1)

    def tearDown(self):
        self.vehicle.close()
        self.gcs.close()

    def send_heartbeat(self, armed):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        self.vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                        base_mode, 0, 0)

    def assert_returns_within(self, seconds, func, *args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        self.assertLess(time.time() - start, seconds)
        return result

    def test_armed_wait_times_out(self):
        self.send_heartbeat(armed=False)
        result = self.assert_returns_within(2, self.gcs.motors_armed_wait, timeout=0.5)
        self.assertFalse(result)

    def test_armed_wait_returns_when_armed(self):
        self.send_heartbeat(armed=True)
        result = self.assert_returns_within(2, self.gcs.motors_armed_wait, timeout=5)
        self.assertTrue(result)

    def test_disarmed_wait_times_out(self):
        self.send_heartbeat(armed=True)
        result = self.assert_returns_within(2, self.gcs.motors_disarmed_wait, timeout=0.5)
        self.assertFalse(result)

    def test_disarmed_wait_returns_when_disarmed(self):
        self.send_heartbeat(armed=False)
        result = self.assert_returns_within(2, self.gcs.motors_disarmed_wait, timeout=5)
        self.assertTrue(result)

    def test_armed_wait_times_out_with_no_link(self):
        result = self.assert_returns_within(2, self.gcs.motors_armed_wait, timeout=0.5)
        self.assertFalse(result)

    def test_gps_fix_times_out(self):
        result = self.assert_returns_within(2, self.gcs.wait_gps_fix, timeout=0.5)
        self.assertIsNone(result)

    def test_gps_fix_returns_fix(self):
        self.vehicle.mav.vfr_hud_send(0, 0, 0, 0, 10, 0)
        self.vehicle.mav.gps_raw_int_send(0, 3, 515000000, -10000, 10000,
                                          100, 100, 0, 0, 10)
        result = self.assert_returns_within(2, self.gcs.wait_gps_fix, timeout=5)
        self.assertIsNotNone(result)
        self.assertEqual(result.fix_type, 3)


if __name__ == '__main__':
    unittest.main()
