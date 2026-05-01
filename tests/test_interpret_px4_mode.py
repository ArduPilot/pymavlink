#!/usr/bin/env python3


"""Unit tests for mavutil.interpret_px4_mode -- regression coverage for #793."""

import unittest

from pymavlink import mavutil


class InterpretPX4ModeTest(unittest.TestCase):

    def test_offboard_decoded_correctly(self):
        # #793: pre-fix this returned "UNKNOWN" because PX4 v1.12+ stopped
        # setting MAV_MODE_FLAG_AUTO_ENABLED in base_mode for OFFBOARD.
        custom_mode = mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
        assert mavutil.interpret_px4_mode(0, custom_mode) == "OFFBOARD"

    def test_base_mode_ignored(self):
        # Locks in the post-fix contract that base_mode is no longer consulted.
        custom_mode = mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
        assert mavutil.interpret_px4_mode(0x00, custom_mode) == "OFFBOARD"
        assert mavutil.interpret_px4_mode(0xFF, custom_mode) == "OFFBOARD"


if __name__ == '__main__':
    unittest.main()
