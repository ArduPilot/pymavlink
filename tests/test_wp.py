#!/usr/bin/env python3

"""
regression tests for mavwp.py
"""

import contextlib
import unittest
from unittest import mock
import os
import tempfile
from importlib.resources import files as importlib_files
import sys

os.environ["MAVLINK20"] = "1"

from pymavlink import mavwp
from pymavlink import mavutil

class MAVWPTest(unittest.TestCase):

    """
    Class to test mavwp
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(MAVWPTest, self).__init__(*args, **kwargs)

    def makewp(self, seq):
        return mavutil.mavlink.MAVLink_mission_item_int_message(
            1, # wp.target_system,
            1, # wp.target_component,
            0, # wp.seq,
            0, # wp.frame,
            0, # wp.command,
            0, # wp.current,
            1, # wp.autocontinue,
            0, # wp.param1,
            0, # wp.param2,
            0, # wp.param3,
            0, # wp.param4,
            0, # int(wp.x*1.0e7),
            0, # int(wp.y*1.0e7),
            seq, # wp.z
        )

    def make_wps(self):
        # create waypoints
        count = 4
        waypoints = []
        for i in range(count):
            waypoints.append(self.makewp(i))

        return waypoints

    def test_add_remove(self):
        """Test we can add/remove waypoints to/from mavwp"""
        loader = mavwp.MAVWPLoader()
        self.assertEqual(loader.mav_mission_type(),
                         mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        waypoints = self.make_wps()

        # make sure basic addition works
        for i in range(len(waypoints)):
            self.assertEqual(loader.count(), i)
            loader.add(waypoints[i])
            self.assertEqual(loader.count(), i+1)

        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(2).seq, 2)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 1)
        self.assertEqual(loader.wp(2).z, 2)
        # short test for the new item() method:
        self.assertEqual(loader.item(0).z, 0)
        self.assertEqual(loader.item(1).z, 1)
        self.assertEqual(loader.item(2).z, 2)


        # remove a middle one, make sure things get renumbered
        loader.remove(waypoints[0])
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(2).seq, 2)
        self.assertEqual(loader.wp(0).z, 1)
        self.assertEqual(loader.wp(1).z, 2)
        self.assertEqual(loader.wp(2).z, 3)

        loader.clear()
        self.assertEqual(loader.count(), 0)

    def test_insert(self):
        """Test we can insert waypoints into mavwp"""
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()

        loader.add(waypoints[0])
        loader.add(waypoints[2])
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 2)
        loader.insert(1, waypoints[3])
        self.assertEqual(loader.count(), 3)
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(2).seq, 2)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 3)
        self.assertEqual(loader.wp(2).z, 2)

    def test_set(self):
        """Test we set waypoints in mavwp"""
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()

        loader.add(waypoints[0])
        loader.add(waypoints[2])
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 2)
        loader.set(waypoints[3], 1)
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 3)

        # setting at the end of the list extends the list:
        loader.set(waypoints[1], 2)

    def test_add_latlonalt(self):
        '''test add_latlonalt method'''
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()

        loader.add(waypoints[0])
        loader.add(waypoints[2])

        loader.add_latlonalt(5, 6, 7, terrain_alt=True)
        loader.add_latlonalt(10, 11, 12, terrain_alt=False)

        wp1 = loader.wp(2)
        self.assertEqual(wp1.x, 5)
        self.assertEqual(wp1.y, 6)
        self.assertEqual(wp1.z, 7)
        self.assertEqual(wp1.frame, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)

        wp1 = loader.wp(3)
        self.assertEqual(wp1.x, 10)
        self.assertEqual(wp1.y, 11)
        self.assertEqual(wp1.z, 12)
        self.assertEqual(wp1.frame, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_save(self):
        '''test load/save in default format'''
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()
        for wp in waypoints:
            loader.add(wp)

        outfile = "test_wp-wp.txt"
        loader.save(outfile)

        loader2 = mavwp.MAVWPLoader()
        loader2.load(outfile)

        outfile2 = "test_wp-wp2.txt"
        loader.save(outfile2)

        self.assertEqual(loader.count(), loader2.count())

        for i in range(loader.count()):
            self.assertEqual(loader.wp(i).z, loader2.wp(i).z)

    def test_wp_is_loiter(self):
        '''test is_loiter method'''
        loader = mavwp.MAVWPLoader()

        wp = self.makewp(1)
        wp.command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
        loader.add(wp)

        wp = self.makewp(2)
        wp.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        loader.add(wp)

        wp = self.makewp(2)
        wp.command = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        loader.add(wp)

        self.assertTrue(loader.wp_is_loiter(0))
        self.assertFalse(loader.wp_is_loiter(1))
        self.assertFalse(loader.wp_is_loiter(2))

        assert True

    def test_is_location_command(self):
        loader = mavwp.MAVWPLoader()
        self.assertFalse(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH))
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT))
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS))

    def make_mission(self, items):
        '''returns a MAVWPLoader holding a mission item for each of
        items, a list of (command, lat, lon) tuples'''
        loader = mavwp.MAVWPLoader()
        for (seq, (command, lat, lon)) in enumerate(items):
            loader.add(mavutil.mavlink.MAVLink_mission_item_int_message(
                1, 1, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command,
                0, 1, 0, 0, 0, 0,
                int(lat * 1.0e7), int(lon * 1.0e7), 100,
            ))
        return loader

    def location_go_around(self):
        '''a context in which MAV_CMD_DO_GO_AROUND is a location command,
        as it is with message definitions which mark it hasLocation'''
        entry = mavutil.mavlink.enums['MAV_CMD'][mavutil.mavlink.MAV_CMD_DO_GO_AROUND]
        return mock.patch.object(entry, 'has_location', True)

    def test_go_around_starts_path(self):
        '''a DO_GO_AROUND with a position is the start of the abort
        sequence's path, and is never flown to'''
        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
        GO_AROUND = mavutil.mavlink.MAV_CMD_DO_GO_AROUND
        with self.location_go_around():
            loader = self.make_mission([
                (WP, -35.0, 149.0),         # 0: home
                (WP, -35.1, 149.0),         # 1
                (LAND, -35.2, 149.0),       # 2
                (GO_AROUND, -35.15, 149.0), # 3: abort sequence starts here
                (WP, -35.15, 149.1),        # 4
                (WP, -35.05, 149.1),        # 5
            ])
            self.assertFalse(loader.is_location_wp(loader.wp(3)))
            self.assertEqual(loader.view_list(), [[0, 1, 2], [3, 4, 5]])
            polygons = loader.polygon_list()
            go_around = loader.wp(3)
            self.assertEqual(polygons[1][0], (go_around.x, go_around.y, (255, 0, 127)))

    def test_go_around_mid_path(self):
        '''the path does not fly to a DO_GO_AROUND between waypoints'''
        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        GO_AROUND = mavutil.mavlink.MAV_CMD_DO_GO_AROUND
        with self.location_go_around():
            loader = self.make_mission([
                (WP, -35.0, 149.0),         # 0
                (WP, -35.1, 149.0),         # 1
                (GO_AROUND, -35.15, 149.0), # 2
                (WP, -35.2, 149.0),         # 3
            ])
            self.assertEqual(loader.view_list(), [[0, 1, 3], [2, 3]])

    @contextlib.contextmanager
    def command_markup(self, cmd, **attributes):
        '''a context in which the MAV_CMD entry for cmd has the given
        attributes; an attribute given as None is removed, as in
        Python generated by a pymavlink which did not know it'''
        entry = mavutil.mavlink.enums['MAV_CMD'][cmd]
        missing = object()
        saved = {name: vars(entry).get(name, missing) for name in attributes}
        try:
            for name, value in attributes.items():
                if value is None:
                    vars(entry).pop(name, None)
                else:
                    setattr(entry, name, value)
            yield
        finally:
            for name, value in saved.items():
                if value is missing:
                    vars(entry).pop(name, None)
                else:
                    setattr(entry, name, value)

    def roi_mission(self):
        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        ROI = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        return self.make_mission([
            (WP, -35.0, 149.0),     # 0
            (WP, -35.1, 149.0),     # 1
            (ROI, -35.15, 149.1),   # 2
            (WP, -35.2, 149.0),     # 3
        ])

    def test_is_destination_command(self):
        loader = mavwp.MAVWPLoader()
        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        ROI = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        RTL = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        with self.command_markup(WP, has_location=True, is_destination=True), \
             self.command_markup(ROI, has_location=True, is_destination=False), \
             self.command_markup(RTL, has_location=False, is_destination=False):
            self.assertTrue(loader.is_destination_command(WP))
            self.assertFalse(loader.is_destination_command(ROI))
            self.assertFalse(loader.is_destination_command(RTL))

    def test_destination_without_location(self):
        '''older definitions mark some commands isDestination without
        hasLocation; with no location they are not destinations'''
        loader = mavwp.MAVWPLoader()
        CHANGE_ALT = mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
        with self.command_markup(CHANGE_ALT, has_location=False, is_destination=True):
            self.assertFalse(loader.is_destination_command(CHANGE_ALT))

    def test_location_not_destination_not_flown_to(self):
        '''the path does not go to a location which is not a destination'''
        ROI = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        with self.command_markup(ROI, has_location=True, is_destination=False):
            loader = self.roi_mission()
            self.assertFalse(loader.is_location_wp(loader.wp(2)))
            # the ROI is a point of its own, so it still gets a marker:
            self.assertEqual(loader.view_list(), [[0, 1, 3], [2]])

    def test_location_markers(self):
        '''locations which are not destinations are each drawn as a
        point of their own, off the path'''
        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
        ROI = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        HOME = mavutil.mavlink.MAV_CMD_DO_SET_HOME
        with self.command_markup(ROI, has_location=True, is_destination=False), \
             self.command_markup(HOME, has_location=True, is_destination=False):
            loader = self.make_mission([
                (WP, -35.0, 149.0),     # 0
                (WP, -35.1, 149.0),     # 1
                (ROI, -35.15, 149.1),   # 2
                (HOME, -35.05, 149.05), # 3
                (WP, -35.2, 149.0),     # 4
                (LAND, -35.25, 149.0),  # 5
                (ROI, -35.3, 149.1),    # 6: after the landing
                (ROI, 0, 0),            # 7: no position, not drawn
            ])
            self.assertEqual(loader.view_list(), [[0, 1, 4, 5], [2], [3], [6]])
            polygons = loader.polygon_list()
            roi = loader.wp(2)
            self.assertEqual(polygons[1], [(roi.x, roi.y)])

    def test_is_destination_unknown(self):
        '''with Python generated before is_destination, the path is
        drawn as it was before is_destination was used'''
        ROI = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        with self.command_markup(ROI, has_location=True, is_destination=None):
            loader = self.roi_mission()
            self.assertEqual(loader.view_list(), [[0, 1, 2, 3]])
        for cmd in [mavutil.mavlink.MAV_CMD_DO_LAND_START,
                    mavutil.mavlink.MAV_CMD_DO_GO_AROUND]:
            with self.command_markup(cmd, has_location=True, is_destination=None):
                self.assertTrue(mavwp.MAVWPLoader().is_location_command(cmd))
                self.assertFalse(mavwp.MAVWPLoader().is_destination_command(cmd))

    def test_zero_position_start_item(self):
        '''a start item without a position is not drawn from 0,0'''
        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
        for start in [mavutil.mavlink.MAV_CMD_DO_LAND_START,
                      mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START,
                      mavutil.mavlink.MAV_CMD_DO_GO_AROUND]:
            with self.location_go_around():
                loader = self.make_mission([
                    (WP, -35.0, 149.0),     # 0
                    (LAND, -35.1, 149.0),   # 1
                    (start, 0, 0),          # 2
                    (WP, -35.2, 149.0),     # 3
                    (LAND, -35.1, 149.0),   # 4
                ])
                self.assertEqual(loader.view_list(), [[0, 1], [3, 4]])

class RallyTest(unittest.TestCase):
    '''tests functions related to loading waypoints and transferring them
    via the mission-item-protocol'''
    def test_rally_load(self):
        '''test loading rally points from old RALLY style file'''
        loader = mavwp.MissionItemProtocol_Rally()
        self.assertEqual(loader.mav_mission_type(),
                         mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT))

        # test loading a QGC WPL 110 file:
        rally_filepath = importlib_files(__spec__.parent).joinpath("rally-110.txt")
        loader.load(rally_filepath)
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(0).command, mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT)

        # test loading tradition "RALLY" style format:
        rally_filepath = importlib_files(__spec__.parent).joinpath("rally.txt")
        loader.load(rally_filepath)
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(0).command, mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT)

    def test_rally_load_altitude(self):
        '''test the altitude from an old RALLY style file is kept in metres'''
        loader = mavwp.MissionItemProtocol_Rally()
        with tempfile.TemporaryDirectory() as tmpdir:
            rally_filepath = os.path.join(tmpdir, "rally.txt")
            with open(rally_filepath, "w") as f:
                f.write("RALLY 40.071553\t-105.229401\t100.000000\t40.000000\t0.000000\t0\n")
            loader.load(rally_filepath)
        self.assertEqual(loader.count(), 1)
        self.assertEqual(loader.wp(0).z, 100)

class LegacyRallyTest(unittest.TestCase):
    def test_save_preserves_landing_heading(self):
        """Legacy rally files store headings in degrees, not centidegrees."""
        with tempfile.TemporaryDirectory() as tmpdir:
            filename = os.path.join(tmpdir, "rally.txt")
            for heading in (0, 29, 113, 125, 9000, 18000, 35999):
                with self.subTest(heading=heading):
                    loader = mavwp.MAVRallyLoader()
                    loader.create_and_append_rally_point(
                        400715530, -1052294010, 100, 40, heading, 0)
                    loader.save(filename)
                    with open(filename) as f:
                        self.assertEqual(float(f.read().split()[5]), heading / 100.0)
                    restored = mavwp.MAVRallyLoader()
                    restored.load(filename)
                    before = loader.rally_point(0)
                    after = restored.rally_point(0)
                    for field in ("lat", "lng", "alt", "break_alt", "land_dir", "flags"):
                        self.assertEqual(getattr(after, field), getattr(before, field))
                    # A loaded point must still fit its MAVLink wire representation.
                    after.pack(mavutil.mavlink.MAVLink(None))


class FenceTest(unittest.TestCase):
    def test_fence_load(self):
        '''test loading rally points from old RALLY style file'''
        loader = mavwp.MissionItemProtocol_Fence()
        self.assertEqual(loader.mav_mission_type(),
                         mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION))

        # test loading a QGC WPL 110 file:
        fence_filepath = importlib_files(__spec__.parent).joinpath("fence-110.txt")
        loader.load(fence_filepath)
        self.assertEqual(loader.count(), 10)
        self.assertEqual(loader.wp(3).command, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

        # test loading tradition lat/lng-pair style format:
        fence_filepath = importlib_files(__spec__.parent).joinpath("fence.txt")
        loader.load(fence_filepath)
        # there are 6 lines in the file - one return point, four fence
        # points and a "fence closing point".  We don't store the
        # fence closing point.
        self.assertEqual(loader.count(), 5)
        self.assertEqual(loader.wp(0).command, mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT)
        self.assertEqual(loader.wp(3).command, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

if __name__ == '__main__':
    unittest.main()
