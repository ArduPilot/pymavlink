#!/usr/bin/env python

"""
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

"""
    Pymavlink example usage with ArduPilot Copter SITL.
    This expect that the SITL is launch with default parameters.
    You can launch SITL from ArduPilot directory with :
    sim_vehicle.py -v ArduCopter -w --console --map
    Then from pymavlink/examples directory, launch this script :
    python mavexample.py

    The script will example :
    - how to connect to the drone
    - Wait for the drone to be ready
    - Change some incomming message rate
    - Get and change parameters
    - Create and upload an auto mission
    - Get back the mission in the drone
    - Run and monitor an auto mission
    - Make a takeoff in Guided mode
    - Wait some target altitude
    - Send some Position target in Guided mode
    - Monitor the drone position
    - Trigger a RTL and monitor the progress

    Those are heavily based on the work done on ArduPilot Autotest framework : https://ardupilot.org/dev/docs/the-ardupilot-autotest-framework.html
"""

import copy
import math
import os
import sys
import time

from pymavlink.dialects.v20 import ardupilotmega
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil
from pymavlink import mavextra
from pymavlink import mavparm
from pymavlink.rotmat import Vector3
from pymavlink import mavwp
from pymavlink.mavutil import location
import datetime

__license__ = "GPLv3}"

class ErrorException(Exception):
    """Base class for other exceptions"""
    pass


class TimeoutException(ErrorException):
    pass


class WaitModeTimeout(TimeoutException):
    """Thrown when fails to achieve given mode change."""
    pass


class WaitAltitudeTimout(TimeoutException):
    """Thrown when fails to achieve given altitude range."""
    pass


class WaitGroundSpeedTimeout(TimeoutException):
    """Thrown when fails to achieve given ground speed range."""
    pass


class WaitRollTimeout(TimeoutException):
    """Thrown when fails to achieve given roll in degrees."""
    pass


class WaitPitchTimeout(TimeoutException):
    """Thrown when fails to achieve given pitch in degrees."""
    pass


class WaitHeadingTimeout(TimeoutException):
    """Thrown when fails to achieve given heading."""
    pass


class WaitDistanceTimeout(TimeoutException):
    """Thrown when fails to attain distance"""
    pass


class WaitLocationTimeout(TimeoutException):
    """Thrown when fails to attain location"""
    pass


class WaitWaypointTimeout(TimeoutException):
    """Thrown when fails to attain waypoint ranges"""
    pass


class SetRCTimeout(TimeoutException):
    """Thrown when fails to send RC commands"""
    pass


class MsgRcvTimeoutException(TimeoutException):
    """Thrown when fails to receive an expected message"""
    pass


class NotAchievedException(ErrorException):
    """Thrown when fails to achieve a goal"""
    pass


class YawSpeedNotAchievedException(NotAchievedException):
    """Thrown when fails to achieve given yaw speed."""
    pass


class SpeedVectorNotAchievedException(NotAchievedException):
    """Thrown when fails to achieve given speed vector."""
    pass


class PreconditionFailedException(ErrorException):
    """Thrown when a precondition for a test is not met"""
    pass


class ArmedAtEndOfTestException(ErrorException):
    """Created when test left vehicle armed"""
    pass


class Copter:
    """ArduPilot Copter class.

    This class is a generic class that show some example on how to use Pymavlink to connect and control ArduPilot Copter drone.
    This is heavily based on ArduPilot Autotest framework : https://github.com/ArduPilot/ardupilot/tree/master/Tools/autotest. You can find there more utilities functions."""

    def __init__(self, default_stream_rate=5, sysid=1):
        self.wp_received = {}
        self.mav = None
        self.streamrate = default_stream_rate
        self.target_system = sysid
        self.target_component = 1
        self.heartbeat_interval_ms = 1000
        self.last_heartbeat_time_ms = None
        self.last_heartbeat_time_wc_s = 0
        self.in_drain_mav = False
        self.total_waiting_to_arm_time = 0
        self.waiting_to_arm_count = 0
        self.wploader = mavwp.MAVWPLoader()
        self.wp_requested = {}
        self.wp_expected_count = 0

    @staticmethod
    def progress(text):
        """Utility to print message with current time."""
        now = datetime.datetime.now()
        formatted_text = "AT %s: %s" % (now.strftime('%H:%M:%S'), text)
        print(formatted_text)

    @staticmethod
    def longitude_scale(lat):
        ret = math.cos(lat * (math.radians(1)))
        print("scale=%f" % ret)
        return ret

    @staticmethod
    def get_distance(loc1, loc2):
        """Get ground distance between two locations."""
        return Copter.get_distance_accurate(loc1, loc2)
        # dlat = loc2.lat - loc1.lat
        # try:
        #     dlong = loc2.lng - loc1.lng
        # except AttributeError:
        #     dlong = loc2.lon - loc1.lon

        # return math.sqrt((dlat*dlat) + (dlong*dlong)*AutoTest.longitude_scale(loc2.lat)) * 1.113195e5

    @staticmethod
    def get_distance_accurate(loc1, loc2):
        """Get ground distance between two locations."""
        try:
            lon1 = loc1.lng
            lon2 = loc2.lng
        except AttributeError:
            lon1 = loc1.lon
            lon2 = loc2.lon

        return mp_util.gps_distance(loc1.lat, lon1, loc2.lat, lon2)

    @staticmethod
    def get_latlon_attr(loc, attrs):
        """return any found latitude attribute from loc"""
        ret = None
        for attr in attrs:
            if hasattr(loc, attr):
                ret = getattr(loc, attr)
                break
        if ret is None:
            raise ValueError("None of %s in loc(%s)" % (str(attrs), str(loc)))
        return ret

    @staticmethod
    def get_lat_attr(loc):
        """return any found latitude attribute from loc"""
        return Copter.get_latlon_attr(loc, ["lat", "latitude"])

    @staticmethod
    def get_lon_attr(loc):
        """return any found latitude attribute from loc"""
        return Copter.get_latlon_attr(loc, ["lng", "lon", "longitude"])

    @staticmethod
    def get_distance_int(loc1, loc2):
        """Get ground distance between two locations in the normal "int" form
        - lat/lon multiplied by 1e7"""
        loc1_lat = Copter.get_lat_attr(loc1)
        loc2_lat = Copter.get_lat_attr(loc2)
        loc1_lon = Copter.get_lon_attr(loc1)
        loc2_lon = Copter.get_lon_attr(loc2)

        return Copter.get_distance_accurate(
            mavutil.location(loc1_lat * 1e-7, loc1_lon * 1e-7),
            mavutil.location(loc2_lat * 1e-7, loc2_lon * 1e-7))

    def connect(self, connection_string='udpin:0.0.0.0:14550'):
        """Set the connection with the drone.
         Use ArduPilot dialect and enforce MAVLink2 usage.
         Set some default streamrate.
         Add some default utility default hook that serve when receiving messages."""
        os.environ['MAVLINK20'] = '1'
        self.mav = mavutil.mavlink_connection(
            connection_string,
            retries=1000,
            robust_parsing=True,
            source_system=250,
            source_component=250,
            autoreconnect=True,
            dialect="ardupilotmega",
        )
        self.set_streamrate(self.streamrate)

        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)

    def set_streamrate(self, streamrate, timeout=20):
        """set MAV_DATA_STREAM_ALL; timeout is wallclock time"""
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise TimeoutException("Failed to set streamrate")
            self.mav.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                streamrate,
                1)
            m = self.mav.recv_match(type='SYSTEM_TIME',
                                    blocking=True,
                                    timeout=1)
            if m is not None:
                break

    def rate_to_interval_us(self, rate):
        return 1 / float(rate) * 1000000.0

    def set_message_rate_hz(self, id, rate_hz):
        """set a message rate in Hz; 0 for original, -1 to disable"""
        if type(id) == str:
            id = eval("mavutil.mavlink.MAVLINK_MSG_ID_%s" % id)
        if rate_hz == 0 or rate_hz == -1:
            set_interval = rate_hz
        else:
            set_interval = self.rate_to_interval_us(rate_hz)
        self.run_cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     id,
                     set_interval,
                     0,
                     0,
                     0,
                     0,
                     0)

    def send_get_message_interval(self, victim_message_id):
        self.mav.mav.command_long_send(
            1,
            1,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            1,  # confirmation
            float(victim_message_id),
            0,
            0,
            0,
            0,
            0,
            0)
        m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
        return self.rate_to_interval_us(m.interval_us)

    def send_set_parameter_direct(self, name, value):
        self.mav.mav.param_set_send(self.target_system,
                                    1,
                                    name.encode('ascii'),
                                    value,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # def send_set_parameter_mavproxy(self, name, value):
    #     self.mavproxy.send("param set %s %s\n" % (name, str(value)))

    def send_set_parameter(self, name, value, verbose=False):
        if verbose:
            self.progress("Send set param for (%s) (%f)" % (name, value))
        # if False:
        #     return self.send_set_parameter_mavproxy(name, value)
        return self.send_set_parameter_direct(name, value)

    def set_parameter(self, name, value, **kwargs):
        self.set_parameters({name: value}, **kwargs)

    def set_parameters(self, parameters, add_to_context=True, epsilon_pct=0.00001, retries=None, verbose=True):
        """Set parameters from vehicle."""
        want = copy.copy(parameters)
        self.progress("set_parameters: (%s)" % str(want))
        self.drain_mav()
        if len(want) == 0:
            return

        if retries is None:
            # we can easily fill ArduPilot's param-set/param-get queue
            # which is quite short.  So we retry *a lot*.
            retries = (len(want) + 1) * 5

        param_value_messages = []

        def add_param_value(mav, m):
            t = m.get_type()
            if t != "PARAM_VALUE":
                return
            param_value_messages.append(m)

        self.install_message_hook(add_param_value)

        original_values = {}
        autopilot_values = {}
        tstart = time.time()
        for i in range(retries):
            self.drain_mav(quiet=True)
            received = set()
            for (name, value) in want.items():
                self.progress("%s want=%f autopilot=%s" % (name, value, autopilot_values.get(name, 'None')))
                if name not in autopilot_values:
                    self.send_get_parameter_direct(name)
                    self.progress("Requesting (%s) (retry=%u)" % (name, i))
                    continue
                delta = abs(autopilot_values[name] - value)
                if delta <= epsilon_pct * 0.01 * abs(value):
                    # correct value
                    self.progress("%s is now %f" % (name, autopilot_values[name]))
                    # if add_to_context:
                    #     context_param_name_list = [p[0] for p in self.context_get().parameters]
                    #     if name.upper() not in context_param_name_list:
                    #         self.context_get().parameters.append((name, original_values[name]))
                    received.add(name)
                    continue
                self.progress("Sending set (%s) to (%f) (old=%f)" % (name, value, original_values[name]))
                self.send_set_parameter_direct(name, value)
            for name in received:
                del want[name]
            if len(want):
                self.wait_heartbeat()
            #     # problem here is that a reboot can happen after we
            #     # send the request but before we receive the reply:
            #     try:
            #         self.do_timesync_roundtrip(quiet=True)
            #     except AutoTestTimeoutException:
            #         pass
            #         # now = self.get_sim_time_cached()
            #         # if tstart > now:
            #         #     self.progress("Time wrap detected")
            #         # else:
            #         #     raise
            do_fetch_all = False
            for m in param_value_messages:
                if m.param_id in want:
                    self.progress("Received wanted PARAM_VALUE %s=%f" %
                                  (str(m.param_id), m.param_value))
                    autopilot_values[m.param_id] = m.param_value
                    if m.param_id not in original_values:
                        original_values[m.param_id] = m.param_value;
                        if (self.should_fetch_all_for_parameter_change(m.param_id.upper()) and
                                m.param_value != 0):
                            do_fetch_all = True
            param_value_messages = []
        #            if do_fetch_all:
        #                self.do_fetch_all()

        self.remove_message_hook(add_param_value)

        if len(want) == 0:
            return
        raise ValueError("Failed to set parameters (%s)" % want)

    @staticmethod
    def should_fetch_all_for_parameter_change(param_name):
        return False  # FIXME: if we allow MAVProxy then allow this
        if fnmatch.fnmatch(param_name, "*_ENABLE") or fnmatch.fnmatch(param_name, "*_ENABLED"):
            return True
        if param_name in ["ARSPD_TYPE",
                          "ARSPD2_TYPE",
                          "BATT2_MONITOR",
                          "CAN_DRIVER",
                          "COMPASS_PMOT_EN",
                          "OSD_TYPE",
                          "RSSI_TYPE",
                          "WENC_TYPE"]:
            return True
        return False

    def get_parameter(self, *args, **kwargs):
        return self.get_parameter_direct(*args, **kwargs)

    def send_get_parameter_direct(self, name):
        encname = name
        if sys.version_info.major >= 3 and type(encname) != bytes:
            encname = bytes(encname, 'ascii')
        self.mav.mav.param_request_read_send(self.target_system,
                                             1,
                                             encname,
                                             -1)

    def get_parameter_direct(self, name, attempts=1, timeout=60, verbose=True, timeout_in_wallclock=False):
        while attempts > 0:
            attempts -= 1
            if verbose:
                self.progress("Sending param_request_read for (%s)" % name)
            # we MUST parse here or collections fail where we need
            # them to work!
            self.drain_mav(quiet=True)
            if timeout_in_wallclock:
                tstart = time.time()
            else:
                tstart = time.time()
            self.send_get_parameter_direct(name)
            while True:
                if timeout_in_wallclock:
                    now = time.time()
                else:
                    now = time.time()
                    if tstart > now:
                        self.progress("Time wrap detected")
                        # we're going to have to send another request...
                        break
                delta_time = now - tstart
                if delta_time > timeout:
                    break
                m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                if verbose:
                    self.progress("get_parameter(%s): %s" % (name, str(m),))
                if m is None:
                    continue
                if m.param_id == name:
                    if delta_time > 5:
                        self.progress("Long time to get parameter: %fs" % (delta_time,))
                    return m.param_value
                if verbose:
                    self.progress("(%s) != (%s)" % (m.param_id, name,))
        raise NotAchievedException("Failed to retrieve parameter (%s)" % name)

    def idle_hook(self, mav):
        """Called when waiting for a mavlink message."""
        if self.in_drain_mav:
            return

    def message_hook(self, mav, msg):
        """Called as each mavlink msg is received.
        Display STATUSTEXT messages.
        Send the heartbeats if needed."""
        if msg.get_type() == 'STATUSTEXT':
            self.progress("AP: %s" % msg.text)
        self.idle_hook(mav)
        self.do_heartbeats()

    def install_message_hook(self, hook):
        self.mav.message_hooks.append(hook)

    def remove_message_hook(self, hook):
        if self.mav is None:
            return
        oldlen = len(self.mav.message_hooks)
        self.mav.message_hooks = list(filter(lambda x: x != hook,
                                             self.mav.message_hooks))
        if len(self.mav.message_hooks) == oldlen:
            raise NotAchievedException("Failed to remove hook")

    def do_heartbeats(self, force=False):
        if self.heartbeat_interval_ms is None and not force:
            return
        x = self.mav.messages.get("SYSTEM_TIME", None)
        now_wc = time.time()
        if (force or
                x is None or
                self.last_heartbeat_time_ms is None or
                self.last_heartbeat_time_ms < x.time_boot_ms or
                x.time_boot_ms - self.last_heartbeat_time_ms > self.heartbeat_interval_ms or
                now_wc - self.last_heartbeat_time_wc_s > 1):
            # self.progress("Sending heartbeat")
            if x is not None:
                self.last_heartbeat_time_ms = x.time_boot_ms
            self.last_heartbeat_time_wc_s = now_wc
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                        0,
                                        0,
                                        0)

    def send_cmd(self,
                 command,
                 p1,
                 p2,
                 p3,
                 p4,
                 p5,
                 p6,
                 p7,
                 target_sysid=None,
                 target_compid=None,
                 ):
        """Send a MAVLink command long."""
        if target_sysid is None:
            target_sysid = self.target_system
        if target_compid is None:
            target_compid = 1
        try:
            command_name = mavutil.mavlink.enums["MAV_CMD"][command].name
        except KeyError as e:
            command_name = "UNKNOWN=%u" % command
        self.progress("Sending COMMAND_LONG to (%u,%u) (%s) (p1=%f p2=%f p3=%f p4=%f p5=%f p6=%f  p7=%f)" %
                      (
                          target_sysid,
                          target_compid,
                          command_name,
                          p1,
                          p2,
                          p3,
                          p4,
                          p5,
                          p6,
                          p7))
        self.mav.mav.command_long_send(target_sysid,
                                       target_compid,
                                       command,
                                       1,  # confirmation
                                       p1,
                                       p2,
                                       p3,
                                       p4,
                                       p5,
                                       p6,
                                       p7)

    def run_cmd(self,
                command,
                p1,
                p2,
                p3,
                p4,
                p5,
                p6,
                p7,
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                target_sysid=None,
                target_compid=None,
                timeout=10,
                quiet=False):
        self.drain_mav_unparsed()
        time.time()  # required for timeout in run_cmd_get_ack to work
        self.send_cmd(command,
                      p1,
                      p2,
                      p3,
                      p4,
                      p5,
                      p6,
                      p7,
                      target_sysid=target_sysid,
                      target_compid=target_compid,
                      )
        self.run_cmd_get_ack(command, want_result, timeout, quiet=quiet)

    def run_cmd_get_ack(self, command, want_result, timeout, quiet=False):
        # note that the caller should ensure that this cached
        # timestamp is reasonably up-to-date!
        tstart = time.time()
        while True:
            delta_time = time.time() - tstart
            if delta_time > timeout:
                raise TimeoutException("Did not get good COMMAND_ACK within %fs" % timeout)
            m = self.mav.recv_match(type='COMMAND_ACK',
                                    blocking=True,
                                    timeout=0.1)
            if m is None:
                continue
            if not quiet:
                self.progress("ACK received: %s (%fs)" % (str(m), delta_time))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError("Expected %s got %s" % (
                        mavutil.mavlink.enums["MAV_RESULT"][want_result].name,
                        mavutil.mavlink.enums["MAV_RESULT"][m.result].name))
                break

    def run_cmd_do_set_mode(self,
                            mode,
                            timeout=30,
                            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     base_mode,
                     custom_mode,
                     0,
                     0,
                     0,
                     0,
                     0,
                     want_result=want_result,
                     timeout=timeout
                     )

    def do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message."""
        tstart = time.time()
        want_custom_mode = self.get_mode_from_mode_mapping(mode)
        while True:
            remaining = timeout - (time.time() - tstart)
            if remaining <= 0:
                raise TimeoutException("Failed to change mode")
            self.run_cmd_do_set_mode(mode, timeout=10)
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=5)
            if m is None:
                raise ErrorException("Heartbeat not received")
            self.progress("Got mode=%u want=%u" % (m.custom_mode, want_custom_mode))
            if m.custom_mode == want_custom_mode:
                return

    def change_mode(self, mode, timeout=60):
        """change vehicle flightmode"""
        self.wait_heartbeat()
        self.progress("Changing mode to %s" % mode)
        self.do_set_mode_via_command_long(mode)

    def mode_is(self, mode, cached=False, drain_mav=True):
        if not cached:
            self.wait_heartbeat(drain_mav=drain_mav)
        try:
            return self.get_mode_from_mode_mapping(self.mav.flightmode) == self.get_mode_from_mode_mapping(mode)
        except Exception as e:
            pass
        # assume this is a number....
        return self.mav.messages['HEARTBEAT'].custom_mode == mode

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        self.progress("Waiting for mode %s" % mode)
        tstart = time.time()
        while not self.mode_is(mode, drain_mav=False):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    time.time() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)

    def get_mode_from_mode_mapping(self, mode):
        """Validate and return the mode number from a string or int."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None:
            mav_type = self.mav.messages['HEARTBEAT'].type
            mav_autopilot = self.mav.messages['HEARTBEAT'].autopilot
            raise ErrorException("No mode map for (mav_type=%s mav_autopilot=%s)" % (mav_type, mav_autopilot))
        if isinstance(mode, str):
            if mode in mode_map:
                return mode_map.get(mode)
        if mode in mode_map.values():
            return mode
        self.progress("Available modes '%s'" % mode_map)
        raise ErrorException("Unknown mode '%s'" % mode)

    def distance_to_home(self, use_cached_home=False):
        m = self.mav.messages.get("HOME_POSITION", None)
        if use_cached_home is False or m is None:
            m = self.poll_home_position(quiet=True)
        here = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True)
        return self.get_distance_int(m, here)

    def poll_home_position(self, quiet=True, timeout=30):
        old = self.mav.messages.get("HOME_POSITION", None)
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise NotAchievedException("Failed to poll home position")
            if not quiet:
                self.progress("Sending MAV_CMD_GET_HOME_POSITION")
            try:
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    quiet=quiet)
            except ValueError as e:
                continue
            m = self.mav.messages.get("HOME_POSITION", None)
            if m is None:
                continue
            if old is None:
                break
            if m._timestamp != old._timestamp:
                break
        self.progress("Polled home position (%s)" % str(m))
        return m

    def home_position_as_mav_location(self):
        m = self.poll_home_position()
        return mavutil.location(m.latitude * 1.0e-7, m.longitude * 1.0e-7, m.altitude * 1.0e-3, 0)

    def wait_altitude(self, altitude_min, altitude_max, relative=False, timeout=30, **kwargs):
        """Wait for a given altitude range."""
        assert altitude_min <= altitude_max, "Minimum altitude should be less than maximum altitude."

        def get_altitude(alt_relative=False, timeout2=30):
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout2)
            if msg:
                if alt_relative:
                    return msg.relative_alt / 1000.0  # mm -> m
                else:
                    return msg.alt / 1000.0  # mm -> m
            raise MsgRcvTimeoutException("Failed to get Global Position")

        def validator(value2, target2=None):
            if altitude_min <= value2 <= altitude_max:
                return True
            else:
                return False

        self.wait_and_maintain(value_name="Altitude", target=altitude_min,
                               current_value_getter=lambda: get_altitude(relative, timeout),
                               accuracy=(altitude_max - altitude_min),
                               validator=lambda value2, target2: validator(value2, target2), timeout=timeout, **kwargs)

    def wait_location(self,
                      loc,
                      accuracy=5.0,
                      timeout=30,
                      target_altitude=None,
                      height_accuracy=-1,
                      **kwargs):
        """Wait for arrival at a location."""

        def get_distance_to_loc():
            return self.get_distance(self.mav.location(), loc)

        def validator(value2, empty=None):
            if value2 <= accuracy:
                if target_altitude is not None:
                    height_delta = math.fabs(self.mav.location().alt - target_altitude)
                    if height_accuracy != -1 and height_delta > height_accuracy:
                        return False
                return True
            else:
                return False

        debug_text = "Distance to Location (%.4f, %.4f) " % (loc.lat, loc.lng)
        if target_altitude is not None:
            debug_text += ",at altitude %.1f height_accuracy=%.1f, d" % (target_altitude, height_accuracy)
        self.wait_and_maintain(value_name=debug_text, target=0, current_value_getter=lambda: get_distance_to_loc(),
                               accuracy=accuracy, validator=lambda value2, target2: validator(value2, None),
                               timeout=timeout, **kwargs)

    def wait_distance_to_home(self, distance_min, distance_max, timeout=10, use_cached_home=True, **kwargs):
        """Wait for distance to home to be within specified bounds."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.distance_to_home(use_cached_home)

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        self.wait_and_maintain(value_name="Distance to home", target=distance_min,
                               current_value_getter=lambda: get_distance(),
                               validator=lambda value2, target2: validator(value2, target2),
                               accuracy=(distance_max - distance_min), timeout=timeout, **kwargs)

    def wait_and_maintain(self, value_name, target, current_value_getter, validator=None, accuracy=2.0, timeout=30,
                          **kwargs):
        tstart = time.time()
        achieving_duration_start = None
        if type(target) is Vector3:
            sum_of_achieved_values = Vector3()
            last_value = Vector3()
        else:
            sum_of_achieved_values = 0.0
            last_value = 0.0
        count_of_achieved_values = 0
        called_function = kwargs.get("called_function", None)
        minimum_duration = kwargs.get("minimum_duration", 0)
        if type(target) is Vector3:
            self.progress("Waiting for %s=(%s) with accuracy %.02f" % (value_name, str(target), accuracy))
        else:
            self.progress("Waiting for %s=%.02f with accuracy %.02f" % (value_name, target, accuracy))
        last_print_time = 0
        while time.time() < tstart + timeout:  # if we failed to received message with the getter the sim time isn't updated
            last_value = current_value_getter()
            if called_function is not None:
                called_function(last_value, target)
            if time.time() - last_print_time > 1:
                if type(target) is Vector3:
                    self.progress("%s=(%s) (want (%s) +- %f)" %
                                  (value_name, str(last_value), str(target), accuracy))
                else:
                    self.progress("%s=%0.2f (want %f +- %f)" %
                                  (value_name, last_value, target, accuracy))
                last_print_time = time.time()
            if validator is not None:
                is_value_valid = validator(last_value, target)
            else:
                is_value_valid = math.fabs(last_value - target) <= accuracy
            if is_value_valid:
                sum_of_achieved_values += last_value
                count_of_achieved_values += 1.0
                if achieving_duration_start is None:
                    achieving_duration_start = time.time()
                if time.time() - achieving_duration_start >= minimum_duration:
                    if type(target) is Vector3:
                        self.progress("Attained %s=%s" % (
                            value_name, str(sum_of_achieved_values * (1.0 / count_of_achieved_values))))
                    else:
                        self.progress(
                            "Attained %s=%f" % (value_name, sum_of_achieved_values / count_of_achieved_values))
                    return True
            else:
                achieving_duration_start = None
                if type(target) is Vector3:
                    sum_of_achieved_values.zero()
                else:
                    sum_of_achieved_values = 0.0
                count_of_achieved_values = 0
        raise TimeoutException("Failed to attain %s want %s, reached %s" % (value_name, str(target), str(
            sum_of_achieved_values * (1.0 / count_of_achieved_values)) if count_of_achieved_values != 0 else str(
            last_value)))

    def wait_for_alt(self, alt_min=30, timeout=30, max_err=5):
        """Wait for minimum altitude to be reached."""
        self.wait_altitude(alt_min - 1,
                           (alt_min + max_err),
                           relative=True,
                           timeout=timeout)

    def drain_mav_unparsed(self, mav=None, quiet=True, freshen_sim_time=False):
        if mav is None:
            mav = self.mav
        self.in_drain_mav = True
        count = 0
        tstart = time.time()
        while True:
            this = self.mav.recv(1000000)
            if len(this) == 0:
                break
            count += len(this)
        if quiet:
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count / float(tdelta),)
        self.progress("Drained %u bytes from mav (%s).  These were unparsed." % (count, rate))
        self.in_drain_mav = False
        if freshen_sim_time:
            time.time()

    def drain_mav(self, mav=None, unparsed=False, quiet=True):
        if unparsed:
            return self.drain_mav_unparsed(quiet=quiet, mav=mav)
        if mav is None:
            mav = self.mav
        count = 0
        tstart = time.time()
        while mav.recv_match(blocking=False) is not None:
            count += 1
        if quiet:
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count / float(tdelta),)

        self.progress("Drained %u messages from mav (%s)" % (count, rate))

    def wait_prearm_sys_status_healthy(self, timeout=60):
        # self.do_timesync_roundtrip()
        tstart = time.time()
        while True:
            t2 = time.time()
            if t2 - tstart > timeout:
                self.progress("Prearm bit never went true.  Attempting arm to elicit reason from autopilot")
                self.arm_vehicle()
                raise TimeoutException("Prearm bit never went true")
            if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK, True, True, True):
                break

    def sensor_has_state(self, sensor, present=True, enabled=True, healthy=True, do_assert=False, verbose=False):
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if m is None:
            raise TimeoutException("Did not receive SYS_STATUS")
        if verbose:
            self.progress("Status: %s" % str(mavutil.dump_message_verbose(sys.stdout, m)))
        reported_present = m.onboard_control_sensors_present & sensor
        reported_enabled = m.onboard_control_sensors_enabled & sensor
        reported_healthy = m.onboard_control_sensors_health & sensor
        if present:
            if not reported_present:
                if do_assert:
                    raise NotAchievedException("Sensor not present")
                return False
        else:
            if reported_present:
                if do_assert:
                    raise NotAchievedException("Sensor present when it shouldn't be")
                return False

        if enabled:
            if not reported_enabled:
                if do_assert:
                    raise NotAchievedException("Sensor not enabled")
                return False
        else:
            if reported_enabled:
                if do_assert:
                    raise NotAchievedException("Sensor enabled when it shouldn't be")
                return False

        if healthy:
            if not reported_healthy:
                if do_assert:
                    raise NotAchievedException("Sensor not healthy")
                return False
        else:
            if reported_healthy:
                if do_assert:
                    raise NotAchievedException("Sensor healthy when it shouldn't be")
                return False
        return True

    def wait_ready_to_arm(self, timeout=120, require_absolute=True, check_prearm_bit=True):
        # wait for EKF checks to pass
        self.progress("Waiting for ready to arm")
        start = time.time()
        self.wait_ekf_happy(timeout=timeout, require_absolute=require_absolute)
        if require_absolute:
            self.wait_gps_sys_status_not_present_or_enabled_and_healthy()
        armable_time = time.time() - start
        if require_absolute:
            m = self.poll_home_position()
            if m is None:
                raise NotAchievedException("Did not receive a home position")
        if check_prearm_bit:
            self.wait_prearm_sys_status_healthy(timeout=timeout)
        self.progress("Took %u seconds to become armable" % armable_time)
        self.total_waiting_to_arm_time += armable_time
        self.waiting_to_arm_count += 1

    def wait_heartbeat(self, drain_mav=True, quiet=False, *args, **x):
        """as opposed to mav.wait_heartbeat, raises an exception on timeout.
Also, ignores heartbeats not from our target system"""
        if drain_mav:
            self.drain_mav(quiet=quiet)
        orig_timeout = x.get("timeout", 10)
        x["timeout"] = 1
        tstart = time.time()
        while True:
            if time.time() - tstart > orig_timeout:
                # if not self.sitl_is_running():
                #     self.progress("SITL is not running")
                raise TimeoutException("Did not receive heartbeat")
            m = self.mav.wait_heartbeat(*args, **x)
            if m is None:
                continue
            if m.get_srcSystem() == self.target_system:
                break

    def wait_ekf_happy(self, timeout=30, require_absolute=True):
        """Wait for EKF to be happy"""

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            return True

        # all of these must be set for arming to happen:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL)
        # none of these bits must be set for arming to happen:
        error_bits = (mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
                      mavutil.mavlink.ESTIMATOR_ACCEL_ERROR)
        if require_absolute:
            required_value |= (mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                               mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                               mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS)
            error_bits |= mavutil.mavlink.ESTIMATOR_GPS_GLITCH
        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_ekf_flags(self, required_value, error_bits, timeout=30):
        self.progress("Waiting for EKF value %u" % required_value)
        self.drain_mav_unparsed()
        last_print_time = 0
        tstart = time.time()
        while timeout is None or time.time() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            errors = current & error_bits
            everything_ok = (errors == 0 and
                             current & required_value == required_value)
            if everything_ok or time.time() - last_print_time > 1:
                self.progress("Wait EKF.flags: required:%u current:%u errors=%u" %
                              (required_value, current, errors))
                last_print_time = time.time()
            if everything_ok:
                self.progress("EKF Flags OK")
                return True
        raise TimeoutException("Failed to get EKF.flags=%u" %
                               required_value)

    def wait_gps_sys_status_not_present_or_enabled_and_healthy(self, timeout=30):
        self.progress("Waiting for GPS health")
        tstart = time.time()
        while True:
            now = time.time()
            if now - tstart > timeout:
                raise TimeoutException("GPS status bits did not become good")
            m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if m is None:
                continue
            if (not (m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not present")
                if now > 20:
                    # it's had long enough to be detected....
                    return
                continue
            if (not (m.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not enabled")
                continue
            if (not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not healthy")
                continue
            self.progress("GPS healthy")
            return

    def wait_waypoint(self,
                      wpnum_start,
                      wpnum_end,
                      allow_skip=True,
                      max_dist=2,
                      timeout=400):
        """Wait for waypoint ranges."""
        tstart = time.time()
        # this message arrives after we set the current WP
        start_wp = self.mav.waypoint_current()
        current_wp = start_wp
        mode = self.mav.flightmode

        self.progress("wait for waypoint ranges start=%u end=%u"
                      % (wpnum_start, wpnum_end))
        # if start_wp != wpnum_start:
        #    raise WaitWaypointTimeout("test: Expected start waypoint %u "
        #                              "but got %u" %
        #                  (wpnum_start, start_wp))

        last_wp_msg = 0
        while time.time() < tstart + timeout:
            seq = self.mav.waypoint_current()
            m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                    blocking=True)
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)

            # if we changed mode, fail
            if self.mav.flightmode != mode:
                raise WaitWaypointTimeout('Exited %s mode' % mode)

            if time.time() - last_wp_msg > 1:
                self.progress("WP %u (wp_dist=%u Alt=%.02f), current_wp: %u,"
                              "wpnum_end: %u" %
                              (seq, wp_dist, m.alt, current_wp, wpnum_end))
                last_wp_msg = time.time()
            if seq == current_wp + 1 or (seq > current_wp + 1 and allow_skip):
                self.progress("test: Starting new waypoint %u" % seq)
                tstart = time.time()
                current_wp = seq
                # the wp_dist check is a hack until we can sort out
                # the right seqnum for end of mission
            # if current_wp == wpnum_end or (current_wp == wpnum_end-1 and
            #                                wp_dist < 2):
            if current_wp == wpnum_end and wp_dist < max_dist:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq >= 255:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq > current_wp + 1:
                raise WaitWaypointTimeout(("Skipped waypoint! Got wp %u expected %u"
                                           % (seq, current_wp + 1)))
        raise WaitWaypointTimeout("Timed out waiting for waypoint %u of %u" %
                                  (wpnum_end, wpnum_end))

    def send_all_waypoints(self, timeout=60):
        """send all waypoints to vehicle"""
        self.mav.waypoint_clear_all_send()
        self.progress("Sending %d waypoints" % self.wploader.count())
        if self.wploader.count() == 0:
            return
        self.mav.waypoint_count_send(self.wploader.count())
        tstart = time.time()
        while True:
            now = time.time()
            if now - tstart > timeout:
                self.progress("Failed to send Mission")
                return
            msg = self.mav.recv_match(type=["MISSION_REQUEST", "WAYPOINT_REQUEST"], blocking=True, timeout=3)
            if msg is None:
                continue
            if msg.seq >= self.wploader.count():
                self.progress("Request for bad waypoint %u (max %u)" % (msg.seq, self.wploader.count()))
                return
            wp = self.wploader.wp(msg.seq)
            wp_send = self.wp_to_mission_item_int(wp)

            self.mav.mav.send(wp_send)
            self.progress("Sent waypoint %u : %s" % (msg.seq, self.wploader.wp(msg.seq)))
            if msg.seq == self.wploader.count() - 1:
                self.progress("Sent all %u waypoints" % self.wploader.count())
                return

    def get_all_waypoints(self, timeout=30):
        self.progress("Requesting Mission item count")
        self.mav.waypoint_request_list_send()
        tstart = time.time()
        while True:
            now = time.time()
            if now - tstart > timeout:
                self.progress("Failed to get Mission total item")
                return
            msg = self.mav.recv_match(type=['WAYPOINT_COUNT', 'MISSION_COUNT'], blocking=True, timeout=3)
            if msg is None:
                continue
            self.wp_expected_count = msg.count
            self.progress("Got %s waypoints to get" % msg.count)
            self.wploader.clear()
            break
        for seq in self.missing_wps_to_request():
            self.wp_requested[seq] = time.time()
            self.progress("Requesting waypoint %d" % seq)
            self.mav.mav.mission_request_int_send(self.target_system, self.target_component, seq)
            tstart = time.time()
            while True:
                now = time.time()
                if now - tstart > timeout:
                    self.progress("Failed to get Waypoint %d" % seq)
                    return
                msg = self.mav.recv_match(type=['WAYPOINT', 'MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True,
                                          timeout=3)
                if msg is None:
                    continue
                if msg.get_type() == 'MISSION_ITEM_INT':
                    if getattr(msg, 'mission_type', 0) != 0:
                        # this is not a mission item, likely fence
                        return
                    # our internal structure assumes MISSION_ITEM'''
                    msg = self.wp_from_mission_item_int(msg)
                if msg.seq < self.wploader.count():
                    # print("DUPLICATE %u" % m.seq)
                    return
                if msg.seq + 1 > self.wp_expected_count:
                    self.progress("Unexpected waypoint number %u - expected %u" % (msg.seq, self.wploader.count()))
                self.wp_received[msg.seq] = msg

                next_seq = self.wploader.count()
                while next_seq in self.wp_received:
                    m = self.wp_received.pop(next_seq)
                    self.wploader.add(m)
                    next_seq += 1
                if self.wploader.count() != self.wp_expected_count:
                    self.progress("m.seq=%u expected_count=%u" % (msg.seq, self.wp_expected_count))
                    break
                if self.wploader.count() == self.wp_expected_count:
                    self.progress("Got all Waypoints")
                    break
        for i in range(self.wploader.count()):
            w = self.wploader.wp(i)
            print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                w.command, w.frame, w.x, w.y, w.z,
                w.param1, w.param2, w.param3, w.param4,
                w.current, w.autocontinue))

        self.wp_requested = {}
        self.wp_received = {}
        return self.wploader.count()

    def missing_wps_to_request(self):
        ret = []
        tnow = time.time()
        next_seq = self.wploader.count()
        for i in range(2 * self.wp_expected_count):
            seq = next_seq + i
            if seq + 1 > self.wp_expected_count:
                continue
            if seq in self.wp_requested and tnow - self.wp_requested[seq] < 2:
                continue
            ret.append(seq)
        return ret

    def wp_to_mission_item_int(self, wp):
        """convert a MISSION_ITEM to a MISSION_ITEM_INT. We always send as MISSION_ITEM_INT
           to give cm level accuracy"""
        if wp.get_type() == 'MISSION_ITEM_INT':
            return wp
        wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(wp.target_system,
                                                                  wp.target_component,
                                                                  wp.seq,
                                                                  wp.frame,
                                                                  wp.command,
                                                                  wp.current,
                                                                  wp.autocontinue,
                                                                  wp.param1,
                                                                  wp.param2,
                                                                  wp.param3,
                                                                  wp.param4,
                                                                  int(wp.x * 1.0e7),
                                                                  int(wp.y * 1.0e7),
                                                                  wp.z)
        return wp_int

    def wp_from_mission_item_int(self, wp):
        '''convert a MISSION_ITEM_INT to a MISSION_ITEM'''
        wp2 = mavutil.mavlink.MAVLink_mission_item_message(wp.target_system,
                                                           wp.target_component,
                                                           wp.seq,
                                                           wp.frame,
                                                           wp.command,
                                                           wp.current,
                                                           wp.autocontinue,
                                                           wp.param1,
                                                           wp.param2,
                                                           wp.param3,
                                                           wp.param4,
                                                           wp.x * 1.0e-7,
                                                           wp.y * 1.0e-7,
                                                           wp.z)
        # preserve srcSystem as that is used for naming waypoint file
        wp2._header.srcSystem = wp.get_srcSystem()
        wp2._header.srcComponent = wp.get_srcComponent()
        return wp2

    def init_wp(self):
        last_home = self.home_position_as_mav_location()
        self.wploader.clear()
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_system
        self.add_waypoint(last_home.lat, last_home.lng, last_home.alt)

    def add_waypoint(self, lat, lon, alt):
        self.wploader.add_latlonalt(lat, lon, alt, terrain_alt=False)

    def add_wp_takeoff(self, lat, lon, alt):
        p = mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                         self.target_component,
                                                         0,
                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                         0, 0, 0, 0, 0, 0,
                                                         lat, lon, alt)
        self.wploader.insert(1, p)

    def add_wp_rtl(self):
        p = mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                         self.target_component,
                                                         0,
                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                                         0, 0, 0, 0, 0, 0,
                                                         0, 0, 0)
        self.wploader.add(p)

    ########################################################################################################################
    # Command functions ####################################################################################################
    ########################################################################################################################
    def user_takeoff(self, alt_min=30):
        """takeoff using mavlink takeoff command"""
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0,  # param1
                     0,  # param2
                     0,  # param3
                     0,  # param4
                     0,  # param5
                     0,  # param6
                     alt_min  # param7
                     )
        self.progress("Ran command")
        self.wait_for_alt(alt_min)

    def land_and_disarm(self, timeout=60):
        """Land the quad."""
        self.progress("STARTING LANDING")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed(timeout=timeout)

    def wait_landed_and_disarmed(self, min_alt=6, timeout=60):
        """Wait to be landed and disarmed"""
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = m.relative_alt / 1000.0  # mm -> m
        if alt > min_alt:
            self.wait_for_alt(min_alt, timeout=timeout)
        #        self.wait_statustext("SIM Hit ground", timeout=timeout)
        self.wait_disarmed()

    # enter RTL mode and wait for the vehicle to disarm
    def do_RTL(self, distance_min=None, check_alt=True, distance_max=10, timeout=250):
        """Enter RTL mode and wait for the vehicle to disarm at Home."""
        self.change_mode("RTL")
        # self.hover()
        self.wait_rtl_complete(check_alt=check_alt, distance_max=distance_max, timeout=timeout)

    def wait_rtl_complete(self, check_alt=True, distance_max=10, timeout=250):
        """Wait for RTL to reach home and disarm"""
        self.progress("Waiting RTL to reach Home and disarm")
        tstart = time.time()
        while time.time() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = m.relative_alt / 1000.0  # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            home = ""
            alt_valid = alt <= 1
            distance_valid = home_distance < distance_max
            if check_alt:
                if alt_valid and distance_valid:
                    home = "HOME"
            else:
                if distance_valid:
                    home = "HOME"
            self.progress("Alt: %.02f  HomeDist: %.02f %s" %
                          (alt, home_distance, home))

            # our post-condition is that we are disarmed:
            if not self.armed():
                if home == "":
                    raise Exception("Did not get home")
                # success!
                return

        raise Exception("Did not get home and disarm")

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()

    def arm_vehicle(self, timeout=20):
        """Arm vehicle with mavlink arm message."""
        self.progress("Arm motors with MAVLink cmd")
        self.drain_mav()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        try:
            self.wait_armed()
        except TimeoutException:
            raise TimeoutException("Failed to ARM with mavlink")
        return True

    def wait_armed(self, timeout=20):
        tstart = time.time()
        while time.time() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("Motors ARMED")
                return
        raise TimeoutException("Did not become armed")

    def disarm_vehicle(self, timeout=60, force=False):
        """Disarm vehicle with mavlink disarm message."""
        self.progress("Disarm motors with MAVLink cmd")
        self.drain_mav_unparsed()
        p2 = 0
        if force:
            p2 = 21196  # magic force disarm value
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     0,  # DISARM
                     p2,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        return self.wait_disarmed()

    def wait_disarmed_default_wait_time(self):
        return 30

    def wait_disarmed(self, timeout=None, tstart=None):
        if timeout is None:
            timeout = self.wait_disarmed_default_wait_time()
        self.progress("Waiting for DISARM")
        if tstart is None:
            tstart = time.time()
        last_print_time = 0
        while True:
            now = time.time()
            delta = now - tstart
            if delta > timeout:
                raise TimeoutException("Failed to DISARM within %fs" %
                                       (timeout,))
            if now - last_print_time > 1:
                self.progress("Waiting for disarm (%.2fs so far of allowed %.2f)" % (delta, timeout))
                last_print_time = now
            self.wait_heartbeat(quiet=True)
            #            self.progress("Got heartbeat")
            if not self.mav.motors_armed():
                self.progress("DISARMED after %.2f seconds (allowed=%.2f)" %
                              (delta, timeout))
                return True

    def get_current_target(self, timeout=10):
        """Get and print POSITION_TARGET_GLOBAL_INT msg send by the drone.
           those message are always in MAV_FRAME_GLOBAL_INT frame."""
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise TimeoutException("Failed to set streamrate")
            msg = self.mav.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=True, timeout=2)
            if msg is not None:
                self.progress("Received local target: %s" % str(msg))
                return location(msg.lat_int * 1.0e-7, msg.lon_int * 1.0e-7, msg.alt, msg.yaw)


##########################

def big_print(text):
    print("##################################################################################")
    print("########## %s  ##########" % text)
    print("##################################################################################")


def main():
    big_print("Welcome in pymavlink Copter example!")
    copter = Copter()

    big_print("Let's connect")
    # Assume that we are connecting to SITL on udp 14550
    copter.connect()

    big_print("Let's wait ready to arm")
    # We wait that can pass all arming check
    copter.wait_ready_to_arm()

    big_print("Let's change some message reception rate")
    # We will change a single message receiption rate by using MESSAGE_INTERVAL.
    # We start getting the current rate
    print("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT rate : %f" % copter.send_get_message_interval(
        ardupilotmega.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT))
    # We set the new rate
    copter.set_message_rate_hz(ardupilotmega.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, 10)
    # We check that it is done correctly
    print("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT rate : %f" % copter.send_get_message_interval(
        ardupilotmega.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT))

    big_print("Let's change some parameters")
    print("RTL_ALT value %f" % copter.get_parameter("RTL_ALT"))
    copter.set_parameters({"RTL_ALT": 2000})
    print("RTL_ALT value %f" % copter.get_parameter("RTL_ALT"))

    big_print("Let's create and write a mission")
    # We will write manually a mission by defining some waypoint
    # We start by initialising mavwp helper library
    copter.init_wp()
    # We get the home position to serve as reference for the mission and as waypoint 0.
    last_home = copter.home_position_as_mav_location()
    # On Copter, we need a takeoff ... for takeoff !
    copter.add_wp_takeoff(last_home.lat, last_home.lng, 10)
    copter.add_waypoint(last_home.lat + 0.0005, last_home.lng + 0.0005, 20)
    copter.add_waypoint(last_home.lat - 0.0005, last_home.lng + 0.0005, 30)
    copter.add_waypoint(last_home.lat - 0.0005, last_home.lng - 0.0005, 20)
    copter.add_waypoint(last_home.lat + 0.0005, last_home.lng - 0.0005, 15)
    # We add a RTL at the end.
    copter.add_wp_rtl()
    # We send everything to the drone
    copter.send_all_waypoints()

    big_print("Let's get the mission written")
    # We get the number of mission waypoint in the drone and print the mission
    wp_count = copter.get_all_waypoints()

    big_print("Let's execute the mission")
    # On ArduPilot, with copter < 4.1 we need to arm before going into Auto mode.
    # We use GUIDED mode as the requirement are closed to AUTO one's
    copter.change_mode("GUIDED")
    # We wait that can pass all arming check
    copter.wait_ready_to_arm()
    copter.arm_vehicle()
    # When armed, we change mode to AUTO
    copter.change_mode("AUTO")
    # As we don't have RC radio here, we trigger mission start with MAVLink.
    copter.send_cmd(mavutil.mavlink.MAV_CMD_MISSION_START,
                    1,  # ARM
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    target_sysid=copter.target_system,
                    target_compid=copter.target_system,
                    )
    # We use the convenient function to track the mission progression
    copter.wait_waypoint(0, wp_count - 1, timeout=500)
    copter.wait_landed_and_disarmed(min_alt=2)

    big_print("Let's do some GUIDED movement")
    # We will do some guided command
    copter.change_mode("GUIDED")
    copter.wait_ready_to_arm()
    if not copter.armed():
        copter.arm_vehicle()
    # example of mavlink takeoff
    copter.user_takeoff(11)
    # example of waiting altitude target
    copter.wait_altitude(10, 13, True)
    # Now we will use a target setpoint
    targetpos = copter.mav.location()
    wp_accuracy = copter.get_parameter("WPNAV_RADIUS", attempts=2)
    wp_accuracy = wp_accuracy * 0.01  # cm to m
    targetpos.lat = targetpos.lat + 0.001
    targetpos.lng = targetpos.lng + 0.001
    targetpos.alt = targetpos.alt + 5
    copter.mav.mav.set_position_target_global_int_send(
        0,  # timestamp
        copter.target_system,  # target system_id
        1,  # target component id
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
        int(targetpos.lat * 1.0e7),  # lat
        int(targetpos.lng * 1.0e7),  # lon
        targetpos.alt,  # alt
        0,  # vx
        0,  # vy
        0,  # vz
        0,  # afx
        0,  # afy
        0,  # afz
        0,  # yaw
        0,  # yawrate
    )
    # Let's control that we are going to the right place
    current_target = copter.get_current_target()
    while current_target.lat != targetpos.lat and current_target.lng != targetpos.lng and current_target.alt != targetpos.alt:
        current_target = copter.get_current_target()

    # Monitor that we are going to the right place
    copter.wait_location(targetpos, accuracy=wp_accuracy, timeout=60,
                         target_altitude=targetpos.alt,
                         height_accuracy=2, minimum_duration=2)
    # Get back to home
    copter.do_RTL()


if __name__ == "__main__":
    main()
