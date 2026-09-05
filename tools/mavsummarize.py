#!/usr/bin/env python3

'''
Summarize MAVLink logs. Useful for identifying which log is of interest in a large set.
'''

import glob
import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("--health", action='store_true', default=False, help="report flight health (vibration, EKF, battery, GPS, errors)")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import distance_two


class Totals(object):
    def __init__(self):
        self.time = 0
        self.distance = 0
        self.flights = 0

    def print_summary(self):
        print("===============================")
        print("Num Flights : %u" % self.flights)
        print("Total distance : {:0.2f}m".format(self.distance))
        print("Total time (mm:ss): {:3.0f}:{:02.0f}".format(self.time / 60, self.time % 60))


totals = Totals()

def PrintSummary(logfile):
    '''Calculate some interesting datapoints of the file'''
    # Open the log file
    mlog = mavutil.mavlink_connection(logfile, notimestamps=args.notimestamps, dialect=args.dialect)

    autonomous_sections = 0 # How many different autonomous sections there are
    autonomous = False # Whether the vehicle is currently autonomous at this point in the logfile
    auto_time = 0.0 # The total time the vehicle was autonomous/guided (seconds)
    start_time = None # The datetime of the first received message (seconds since epoch)
    total_dist = 0.0 # The total ground distance travelled (meters)
    last_gps_msg = None # The first GPS message received
    first_gps_msg = None # The last GPS message received
    true_time = None # Track the first timestamp found that corresponds to a UNIX timestamp

    types = set(['HEARTBEAT', 'GPS_RAW_INT'])

    # Health tracking accumulators (only used with --health)
    vibe_x_max = vibe_y_max = vibe_z_max = 0.0
    clip_total = 0
    ekf_sv_max = ekf_sp_max = ekf_sh_max = 0.0
    ekf_fault_union = 0
    ekf_msg_type = None
    bat_first_volt = bat_last_volt = bat_min_volt = None
    gps_worst_hdop = 0.0
    gps_min_nsats = None
    health_errors = []

    if args.health:
        types.update(['VIBE', 'XKF4', 'NKF4', 'BAT', 'GPS', 'ERR'])

    while True:
        m = mlog.recv_match(condition=args.condition, type=types)

        # If there's no match, it means we're done processing the log.
        if m is None:
            break

        # Ignore any failed messages
        if m.get_type() == 'BAD_DATA':
            continue

        # Keep track of the latest timestamp for various calculations
        timestamp = getattr(m, '_timestamp', 0.0)

        # Log the first message time
        if start_time is None:
            start_time = timestamp

        # Log the first timestamp found that is a true timestamp. We first try
        # to get the groundstation timestamp from the log directly. If that fails
        # we try to find a message that outputs a Unix time-since-epoch.
        if true_time is None:
            if not args.notimestamps and timestamp >= 1230768000:
                true_time = timestamp
            elif 'time_unix_usec' in m.__dict__ and m.time_unix_usec >= 1230768000:
                true_time = m.time_unix_usec * 1.0e-6
            elif 'time_usec' in m.__dict__ and m.time_usec >= 1230768000:
                true_time = m.time_usec * 1.0e-6

        # Track the vehicle's speed and status
        if m.get_type() == 'GPS_RAW_INT':

            # Ignore GPS messages without a proper fix
            if m.fix_type < 3 or m.lat == 0 or m.lon == 0:
                continue

            # Log the first GPS location found, being sure to skip GPS fixes
            # that are bad (at lat/lon of 0,0)
            if first_gps_msg is None:
                first_gps_msg = m

            # Track the distance travelled, being sure to skip GPS fixes
            # that are bad (at lat/lon of 0,0)
            if last_gps_msg is None or m.time_usec > last_gps_msg.time_usec or m.time_usec+30e6 < last_gps_msg.time_usec:
                if last_gps_msg is not None:
                    total_dist += distance_two(last_gps_msg, m)

                # Save this GPS message to do simple distance calculations with
                last_gps_msg = m

        elif m.get_type() == 'HEARTBEAT':
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED or
                m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and not autonomous:
                autonomous = True
                autonomous_sections += 1
                start_auto_time = timestamp
            elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED and
                not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous:
                autonomous = False
                auto_time += timestamp - start_auto_time

        # --health: accumulate stats from additional message types
        if args.health:
            mtype = m.get_type()

            if mtype == 'VIBE':
                vibe_x_max = max(vibe_x_max, getattr(m, 'VibeX', 0.0))
                vibe_y_max = max(vibe_y_max, getattr(m, 'VibeY', 0.0))
                vibe_z_max = max(vibe_z_max, getattr(m, 'VibeZ', 0.0))
                clip_total += getattr(m, 'Clip0', 0) + getattr(m, 'Clip1', 0) + getattr(m, 'Clip2', 0)

            elif mtype in ('XKF4', 'NKF4'):
                # prefer XKF4 (EKF3) over NKF4 (EKF2) if both exist
                if ekf_msg_type is None:
                    ekf_msg_type = mtype
                elif mtype == 'XKF4' and ekf_msg_type == 'NKF4':
                    ekf_msg_type = 'XKF4'
                    ekf_sv_max = ekf_sp_max = ekf_sh_max = 0.0
                    ekf_fault_union = 0
                if mtype == ekf_msg_type:
                    ekf_sv_max = max(ekf_sv_max, getattr(m, 'SV', 0.0))
                    ekf_sp_max = max(ekf_sp_max, getattr(m, 'SP', 0.0))
                    ekf_sh_max = max(ekf_sh_max, getattr(m, 'SH', 0.0))
                    ekf_fault_union |= int(getattr(m, 'FS', 0))

            elif mtype == 'BAT':
                volt = getattr(m, 'Volt', 0.0)
                if bat_first_volt is None:
                    bat_first_volt = volt
                bat_last_volt = volt
                if bat_min_volt is None or volt < bat_min_volt:
                    bat_min_volt = volt

            elif mtype == 'GPS':
                hdop = getattr(m, 'HDop', 0.0)
                nsats = getattr(m, 'NSats', 0)
                gps_worst_hdop = max(gps_worst_hdop, hdop)
                if gps_min_nsats is None or nsats < gps_min_nsats:
                    gps_min_nsats = nsats

            elif mtype == 'ERR':
                subsys = int(getattr(m, 'Subsys', 0))
                ecode = int(getattr(m, 'ECode', 0))
                health_errors.append((subsys, ecode))

    # If there were no messages processed, say so
    if start_time is None:
        print("ERROR: No messages found.")
        return

    # If the vehicle ends in autonomous mode, make sure we log the total time
    if autonomous:
        auto_time += timestamp - start_auto_time

    # Compute the total logtime, checking that timestamps are 2009 or newer for validity
    # (MAVLink didn't exist until 2009)
    if true_time:
        start_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(true_time))
        print("Log started at about {}".format(start_time_str))
    else:
        print("Warning: No absolute timestamp found in datastream. No starting time can be provided for this log.")

    # Print location data
    if last_gps_msg is not None:
        first_gps_position = (first_gps_msg.lat / 1e7, first_gps_msg.lon / 1e7)
        last_gps_position = (last_gps_msg.lat / 1e7, last_gps_msg.lon / 1e7)
        print("Travelled from ({0[0]}, {0[1]}) to ({1[0]}, {1[1]})".format(first_gps_position, last_gps_position))
        print("Total distance : {:0.2f}m".format(total_dist))
    else:
        print("Warning: No GPS data found, can't give position summary.")

    # Print out the rest of the results.
    total_time = timestamp - start_time
    print("Total time (mm:ss): {:3.0f}:{:02.0f}".format(total_time / 60, total_time % 60))
    # The autonomous time should be good, as a steady HEARTBEAT is required for MAVLink operation
    print("Autonomous sections: {}".format(autonomous_sections))
    if autonomous_sections > 0:
        print("Autonomous time (mm:ss): {:3.0f}:{:02.0f}".format(auto_time / 60, auto_time % 60))

    # --health report
    if args.health:
        err_subsys_names = {
            1: "Main", 2: "Radio", 3: "Compass", 5: "Radio/FS",
            6: "EKF/InertialNav", 12: "CrashCheck", 17: "EKF_primary",
            22: "GPS_Glitch", 26: "EKF_primary", 27: "EKF_lane_switch",
        }
        print("")
        print("=== Flight Health ===")
        # vibration
        if vibe_x_max > 0 or vibe_y_max > 0 or vibe_z_max > 0:
            print("Vibration:  max X=%.1f Y=%.1f Z=%.1f m/s² | Clips: %d" % (
                vibe_x_max, vibe_y_max, vibe_z_max, clip_total))
        else:
            print("Vibration:  no VIBE data")
        # ekf
        if ekf_msg_type is not None:
            if ekf_sv_max > 1.0 or ekf_sp_max > 1.0 or ekf_sh_max > 1.0:
                status = "WARNING"
            elif ekf_sv_max > 0.8 or ekf_sp_max > 0.8 or ekf_sh_max > 0.8:
                status = "MARGINAL"
            else:
                status = "OK"
            print("EKF:        max SV=%.2f SP=%.2f SH=%.2f (%s) | Faults: %d | %s" % (
                ekf_sv_max, ekf_sp_max, ekf_sh_max, ekf_msg_type, ekf_fault_union, status))
        else:
            print("EKF:        no XKF4/NKF4 data")
        # battery
        if bat_first_volt is not None:
            print("Battery:    %.1fV → %.1fV (min %.1fV)" % (bat_first_volt, bat_last_volt, bat_min_volt))
        else:
            print("Battery:    no BAT data")
        # gps
        if gps_min_nsats is not None:
            print("GPS:        worst HDop=%.1f | min sats=%d" % (gps_worst_hdop, gps_min_nsats))
        else:
            print("GPS:        no GPS data")
        # errors
        if health_errors:
            seen = set()
            parts = []
            for subsys, ecode in health_errors:
                key = (subsys, ecode)
                if key not in seen:
                    seen.add(key)
                    name = err_subsys_names.get(subsys, "Unknown(%d)" % subsys)
                    parts.append("%s(code=%d)" % (name, ecode))
            print("Errors:     %s" % ", ".join(parts))
        else:
            print("Errors:     none")

    totals.time += total_time
    totals.distance += total_dist
    totals.flights += 1

for filename in args.logs:
    for f in glob.glob(filename):
        print("Processing log %s" % filename)
        PrintSummary(f)

totals.print_summary()
