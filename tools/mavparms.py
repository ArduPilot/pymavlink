#!/usr/bin/env python

"""
Extract mavlink parameter values from log files.
Its works both on dataflashlog (.BIN) and (.log) or telemetry log (.tlog).
"""
from __future__ import print_function
import time
from pymavlink import mavutil

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("-c", "--changes", dest="changesOnly", default=False, action="store_true", help="Show only changes to parameters.")
parser.add_argument("logs", metavar="LOG", nargs="+")
parser.add_argument("-s", "--save", action="store_true", help="Save the extracted parameters to a file.")

args = parser.parse_args()

parms = {}
saved_file = None
def mavparms(logfile):
    """Extract mavlink parameters."""
    mlog = mavutil.mavlink_connection(filename)

    while True:
        try:
            m = mlog.recv_match(type=['PARAM_VALUE', 'PARM'])
            if m is None:
                return
        except Exception:
            return
        if m.get_type() == 'PARAM_VALUE':
            pname = str(m.param_id).strip()
            value = m.param_value
        else:
            pname = m.Name
            value = m.Value
        if len(pname) > 0:
            if args.changesOnly is True and pname in parms and parms[pname] != value:
                print("%s %-15s %.6f -> %.6f" % (time.asctime(time.localtime(m._timestamp)), pname, parms[pname], value))

            parms[pname] = value


total = 0.0
for filename in args.logs:
    print("# Extracting Parameters from %s" % filename)
    mavparms(filename)


if (args.changesOnly is False):
    if args.save:
        formated_file_name = args.logs[0].rsplit(".", 1)[0] + ".parm"
        saved_file = open(formated_file_name, "w")
        print("# Saving extracted parameters to %s" % formated_file_name)
        saved_file.write("# Parameters extracted from %s file\n" % args.logs[0])
    keys = list(parms.keys())
    keys.sort()
    for p in keys:
        line = "%-15s %.6f" % (p, parms[p])
        print(line)
        if args.save:
            saved_file.write(line + "\n")
    saved_file.close()
