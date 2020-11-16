#!/usr/bin/env python

'''
Converts a log into a .mat file
Inspired by the data flash log converter in MissionPlanner
Most of the code was copied from mavlogdump.py
'''
from __future__ import print_function

import array
import fnmatch
import json
import os
import struct
import sys
import time
import scipy.io
import numpy as np

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported..")

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("-f", "--follow", action='store_true', help="keep waiting for more data at end of file")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("-p", "--parms", action='store_true', help="preserve parameters in output with -o")
parser.add_argument("--types", default=None, help="types of messages (comma separated with wildcard)")
parser.add_argument("--nottypes", default=None, help="types of messages not to include (comma separated with wildcard)")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("--no-bad-data", action='store_true', help="Don't output corrupted messages")
parser.add_argument("--show-source", action='store_true', help="Show source system ID and component ID")
parser.add_argument("--show-seq", action='store_true', help="Show sequence numbers")
parser.add_argument("--show-types", action='store_true', help="Shows all message types available on opened log")
parser.add_argument("--source-system", type=int, default=None, help="filter by source system ID")
parser.add_argument("--source-component", type=int, default=None, help="filter by source component ID")
parser.add_argument("--link", type=int, default=None, help="filter by comms link ID")
parser.add_argument("--mav10", action='store_true', help="parse as MAVLink1")
parser.add_argument("--reduce", type=int, default=0, help="reduce streaming messages")
parser.add_argument("-c", "--compress", action='store_true', help="Compress .mat file data")
parser.add_argument("log", metavar="LOG")
parser.add_argument("output", metavar="OUTPUT", help="Output .mat file")
parser.add_argument("--profile", action='store_true', help="run the Yappi python profiler")

args = parser.parse_args()

if not args.mav10:
    os.environ['MAVLINK20'] = '1'

import inspect

from pymavlink import mavutil

if args.profile:
    import yappi    # We do the import here so that we won't barf if run normally and yappi not available
    yappi.start()

filename = args.log
mlog = mavutil.mavlink_connection(filename, planner_format=args.planner,
                                  notimestamps=args.notimestamps,
                                  robust_parsing=args.robust,
                                  dialect=args.dialect,
                                  zero_time_base=args.zero_time_base)

types = args.types
if types is not None:
    types = types.split(',')

nottypes = args.nottypes
if nottypes is not None:
    nottypes = nottypes.split(',')

ext = os.path.splitext(filename)[1]
isbin = ext in ['.bin', '.BIN', '.px4log']
islog = ext in ['.log', '.LOG'] # NOTE: "islog" does not mean a tlog
istlog = ext in ['.tlog', '.TLOG']

# list of msgs to reduce in rate when --reduce is used
reduction_msgs = ['NKF*', 'XKF*', 'IMU*', 'AHR2', 'BAR*', 'ATT', 'BAT*', 'CTUN', 'NTUN', 'GP*', 'IMT*', 'MAG*', 'PL', 'POS', 'POW*', 'RATE', 'RC*', 'RFND', 'UBX*', 'VIBE', 'NKQ*', 'MOT*', 'CTRL', 'FTS*', 'DSF', 'CST*', 'LOS*', 'UWB*']
reduction_yes = set()
reduction_no = set()
reduction_count = {}

def reduce_msg(mtype, reduction_ratio):
    '''return True if this msg should be discarded by reduction'''
    global reduction_count, reduction_msgs, reduction_yes, reduction_no
    if mtype in reduction_no:
        return False
    if not mtype in reduction_yes:
        for m in reduction_msgs:
            if fnmatch.fnmatch(mtype, m):
                reduction_yes.add(mtype)
                reduction_count[mtype] = 0
                break
        if not mtype in reduction_yes:
            reduction_no.add(mtype)
            return False
    reduction_count[mtype] += 1
    if reduction_count[mtype] == reduction_ratio:
        reduction_count[mtype] = 0
        return False
    return True

def match_type(mtype, patterns):
    '''return True if mtype matches pattern'''
    for p in patterns:
        if fnmatch.fnmatch(mtype, p):
            return True
    return False

# Track types found
available_types = set()

# for DF logs pre-calculate types list
match_types=None
if types is not None and hasattr(mlog, 'name_to_id'):
    for k in mlog.name_to_id.keys():
        if match_type(k, types):
            if nottypes is not None and match_type(k, nottypes):
                continue
            if match_types is None:
                match_types = []
            match_types.append(k)

# Keep track of data from the current timestep. If the following timestep has the same data, it's stored in here as well. Output should therefore have entirely unique timesteps.
MAT = {}    # Dictionary to hold output data
while True:
    m = mlog.recv_match(blocking=args.follow, type=match_types)
    if m is None:
        # quit
        break
    available_types.add(m.get_type())

    if args.reduce and reduce_msg(m.get_type(), args.reduce):
        continue

    # Apply filters
    if not mavutil.evaluate_condition(args.condition, mlog.messages):
        # Condition not met. skip this message
        continue
    if args.source_system is not None and args.source_system != m.get_srcSystem():
        # Source system filter not satisfied
        continue
    if args.source_component is not None and args.source_component != m.get_srcComponent():
        # Source component filter not satisfied
        continue
    if args.link is not None and args.link != m._link:
        # Link filter not met
        continue

    if types is not None and m.get_type() != 'BAD_DATA' and not match_type(m.get_type(), types):
        continue

    if nottypes is not None and match_type(m.get_type(), nottypes):
        continue

    # Ignore BAD_DATA messages is the user requested or if they're because of a bad prefix. The
    # latter case is normally because of a mismatched MAVLink version.
    if m.get_type() == 'BAD_DATA' and (args.no_bad_data is True or m.reason == "Bad prefix"):
        continue

    # If this packet contains data (i.e. is not a FMT
    # packet), append the data in this packet to the
    # corresponding list
    if m.get_type()!='FMT':

        # If this packet type has not yet been
        # seen, add a new entry to the big dict
        if m.get_type() not in MAT:
            MAT[m.get_type()] = {}

        md = m.to_dict()
        del md['mavpackettype']
        cols = md.keys()
        for col in cols:
            # If this column hasn't had data entered,
            # make a new key and list
            if col in MAT[m.get_type()]:
                MAT[m.get_type()][col].append(md[col])
            else:
                MAT[m.get_type()][col] = [md[col]]

if args.show_types:
    for msgType in available_types:
        print(msgType)

if args.profile:
    yappi.get_func_stats().print_all()
    yappi.get_thread_stats().print_all()

# Convert big dict to dataframes
#for packet_type in MAT:
#    MAT[packet_type] = pd.DataFrame(MAT[packet_type])

# Rearrange the dictionary so it exports correctly
MAT2 = {}
for packet_type in MAT:
    vars = list(MAT[packet_type].keys())
    data = []   # 2D list
    i = 0
    MAT2[packet_type+'_label'] = np.zeros((len(vars), 1), dtype=object)
    for var in vars:
        data.append(MAT[packet_type][var])
        MAT2[packet_type+'_label'][i] = var
        i += 1
    # Transpose the list of lists
    MAT2[packet_type] = list(map(list, zip(*data)))

# Save file
scipy.io.savemat(args.output, MAT2, do_compression=args.compress)
