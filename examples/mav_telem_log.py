#!/usr/bin/env python

'''
connect to a mav and log received messages
'''
from __future__ import print_function

from argparse import ArgumentParser
from pymavlink import mavutil

parser = ArgumentParser()
parser.add_argument("--notimestamps", action='store_true', help="no timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()


if __debug__:
    import pymavlink.dialects.v10.ardupilotmega as mavlink

    mavutil.mavlink = mavlink


def process(logfile):
    '''display all logs from a file'''
    mlog = mavutil.mavlink_connection(logfile,
                                      notimestamps=args.notimestamps,
                                      planner_format=args.planner,
                                      robust_parsing=args.robust)

    while True:
        msg: mavlink.MAVLink_message = mlog.recv_match()
        if msg is None:
            break
        print(msg.get_type(), msg)


for filename in args.logs:
    process(filename)
