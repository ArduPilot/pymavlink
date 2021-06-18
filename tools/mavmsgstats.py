#!/usr/bin/env python

'''
show stats on messages in a log
'''

import os
import fnmatch

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

categories = {
    'EKF2' : ['NK*'],
    'EKF3' : ['XK*'],
    'SENSORS' : ['IMU*', 'BAR*', 'GPS*', 'RFND', 'ACC', 'GYR' ],
    'RC' : ['RCIN', 'RCOU' ],
    'TUNING' : ['RATE', 'PID*', 'CTUN', 'NTUN', 'ATT', 'PSC'],
    'SYSTEM' : ['MAV', 'BAT*', 'EV', 'CMD', 'MODE'],
    'REPLAY' : [ 'RFRH', 'RFRF', 'REV2', 'RSO2', 'RWA2', 'REV3', 'RSO3', 'RWA3', 'RMGI',
                 'REY3', 'RFRN', 'RISH', 'RISI', 'RISJ', 'RBRH', 'RBRI', 'RRNH', 'RRNI',
                 'RGPH', 'RGPI', 'RGPJ', 'RASH', 'RASI', 'RBCH', 'RBCI', 'RVOH', 'RMGH',
                 'R??H', 'R??I', 'R??J'],
}

def show_stats(logfile):
    '''show stats on a file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)
    sizes = {}
    total_size = 0
    names = mlog.name_to_id.keys()
    pairs = []

    for name in names:
        sizes[name] = 0

    for name in names:
        mid = mlog.name_to_id[name]
        count = mlog.counts[mid]
        mlen = mlog.formats[mid].len
        size = count * mlen
        total_size += size
        sizes[name] += size
        pairs.append((name, count*mlen))

    pairs = sorted(pairs, key = lambda p : p[1])
    print("Total size: %u" % total_size)
    for (name,size) in pairs:
        if size > 0:
            print(f"{name:<4} {100.0 * size / total_size:.2f}%")

    print("")
    category_total = 0
    for c in categories.keys():
        total = 0
        for name in names:
            for p in categories[c]:
                if fnmatch.fnmatch(name, p):
                    total += sizes[name]
                    break
        category_total += total
        if total > 0:
            print(f"@{c} {100.0 * total / total_size:.2f}%")
    print("@OTHER %.2f%%" % (100.0 * (total_size-category_total) / total_size))

for filename in args.logs:
    show_stats(filename)
