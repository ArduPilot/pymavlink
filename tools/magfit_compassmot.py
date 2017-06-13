#!/usr/bin/env python

'''
estimate COMPASS_MOT_* parameters for throttle based compensation
'''
from __future__ import print_function
from builtins import range
import math

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

def mag_error(p, data):
    cx,cy,cz = p
    corr = Vector3(cx,cy,cz)
    (baseline_field,baseline_throttle) = data[0]
    ret = []
    for d in data:
        (mag,throttle) = d
        cmag = mag + corr * throttle
        err = (cmag - baseline_field).length()
        ret.append(err)
    return ret

def fit_data(data):
    from scipy import optimize

    p0 = [0.0, 0.0, 0.0]
    p1, ier = optimize.leastsq(mag_error, p0[:], args=(data))
    if not ier in [1, 2, 3, 4]:
        raise RuntimeError("Unable to find solution")
    return Vector3(p1[0], p1[1], p1[2])

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    data = []
    throttle = 0

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        if m.get_type() == "MAG":
            mag = Vector3(m.MagX, m.MagY, m.MagZ)
            data.append((mag, throttle))
        if m.get_type() == "CTUN":
            throttle = m.ThO

    print("Extracted %u data points" % len(data))

    cmot = fit_data(data)
    print("Fit    : %s" % cmot)

for filename in args.logs:
    magfit(filename)
