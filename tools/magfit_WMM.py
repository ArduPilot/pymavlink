#!/usr/bin/env python

'''
fit best estimate of magnetometer offsets, diagonals, off-diagonals, cmot and scaling using WMM target
'''

import sys, time, os, math, copy

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--mag", type=int, default=1, help="mag index, 1 for first mag")
parser.add_argument("--reduce", type=int, default=1, help="reduce data points by given factor")
parser.add_argument("--min-scale", type=float, default=0.6, help="min scale factor")
parser.add_argument("--max-scale", type=float, default=1.5, help="max scale factor")
parser.add_argument("--max-offset", type=float, default=1500, help="max offset")
parser.add_argument("--min-diag", type=float, default=0.8, help="min diagonal")
parser.add_argument("--max-diag", type=float, default=1.2, help="max diagonal")
parser.add_argument("--min-offdiag", type=float, default=-0.2, help="min off diagonal")
parser.add_argument("--max-offdiag", type=float, default=0.2, help="max off diagonal")
parser.add_argument("--max-cmot", type=float, default=10.0, help="max compassmot")
parser.add_argument("--no-offset-change", action='store_true', help="don't change offsets")
parser.add_argument("--no-cmot-change", action='store_true', help="don't change cmot")
parser.add_argument("--elliptical", action='store_true', help="fit elliptical corrections")
parser.add_argument("--cmot", action='store_true', help="fit compassmot corrections")
parser.add_argument("log", metavar="LOG")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink import mavextra
from pymavlink.rotmat import Vector3
from pymavlink.rotmat import Matrix3

import matplotlib
import matplotlib.pyplot as pyplot
import numpy

earth_field = None
declination = None

if args.mag > 1:
    mag_idx = str(args.mag)
else:
    mag_idx = ''

class Correction:
    def __init__(self):
        self.offsets = Vector3(0.0, 0.0, 0.0)
        self.diag = Vector3(1.0, 1.0, 1.0)
        self.offdiag = Vector3(0.0, 0.0, 0.0)
        self.cmot = Vector3(0.0, 0.0, 0.0)
        self.scaling = 1.0

    def show_parms(self):
        print("COMPASS_OFS%s_X %d" % (mag_idx, int(self.offsets.x)))
        print("COMPASS_OFS%s_Y %d" % (mag_idx, int(self.offsets.y)))
        print("COMPASS_OFS%s_Z %d" % (mag_idx, int(self.offsets.z)))
        print("COMPASS_DIA%s_X %.3f" % (mag_idx, self.diag.x))
        print("COMPASS_DIA%s_Y %.3f" % (mag_idx, self.diag.y))
        print("COMPASS_DIA%s_Z %.3f" % (mag_idx, self.diag.z))
        print("COMPASS_ODI%s_X %.3f" % (mag_idx, self.offdiag.x))
        print("COMPASS_ODI%s_Y %.3f" % (mag_idx, self.offdiag.y))
        print("COMPASS_ODI%s_Z %.3f" % (mag_idx, self.offdiag.z))
        print("COMPASS_MOT%s_X %.3f" % (mag_idx, self.cmot.x))
        print("COMPASS_MOT%s_Y %.3f" % (mag_idx, self.cmot.y))
        print("COMPASS_MOT%s_Z %.3f" % (mag_idx, self.cmot.z))
        print("COMPASS_SCALE%s %.2f" % (mag_idx, self.scaling))

def correct(MAG, BAT, c):
    '''correct a mag sample, returning a Vector3'''
    mag = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)

    # add the given offsets
    mag += c.offsets

    # multiply by scale factor
    mag *= c.scaling

    # apply elliptical corrections
    mat = Matrix3(
        Vector3(c.diag.x,    c.offdiag.x,  c.offdiag.y),
        Vector3(c.offdiag.x, c.diag.y,     c.offdiag.z),
        Vector3(c.offdiag.y, c.offdiag.z,  c.diag.z))

    mag = mat * mag

    # apply compassmot corrections
    if BAT is not None and not math.isnan(BAT.Curr):
        mag += c.cmot * BAT.Curr

    return mag

def get_yaw(ATT,MAG,BAT,c):
    '''calculate heading from raw magnetometer and new offsets'''

    mag = correct(MAG, BAT, c)

    # go via a DCM matrix to match the APM calculation
    dcm_matrix = mavextra.rotation_df(ATT)
    cos_pitch_sq = 1.0-(dcm_matrix.c.x*dcm_matrix.c.x)
    headY = mag.y * dcm_matrix.c.z - mag.z * dcm_matrix.c.y
    headX = mag.x * cos_pitch_sq - dcm_matrix.c.x * (mag.y * dcm_matrix.c.y + mag.z * dcm_matrix.c.z)

    global declination
    yaw = math.degrees(math.atan2(-headY,headX)) + declination
    if yaw < 0:
        yaw += 360
    return yaw

def expected_field(ATT, yaw):
    '''return expected magnetic field for attitude'''
    global earth_field

    roll = ATT.Roll
    pitch = ATT.Pitch

    rot = Matrix3()
    rot.from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))

    field = rot.transposed() * earth_field

    return field

data = None
old_corrections = Correction()

def wmm_error(p):
    '''world magnetic model error with correction fit'''
    p = list(p)
    c = copy.copy(old_corrections)

    c.offsets = Vector3(p.pop(0), p.pop(0), p.pop(0))
    c.scaling = p.pop(0)
    if args.elliptical:
        c.diag = Vector3(p.pop(0), p.pop(0), p.pop(0))
        c.offdiag = Vector3(p.pop(0), p.pop(0), p.pop(0))
    else:
        c.diag = Vector3(1.0, 1.0, 1.0)
        c.offdiag = Vector3(0.0, 0.0, 0.0)

    if args.cmot:
        c.cmot = Vector3(p.pop(0), p.pop(0), p.pop(0))

    ret = 0

    for (MAG,ATT,BAT) in data:
        yaw = get_yaw(ATT,MAG,BAT,c)
        expected = expected_field(ATT, yaw)
        observed = correct(MAG,BAT,c)

        error = (expected - observed).length()
        ret += error

    ret /= len(data)

    return ret

def fit_WWW():
    from scipy import optimize

    c = copy.copy(old_corrections)
    p = [c.offsets.x, c.offsets.y, c.offsets.z, c.scaling]
    if args.elliptical:
        p.extend([c.diag.x, c.diag.y, c.diag.z, c.offdiag.x, c.offdiag.y, c.offdiag.z])
    if args.cmot:
        p.extend([c.cmot.x, c.cmot.y, c.cmot.z])

    ofs = args.max_offset
    bounds = [(-ofs,ofs),(-ofs,ofs),(-ofs,ofs),(args.min_scale,args.max_scale)]
    if args.no_offset_change:
        bounds[0] = (c.offsets.x, c.offsets.x)
        bounds[1] = (c.offsets.y, c.offsets.y)
        bounds[2] = (c.offsets.z, c.offsets.z)

    if args.elliptical:
        for i in range(3):
            bounds.append((args.min_diag,args.max_diag))
        for i in range(3):
            bounds.append((args.min_offdiag,args.max_offdiag))

    if args.cmot:
        if args.no_cmot_change:
            bounds.append((c.cmot.x, c.cmot.x))
            bounds.append((c.cmot.y, c.cmot.y))
            bounds.append((c.cmot.z, c.cmot.z))
        else:
            for i in range(3):
                bounds.append((-args.max_cmot,args.max_cmot))

    p = optimize.fmin_slsqp(wmm_error, p, bounds=bounds)
    p = list(p)

    c.offsets = Vector3(p.pop(0), p.pop(0), p.pop(0))
    c.scaling = p.pop(0)

    if args.elliptical:
        c.diag = Vector3(p.pop(0), p.pop(0), p.pop(0))
        c.offdiag = Vector3(p.pop(0), p.pop(0), p.pop(0))
    else:
        c.diag = Vector3(1.0, 1.0, 1.0)
        c.offdiag = Vector3(0.0, 0.0, 0.0)

    if args.cmot:
        c.cmot = Vector3(p.pop(0), p.pop(0), p.pop(0))
    else:
        c.cmot = Vector3(0.0, 0.0, 0.0)
    return c

def remove_offsets(MAG, BAT, c):
    '''remove all corrections to get raw sensor data'''
    correction_matrix = Matrix3(Vector3(c.diag.x,    c.offdiag.x, c.offdiag.y),
                                Vector3(c.offdiag.x, c.diag.y,    c.offdiag.z),
                                Vector3(c.offdiag.y, c.offdiag.z, c.diag.z))
    correction_matrix = correction_matrix.invert()

    field = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)
    if BAT is not None and not math.isnan(BAT.Curr):
        field -= c.cmot * BAT.Curr
    field = correction_matrix * field
    field *= 1.0 / c.scaling
    field -= Vector3(MAG.OfsX, MAG.OfsY, MAG.OfsZ)

    if math.isnan(field.x) or math.isnan(field.y) or math.isnan(field.z):
        return False
    MAG.MagX = int(field.x)
    MAG.MagY = int(field.y)
    MAG.MagZ = int(field.z)
    return True

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % logfile)
    mlog = mavutil.mavlink_connection(logfile)

    global earth_field, declination

    global data
    data = []

    ATT = None
    BAT = None

    mag_msg = 'MAG%s' % mag_idx

    count = 0
    parameters = {}

    # get parameters
    while True:
        msg = mlog.recv_match(type=['PARM'])
        if msg is None:
            break
        parameters[msg.Name] = msg.Value

    mlog.rewind()

    # extract MAG data
    while True:
        msg = mlog.recv_match(type=['GPS',mag_msg,'ATT','CTUN','BARO', 'BAT'], condition=args.condition)
        if msg is None:
            break
        if msg.get_type() == 'GPS' and msg.Status >= 3 and earth_field is None:
            earth_field = mavextra.expected_earth_field(msg)
            (declination,inclination,intensity) = mavextra.get_mag_field_ef(msg.Lat, msg.Lng)
            print("Earth field: %s  strength %.0f declination %.1f degrees" % (earth_field, earth_field.length(), declination))
        if msg.get_type() == 'ATT':
            ATT = msg
        if msg.get_type() == 'BAT':
            BAT = msg
        if msg.get_type() == mag_msg and ATT is not None:
            if count % args.reduce == 0:
                data.append((msg,ATT,BAT))
            count += 1

    old_corrections.offsets = Vector3(parameters.get('COMPASS_OFS%s_X' % mag_idx,0.0),
                                      parameters.get('COMPASS_OFS%s_Y' % mag_idx,0.0),
                                      parameters.get('COMPASS_OFS%s_Z' % mag_idx,0.0))
    old_corrections.diag = Vector3(parameters.get('COMPASS_DIA%s_X' % mag_idx,1.0),
                                   parameters.get('COMPASS_DIA%s_Y' % mag_idx,1.0),
                                   parameters.get('COMPASS_DIA%s_Z' % mag_idx,1.0))
    old_corrections.offdiag = Vector3(parameters.get('COMPASS_ODI%s_X' % mag_idx,0.0),
                                      parameters.get('COMPASS_ODI%s_Y' % mag_idx,0.0),
                                      parameters.get('COMPASS_ODI%s_Z' % mag_idx,0.0))
    if parameters.get('COMPASS_MOTCT',0) == 2:
        # only support current based corrections for now
        old_corrections.cmot = Vector3(parameters.get('COMPASS_MOT%s_X' % mag_idx,0.0),
                                       parameters.get('COMPASS_MOT%s_Y' % mag_idx,0.0),
                                       parameters.get('COMPASS_MOT%s_Z' % mag_idx,0.0))
    old_corrections.scaling = parameters.get('COMPASS_SCALE%s' % mag_idx, None)
    if old_corrections.scaling is None or old_corrections.scaling < 0.1:
        force_scale = False
        old_corrections.scaling = 1.0
    else:
        force_scale = True

    # remove existing corrections
    data2 = []
    for (MAG,ATT,BAT) in data:
        if remove_offsets(MAG, BAT, old_corrections):
            data2.append((MAG,ATT,BAT))
    data = data2

    print("Extracted %u points" % len(data))
    print("Current: %s diag: %s offdiag: %s cmot: %s scale: %.2f" % (
        old_corrections.offsets, old_corrections.diag, old_corrections.offdiag, old_corrections.cmot, old_corrections.scaling))

    # do fit
    c = fit_WWW()

    # normalise diagonals to scale factor
    if force_scale:
        avgdiag = (c.diag.x + c.diag.y + c.diag.z)/3.0
        calc_scale = c.scaling
        c.scaling *= avgdiag
        if c.scaling > args.max_scale:
            c.scaling = args.max_scale
        if c.scaling < args.min_scale:
            c.scaling = args.min_scale
        scale_change = c.scaling / calc_scale
        c.diag *= 1.0/scale_change
        c.offdiag *= 1.0/scale_change

    print("New: %s diag: %s offdiag: %s cmot: %s scale: %.2f" % (
        c.offsets, c.diag, c.offdiag, c.cmot, c.scaling))

    x = []

    corrected = {}
    corrected['Yaw'] = []
    expected1 = {}
    expected2 = {}
    uncorrected = {}
    uncorrected['Yaw'] = []
    yaw_change1 = []
    yaw_change2 = []
    for i in range(len(data)):
        (MAG,ATT,BAT) = data[i]
        yaw1 = get_yaw(ATT,MAG,BAT,c)
        corrected['Yaw'].append(yaw1)
        ef1 = expected_field(ATT, yaw1)
        cf = correct(MAG, BAT, c)

        yaw2 = get_yaw(ATT,MAG,BAT,old_corrections)
        ef2 = expected_field(ATT, yaw2)
        uncorrected['Yaw'].append(yaw2)
        uf = correct(MAG, BAT, old_corrections)

        yaw_change1.append(mavextra.wrap_180(yaw1 - yaw2))
        yaw_change2.append(mavextra.wrap_180(yaw1 - ATT.Yaw))
        for axis in ['x','y','z']:
            if not axis in corrected:
                corrected[axis] = []
                uncorrected[axis] = []
                expected1[axis] = []
                expected2[axis] = []
            corrected[axis].append(getattr(cf, axis))
            uncorrected[axis].append(getattr(uf, axis))
            expected1[axis].append(getattr(ef1, axis))
            expected2[axis].append(getattr(ef2, axis))
        x.append(i)

    c.show_parms()

    fig, axs = pyplot.subplots(3, 1, sharex=True)

    for axis in ['x','y','z']:
        axs[0].plot(numpy.array(x), numpy.array(uncorrected[axis]), label='Uncorrected %s' % axis.upper() )
        axs[0].plot(numpy.array(x), numpy.array(expected2[axis]), label='Expected %s' % axis.upper() )
        axs[0].legend(loc='upper left')
        axs[0].set_title('Original')
        axs[0].set_ylabel('Field (mGauss)')

        axs[1].plot(numpy.array(x), numpy.array(corrected[axis]), label='Corrected %s' % axis.upper() )
        axs[1].plot(numpy.array(x), numpy.array(expected1[axis]), label='Expected %s' % axis.upper() )
        axs[1].legend(loc='upper left')
        axs[1].set_title('Corrected')
        axs[1].set_ylabel('Field (mGauss)')

    # show change in yaw estimate from old corrections to new
    axs[2].plot(numpy.array(x), numpy.array(yaw_change1), label='Mag Yaw Change')
    axs[2].plot(numpy.array(x), numpy.array(yaw_change2), label='ATT Yaw Change')
    axs[2].set_title('Yaw Change (degrees)')
    axs[2].legend(loc='upper left')

    pyplot.show()


magfit(args.log)
