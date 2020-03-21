#!/usr/bin/env python

'''
extract ISBH and ISBD messages from AP_Logging files and produce FFT plots
'''
from __future__ import print_function

import numpy
import os
import pylab
import sys
import time
import copy

from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")
parser.add_argument("--scale", dest='fft_scale', default='db', action='store', help="scale to use for displaying frequency information: 'db' or 'linear'")
parser.add_argument("--window", dest='fft_window', default='hanning', action='store', help="windowing function to use for processing the data: 'hanning', 'blackman' or 'None'")
parser.add_argument("--overlap", dest='fft_overlap', default=False, action='store_true', help="whether or not to use window overlap when analysing data")
parser.add_argument("--output", dest='fft_output', default='psd', action='store', help="whether to output frequency spectrum information as power or linear spectral density: 'psd' or 'lsd'")

args = parser.parse_args()

from pymavlink import mavutil

def mavfft_fttd(logfile):
    '''display fft for raw ACC data in logfile'''

    '''object to store data about a single FFT plot'''
    class PlotData(object):
        def __init__(self, ffth):
            self.seqno = -1
            self.fftnum = ffth.N
            self.sensor_type = ffth.type
            self.instance = ffth.instance
            self.sample_rate_hz = ffth.smp_rate
            self.multiplier = ffth.mul
            self.data = {}
            self.data["X"] = []
            self.data["Y"] = []
            self.data["Z"] = []
            self.holes = False
            self.freq = None

        def add_fftd(self, fftd):
            if fftd.N != self.fftnum:
                print("Skipping ISBD with wrong fftnum (%u vs %u)\n" % (fftd.fftnum, self.fftnum))
                return
            if self.holes:
                print("Skipping ISBD(%u) for ISBH(%u) with holes in it" % (fftd.seqno, self.fftnum))
                return
            if fftd.seqno != self.seqno+1:
                print("ISBH(%u) has holes in it" % fftd.N)
                self.holes = True
                return
            self.seqno += 1
            self.data["X"].extend(fftd.x)
            self.data["Y"].extend(fftd.y)
            self.data["Z"].extend(fftd.z)

        def overlap_windows(self, plotdata):
            newplotdata = copy.deepcopy(self)
            if plotdata.tag() != self.tag():
                print("Invalid FFT data tag (%s vs %s) for window overlap" % (plotdata.tag(), self.tag()))
                return self
            if plotdata.fftnum <= self.fftnum:
                print("Invalid FFT sequence (%u vs %u) for window overlap" % (plotdata.fftnum, self.fftnum))
                return self
            newplotdata.data["X"] = numpy.split(numpy.asarray(self.data["X"]), 2)[1].tolist() + numpy.split(numpy.asarray(plotdata.data["X"]), 2)[0].tolist()
            newplotdata.data["Y"] = numpy.split(numpy.asarray(self.data["Y"]), 2)[1].tolist() + numpy.split(numpy.asarray(plotdata.data["Y"]), 2)[0].tolist()
            newplotdata.data["Z"] = numpy.split(numpy.asarray(self.data["Z"]), 2)[1].tolist() + numpy.split(numpy.asarray(plotdata.data["Z"]), 2)[0].tolist()
            return newplotdata

        def prefix(self):
            if self.sensor_type == 0:
                return "Accel"
            elif self.sensor_type == 1:
                return "Gyro"
            else:
                return "?Unknown Sensor Type?"

        def tag(self):
            return str(self)

        def __str__(self):
            return "%s[%u]" % (self.prefix(), self.instance)

    print("Processing log %s" % logfile)
    mlog = mavutil.mavlink_connection(logfile)

    # see https://holometer.fnal.gov/GH_FFT.pdf for a description of the techniques used here
    things_to_plot = []
    plotdata = None
    prev_plotdata = {}
    msgcount = 0
    hntch_mode = None
    hntch_option = None
    batch_mode = None
    start_time = time.time()
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        msgcount += 1
        if msgcount % 1000 == 0:
            sys.stderr.write(".")
        msg_type = m.get_type()
        if msg_type == "ISBH":
            if plotdata is not None:
                sensor = plotdata.tag()
                # close off previous data collection
                # in order to get 50% window overlap we need half the old data + half the new data and then the new data itself
                if args.fft_overlap and sensor in prev_plotdata:
                    things_to_plot.append(prev_plotdata[sensor].overlap_windows(plotdata))
                things_to_plot.append(plotdata)
                prev_plotdata[sensor] = plotdata
            plotdata = PlotData(m)
            continue

        if msg_type == "ISBD":
            if plotdata is None:
                sys.stderr.write("?(fftnum=%u)" % m.N)
                continue
            plotdata.add_fftd(m)
        
        if msg_type == "PARM":
            if m.Name == "INS_HNTCH_MODE":
                hntch_mode = m.Value
            elif m.Name == "INS_HNTCH_OPTS":
                hntch_option = m.Value
            elif m.Name == "INS_LOG_BAT_OPT":
                batch_mode = m.Value

    print("", file=sys.stderr)
    time_delta = time.time() - start_time
    print("%us messages  %u messages/second  %u kB/second" % (msgcount, msgcount/time_delta, os.stat(filename).st_size/time_delta))
    print("Extracted %u fft data sets" % len(things_to_plot), file=sys.stderr)

    sum_fft = {}
    freqmap = {}
    sample_rates = {}
    counts = {}
    window = {}
    S2 = {}
    hntch_mode_names = { 0:"No", 1:"Throttle", 2:"RPM", 3:"ESC", 4:"FFT"}
    hntch_option_names = { 0:"Single", 1:"Double"}
    batch_mode_names = { 0:"Pre-filter", 1:"Sensor-rate", 2:"Post-filter" }

    first_freq = None
    for thing_to_plot in things_to_plot:

        fft_len = len(thing_to_plot.data["X"])

        if thing_to_plot.tag() not in sum_fft:
            sum_fft[thing_to_plot.tag()] = {
                "X": numpy.zeros(fft_len//2+1),
                "Y": numpy.zeros(fft_len//2+1),
                "Z": numpy.zeros(fft_len//2+1),
            }
            counts[thing_to_plot.tag()] = 0

        if thing_to_plot.tag() not in window:
            if args.fft_window == 'hanning':
                # create hanning window
                window[thing_to_plot.tag()] = numpy.hanning(fft_len)
            elif args.fft_window == 'blackman':
                # create blackman window
                window[thing_to_plot.tag()] = numpy.blackman(fft_len)
            else:
                window[thing_to_plot.tag()] = numpy.arange(fft_len)
                window[thing_to_plot.tag()].fill(1)
            # calculate NEBW constant
            S2[thing_to_plot.tag()] = numpy.inner(window[thing_to_plot.tag()], window[thing_to_plot.tag()])

        for axis in [ "X","Y","Z" ]:
            # normalize data and convert to dps in order to produce more meaningful magnitudes
            if thing_to_plot.sensor_type == 1:
                d = numpy.array(numpy.degrees(thing_to_plot.data[axis])) / float(thing_to_plot.multiplier)
            else:
                d = numpy.array(thing_to_plot.data[axis]) / float(thing_to_plot.multiplier)
            if len(d) == 0:
                print("No data?!?!?!")
                continue
            if len(window[thing_to_plot.tag()]) != fft_len:
                print("Skipping corrupted frame of length %d" % fft_len)
                continue

            # apply window to the input
            d *= window[thing_to_plot.tag()]
            # perform RFFT
            d_fft = numpy.fft.rfft(d)
            # convert to squared complex magnitude
            d_fft = numpy.square(abs(d_fft))
            # remove DC component
            d_fft[0] = 0
            d_fft[-1] = 0
            # accumulate the sums
            sum_fft[thing_to_plot.tag()][axis] += d_fft
            freq = numpy.fft.rfftfreq(len(d), 1.0/thing_to_plot.sample_rate_hz)
            freqmap[thing_to_plot.tag()] = freq

        sample_rates[thing_to_plot.tag()] = thing_to_plot.sample_rate_hz
        counts[thing_to_plot.tag()] += 1

    numpy.seterr(divide = 'ignore')
    for sensor in sum_fft:
        print("Sensor: %s" % str(sensor))
        fig = pylab.figure(str(sensor))
        for axis in [ "X","Y","Z" ]:
            # normalize output to averaged PSD
            psd = 2 * (sum_fft[sensor][axis] / counts[sensor]) / (sample_rates[sensor] * S2[sensor])
            # linerize of requested
            if args.fft_output == 'lsd':
                psd = numpy.sqrt(psd)
            # convert to db if requested
            if args.fft_scale == 'db':
                psd = 10 * numpy.log10 (psd)
            pylab.plot(freqmap[sensor], psd, label=axis)
        pylab.legend(loc='upper right')
        pylab.xlabel('Hz')
        scale_label=''
        psd_label='PSD'
        if args.fft_scale =='db':
            scale_label="dB "
        if args.fft_output == 'lsd':
            if str(sensor).startswith("Gyro"):
                pylab.ylabel('LSD $' + scale_label + 'd/s/\sqrt{Hz}$')
            else:
                pylab.ylabel('LSD $' + scale_label + 'm/s^2/\sqrt{Hz}$')
        else:
            if str(sensor).startswith("Gyro"):
                pylab.ylabel('PSD $' + scale_label + 'd^2/s^2/Hz$')
            else:
                pylab.ylabel('PSD $' + scale_label + 'm^2/s^4/Hz$')

        if hntch_mode is not None and hntch_option is not None and batch_mode is not None:
            textstr = '\n'.join((
                r'%s tracking' % (hntch_mode_names[hntch_mode], ),
                r'%s notch' % (hntch_option_names[hntch_option], ),
                r'%s sampling' % (batch_mode_names[batch_mode], )))

            props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

            pylab.text(0.5, 0.95, textstr, fontsize=12,
                verticalalignment='top', bbox=props, transform=pylab.gca().transAxes)

for filename in args.logs:
    mavfft_fttd(filename)

pylab.show()
