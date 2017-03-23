#!/usr/bin/env python

'''
interactively select accel and gyro data for FFT analysis
'''
from __future__ import print_function

import numpy
import pylab
import matplotlib

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

def fft(logfile):
    '''display fft for raw ACC data in logfile'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    data = {'ACC1.rate' : 1000,
            'ACC2.rate' : 1600,
            'ACC3.rate' : 1000,
            'GYR1.rate' : 1000,
            'GYR2.rate' :  800,
            'GYR3.rate' : 1000}
    for acc in ['ACC1','ACC2','ACC3']:
        for ax in ['AccX', 'AccY', 'AccZ']:
            data[acc+'.'+ax] = []
    for gyr in ['GYR1','GYR2','GYR3']:
        for ax in ['GyrX', 'GyrY', 'GyrZ']:
            data[gyr+'.'+ax] = []

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        type = m.get_type()
        if type.startswith("ACC"):
            data[type+'.AccX'].append(m.AccX)
            data[type+'.AccY'].append(m.AccY)
            data[type+'.AccZ'].append(m.AccZ)
        if type.startswith("GYR"):
            data[type+'.GyrX'].append(m.GyrX)
            data[type+'.GyrY'].append(m.GyrY)
            data[type+'.GyrZ'].append(m.GyrZ)

    print("Extracted %u data points" % len(data['ACC1.AccX']))
    
    # interactive selection of analysis window
    preview = pylab.figure()
    preview.set_size_inches(12, 3, forward=True)
    field = 'ACC1.AccZ'
    d = numpy.array(data[field])
    pylab.plot( d, label=field )
    pylab.legend(loc='upper right')
    pylab.ylabel('m/sec/sec')
    pylab.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.16)
    pylab.ion()
    pylab.show()
    currentAxes = preview.gca()
    print(currentAxes.get_xlim())
    
    try:
        wait = input('select data, then hit enter:')
    except:
        pass
        
    s_start = int(currentAxes.get_xlim()[0])
    s_end = int(currentAxes.get_xlim()[1])
    n_samp = s_end - s_start
    print('sample range: ', s_start, s_end)
    print('N samples: ', n_samp)
    
    preview.canvas.set_window_title("FFT input data length: " + str(n_samp) + " samples")
    pylab.xlabel('sample index : nsamples = ' + str(n_samp))
    preview.savefig('acc1z.png')
        
    for msg in ['ACC1', 'GYR1']:
        fftwin = pylab.figure()
        fftwin.set_size_inches(12, 3, forward=True)

        if msg.startswith('ACC'):
            prefix = 'Acc'
            title = 'Accelerometer FFT'
        else:
            prefix = 'Gyr'
            title = 'Gyro FFT'
            
        for axis in ['X', 'Y', 'Z']:
            field = msg + '.' + prefix + axis
            d = data[field][s_start:s_end]
            d = numpy.array(d)
            if len(d) == 0:
                continue
            avg = numpy.sum(d) / len(d)
            d -= avg
            d_fft = numpy.fft.rfft(d)
            freq  = numpy.fft.rfftfreq(len(d), 1.0 / data[msg+'.rate'])
            f_res = float(data[msg+'.rate']) / n_samp
            pylab.plot( freq, numpy.abs(d_fft), label=field )
            fftwin.canvas.set_window_title(title)
            
        pylab.legend(loc='upper right')
        pylab.xlabel('Hz : resolution = ' + '{0:.3f}'.format(f_res))
        pylab.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.16)
        fftwin.savefig(msg + '_fft.png')

for filename in args.logs:
    fft(filename)

