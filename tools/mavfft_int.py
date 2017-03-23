#!/usr/bin/env python

'''
interactively select accel and gyro data for FFT analysis
'''
from __future__ import print_function

import numpy
import pylab
import matplotlib
import matplotlib.pyplot as pyplot

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

    data = {'ACC1.rate' : 900,
            'ACC2.rate' : 1600,
            'ACC3.rate' : 1000,
            'GYR1.rate' : 900,
            'GYR2.rate' :  800,
            'GYR3.rate' : 1000}
    for acc in ['ACC1','ACC2','ACC3']:
        for ax in ['AccX', 'AccY', 'AccZ']:
            data[acc+'.'+ax] = []
    for gyr in ['GYR1','GYR2','GYR3']:
        for ax in ['GyrX', 'GyrY', 'GyrZ']:
            data[gyr+'.'+ax] = []

    # now gather all the data
    timestamp1 = []
    seqcnt1 = []
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        type = m.get_type()
        if type.startswith("ACC1"):
            seqcnt1.append(m.SampleUS)
            timestamp1.append(m.TimeUS)
        if type.startswith("ACC"):
            data[type+'.AccX'].append(m.AccX)
            data[type+'.AccY'].append(m.AccY)
            data[type+'.AccZ'].append(m.AccZ)
        if type.startswith("GYR"):
            data[type+'.GyrX'].append(m.GyrX)
            data[type+'.GyrY'].append(m.GyrY)
            data[type+'.GyrZ'].append(m.GyrZ)

    # SampleUS is just a sample counter
    ts = 1e-6 * numpy.array(timestamp1)
    seqcnt = numpy.array(seqcnt1)

    print("Extracted %u data points" % len(data['ACC1.AccX']))
    
    # interactive selection of analysis window
    preview = pylab.figure()
    preview.set_size_inches(12, 3, forward=True)
    msg = 'ACC1'
    for axis in ['X', 'Y', 'Z']:
        field = msg + '.Acc' + axis
        d = numpy.array(data[field])
        pylab.plot( d, label=field )
    pylab.legend(loc='upper right')
    pylab.ylabel('m/sec/sec')
    pylab.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.16)
    pylab.show()
    currentAxes = preview.gca()
    s_start = 0
    s_end = len(timestamp1)-1
    n_samp = s_end - s_start
    currentAxes.set_xlim(s_start, s_end)

    # outer loop for repeating time window selection
    while True:
        
        while True:
            print('select sample range for fft analysis')
            preview.canvas.set_window_title('select sample range')
            try:
                s_start = input('start sample: ')
                s_end = input('end sample: ')
                currentAxes.set_xlim(s_start, s_end)
            except:
                break
            
        # process selected samples
        s_start = int(currentAxes.get_xlim()[0])
        s_end = int(currentAxes.get_xlim()[1])
        n_samp = s_end - s_start
        print('sample range: ', s_start, s_end)
        print('N samples: ', n_samp)
        
        # check for dropouts: (delta > 1)
        deltas = numpy.diff(seqcnt[s_start:s_end])
        print('ndeltas: ', len(deltas))
        duration = ts[s_end] - ts[s_start]
        print('duration: {0:.3f} seconds'.format(duration))
        avg_rate = float(n_samp-1) / duration
        print('average sample rate: {0:.0f} Hz'.format(avg_rate))
        ts_mean = numpy.mean(deltas) 
        print('mean sample interval: ', '{0:.2f}'.format(ts_mean))
        print('std: ', '{0:.0f}'.format(numpy.std(deltas)))
        
        title = 'FFT input: {0:s} ACC1[{1:d}:{2:d}], {3:d} samples'.format(logfile, s_start, s_end, n_samp)
        pylab.xlabel('sample index : nsamples: {0:d}, avg rate: {1:.0f} Hz'.format(n_samp, avg_rate))
        preview.canvas.set_window_title(title)
        preview.savefig('acc1z.png')
            
        drop_lens = []
        drop_times = []
        intvl_count = [0]
        interval = 0
        for i in range(0, len(deltas)):
            if (deltas[i] < 1.5 * ts_mean):
                interval += 1
                intvl_count.append(interval)
            else:
                interval += 2
                intvl_count.append(interval)
                drop_lens.append(deltas[i])
                drop_times.append(ts[s_start+i])
                #print('dropout at sample {0}: length {1:.2f} msec'.format(i, 1e3*deltas[i]))
        
        print('{0:d} sample intervals > {1:.3f} msec'.format(len(drop_lens), 1.5 * ts_mean))

        for msg in ['ACC1', 'GYR1']:
            fftwin = pylab.figure()
            fftwin.set_size_inches(12, 3, forward=True)
            f_res = float(data[msg+'.rate']) / n_samp
    
            if msg.startswith('ACC'):
                prefix = 'Acc'
                title = 'Accelerometer FFT [{0:d}:{1:d}]'.format(s_start, s_end)
            else:
                prefix = 'Gyr'
                title = 'Gyro FFT [{0:d}:{1:d}]'.format(s_start, s_end)
                
            max_fft = 0
            abs_fft = {}
            index = 0
            for axis in ['X', 'Y', 'Z']:
                field = msg + '.' + prefix + axis
                d = data[field][s_start:s_end]
                if len(d) == 0:
                    continue
    
                d = numpy.array(d)
                freq  = numpy.fft.rfftfreq(len(d), 1.0 / data[msg+'.rate'])
                # remove mean
                avg = numpy.mean(d)
                d -= avg
                # transform
                d_fft = numpy.fft.rfft(d)
                abs_fft[axis] = numpy.abs(d_fft)
                # remember the max over all axes
                thismax = numpy.max(abs_fft[axis])
                if (max_fft < thismax):
                    max_fft = thismax
                index += 1
                
            for axis in ['X', 'Y', 'Z']:
                # scale to 0dB = max
                field = msg + '.' + prefix + axis
                db_fft = 20 * numpy.log10(abs_fft[axis] / max_fft)
                pylab.plot( freq, db_fft, label=field )
    
            fftwin.canvas.set_window_title(title)
            fftwin.gca().set_ylim(-90, 0)
            pylab.legend(loc='upper right')
            pylab.xlabel('Hz : resolution = ' + '{0:.3f}'.format(f_res))
            pylab.ylabel('dB')
            pylab.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.16)
            fftwin.savefig(msg + '_fft.png')
        
        try:
            selection = raw_input('q to proceed to next file, anything else to select a new range: ')
            print(selection)
        except:
            continue

        if (selection == 'q'): 
            break

pylab.ion()
for filename in args.logs:
    fft(filename)

print('type ctrl-c to close windows and exit')
pylab.ioff()
pylab.show()

