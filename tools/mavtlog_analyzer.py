#!/usr/bin/env python

'''
Automatically analyze flight from telemetry log
Execution example -> python mavtlog_analyzer.py flight.tlog
'''
from __future__ import print_function

from pymavlink import mavutil
import time

#args
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()

#analyze contents
check_list = ["VIBRATION", "EKF_STATUS_REPORT"]
vibration_max = [0, 0, 0]
vibration_res = []
vibration_field = ['time_usec', 'vibration_x', 'vibration_y', 'vibration_z', 'clipping_0', 'clipping_1', 'clipping_2']
ekf_status_max = [0, 0, 0, 0, 0]
ekf_status_res = []
ekf_status_field = ['flags', 'velocity_variance', 'pos_horiz_variance', 'pos_vert_variance', 'compass_variance', 'terrain_alt_variance']

#analyze telemetry log
def analyze(filename):
    mlog = mavutil.mavlink_connection(filename)#get log

    while True:
        try:#Analyze line by line (Analysis object only)
            m = mlog.recv_match(type=check_list)
            if m is None:#EOF
                eval_logs()
                return
        except Exception as e:
            print(e)
            return
            
        #check fields value
        if m.get_type() == check_list[0]:  #check vibration
            input_vibration(m)
        elif m.get_type() == check_list[1]:#check EKF status
            input_ekf_status(m)

#input vibration
def input_vibration(msg):
    for i in range(len(vibration_max)):
        value = msg.format_attr(vibration_field[i + 1])
        if value > vibration_max[i]:#update max value
            vibration_max[i] = value

#input EKF status
def input_ekf_status(msg):
    fields = msg.get_fieldnames()
    for i in range(len(ekf_status_max)):
        value = msg.format_attr(ekf_status_field[i + 1])
        if value > ekf_status_max[i]:#update maximum value
            ekf_status_max[i] = value
            
#Evaluate log information
def eval_logs():
    #vibration
    judge(vibration_max,30,60,vibration_res)
    #EKF status
    judge(ekf_status_max,0.5,0.8,ekf_status_res)

#Judge warnings and errors
def judge(data,warn_threshold,error_threshold,res):
    for i in range(len(data)):
        if data[i] < warn_threshold:
            res.append("SAFE")
        elif data[i] < error_threshold:
            res.append("WARN")
        else:
            res.append("ERROR")

#output the result
def output_result():
    print("[VIBRATION]")
    for i in range(len(vibration_res)):
        print("    %s:%s (max:%.1f)" % (vibration_field[i + 1], vibration_res[i], vibration_max[i]))
    print("[EKF status]")
    for i in range(len(ekf_status_res)):
        print("    %s:%s (max:%.3f)" % (ekf_status_field[i + 1], ekf_status_res[i], ekf_status_max[i]))


filename = args.logs[0]   #get filename
analyze(filename)         #start analyze
output_result()           #output result
