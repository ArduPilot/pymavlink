#!/usr/bin/env python

'''
connect to a mav and log received messages
'''
from __future__ import print_function

from argparse import ArgumentParser
from pymavlink import mavutil
from datetime import datetime

parser = ArgumentParser()
parser.add_argument("connection", metavar="connection string", nargs=1)
args = parser.parse_args()


def process(connection):
    '''write a log for a connection'''
    
    print("connecting to {}".format(connection))
    
    conn = mavutil.mavlink_connection(connection)
    
    conn.setup_logfile("my_log.tlog")
    conn.setup_logfile_raw("my_log.tlog.raw")

    while True:
        try:
            msg = conn.recv_msg()
            
            if msg is not None:
                print(datetime.now().strftime('%H:%M:%S'), msg)
        except KeyboardInterrupt:
            conn.close()
            print("\nConnection closed.")
            break



process(args.connection[0])
