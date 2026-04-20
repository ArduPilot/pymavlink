#!/usr/bin/python3

"""
A webserver for GPS updates.

This code may be useful in situations where a companion computer is connected
to the ArduPilot board and is able to run mavproxy with an output on the IP and
port as defined in the runtime arguments.  The expected use is that the output
of an external GPS is transmitted over http to the companion as a proxy for the
ArduPilot connection.

A typical set of ArduPilot parameters for using this code:
    EK3_SRC1_POSXY == 3  (GPS)
    EK3_SRC1_POSZ  == 3  (GPS)
    EK3_SRC1_VELXY == 3  (GPS)
    EK3_SRC1_VELZ  == 3  (GPS)
    EK3_SRC1_YAW   == 3  (GPS with compass fallback)
    GPS_TYPE       == 14 (MAV)

Example syntax for running this code:
    python3 ./gps_web.py -m 127.0.0.1 -s 127.0.0.1 -d

Example SITL syntax:
    python3 ./sim_vehicle.py -v ArduCopter --console --map --out 127.0.0.1:9898

An example of the expected type of input to the webserver:
    'GPS_ms':'236755470'                                                        ## integer
    'GPS_week':'2272'                                                           ## integer
    'Fix':'5'                                                                   ## integer
    'Lat':'45.60451248'                                                         ## float
    'Lon':'-122.63580536'                                                       ## float
    'Alt':'0.0000'                                                              ## float
    'HDOP':'0.10'                                                               ## float
    'VDOP':'0.10'                                                               ## float
    'SpeedNorth':'0.25'                                                         ## float
    'SpeedEast':'-1.91'                                                         ## float
    'Heading':'92.806'                                                          ## float

Message testing with curl:
    ** Linux
    curl -X POST http://127.0.0.1:10000 -H 'Content-Type: application/json' -i -d '{"GPS_ms":"236755470","GPS_week":"2272","Fix":"5","Lat":"-35.36609329","Lon":"149.17195595","Alt":"0.0000","HDOP":"0.10","VDOP":"0.10","SpeedNorth":"0.25","SpeedEast":"-1.91","Heading":"92.806"}'

    ** Windows
    curl -X POST http://127.0.0.1:10000 -H 'Content-Type: application/json' -i -d {'GPS_ms':'236755470','GPS_week':'2272','Fix':'5','Lat':'45.60451248','Lon':'-122.63580536','Alt':'0.0000','HDOP':'0.10','VDOP':'0.10','SpeedNorth':'0.25','SpeedEast':'-1.91','Heading':'92.806'}

Released under the GNU GPL version 3 by stryngs
"""

import argparse
import ast
import psutil
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from io import BytesIO
from pymavlink import mavutil, mavwp
from queue import Queue

class Heartbeat():
    """Setup and hold the heartbeat"""
    __slots__ = ('heartBeat')
    def __init__(self, parent):
        self.heartBeat = threading.Thread(target = parent.heartBeat)
        self.heartBeat.daemon = True
        self.heartBeat.start()


class Webapp():
    """Provide an input"""
    __slots__ = ('webApp')
    def __init__(self, parent):
        self.webApp = threading.Thread(target = parent.webApp)
        self.webApp.daemon = True
        self.webApp.start()


class Gpsloop():
    """Launch GPS"""
    __slots__ = ('gpsLoop')
    def __init__(self, parent):
        self.gpsLoop = threading.Thread(target = parent.gpsLoop)
        self.gpsLoop.daemon = True
        self.gpsLoop.start()
        print('[~] Waiting for GPS')


class httpHandler(BaseHTTPRequestHandler):
    """A threaded handler for web requests"""

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        global gList_

        try:
            coord = ast.literal_eval(body.decode())

            ## gps
            try:
                lt = coord.get('Lat').split('.')
                theLat = int(lt[0] + lt[1][:7].ljust(7, '0'))
                ln = coord.get('Lon').split('.')
                theLon = int(ln[0] + ln[1][:7].ljust(7, '0'))
                hd = coord.get('Heading').split('.')
                theHdg = int(hd[0] + hd[1][:2].ljust(2, '0'))

                ## Update the gps list
                gList_.append((int(psutil.boot_time()),                         # Timestamp (micros since boot or Unix epoch)
                               0,                                               # ID of the GPS for multiple GPS inputs
                               0,                                               # (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
                               int(coord.get('GPS_ms')),                        # GPS time (milliseconds from start of GPS week)
                               int(coord.get('GPS_week')),                      # GPS week number
                               int(coord.get('Fix')),                           # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
                               theLat,                                          # Latitude (WGS84), in degrees * 1E7
                               theLon,                                          # Longitude (WGS84), in degrees * 1E7
                               float(coord.get('Alt')),                         # Altitude (AMSL, not WGS84), in m (positive for up)
                               float(coord.get('HDOP')),                        # GPS HDOP horizontal dilution of position in m
                               float(coord.get('VDOP')),                        # GPS VDOP vertical dilution of position in m
                               float(coord.get('SpeedNorth')),                  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
                               float(coord.get('SpeedEast')),                   # GPS velocity in m/s in EAST direction in earth-fixed NED frame
                               0,                                               # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
                               .1,                                              # GPS speed accuracy in m/s
                               .1,                                              # GPS horizontal accuracy in m
                               .1,                                              # GPS vertical accuracy in m
                               10,                                              # Number of satellites visible.
                               theHdg))                                         # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
                self.send_response(200)
                self.end_headers()
                response = BytesIO()
                response.write(body)
                response.write(b'\n')
                self.wfile.write(response.getvalue())
            except Exception as E:
                print('Responding with a 500 - inner try')
                self.send_response(500)
                self.end_headers()
                response = BytesIO()
                self.wfile.write(response.getvalue())
                print(E)
        except Exception as E:
            print('Responding with a 500 - outer try')
            self.send_response(500)
            self.end_headers()
            response = BytesIO()
            self.wfile.write(response.getvalue())
            print(E)


class Parent(object):
    """Parent class responsible for user interactions and backgrounding"""

    def __init__(self, args):
        self.args = args
        self.gList = []
        if self.args.mport is None:
            mport = 9898
        else:
            mport = self.args.mport
        print('[~] Waiting on MAVLink')
        self.m = mavutil.mavlink_connection('udpin:%s:%s' % (self.args.m, mport))
        self.m.wait_heartbeat(blocking = True)
        print('[~] MAVLink established')

    def gpsLoop(self):
        """
        Loops on self.gList and looks for a new object.  pop(0) approach.
        If no object, sleep @ .001
        """
        print('[~] Creating GPS input loop')
        while True:
            try:
                if len(self.gList) > 0:

                    ## Values from POST
                    gpsT = self.gList.pop(0)
                    self.m.mav.gps_input_send(gpsT[0],
                                              gpsT[1],
                                              gpsT[2],
                                              gpsT[3],
                                              gpsT[4],
                                              gpsT[5],
                                              gpsT[6],
                                              gpsT[7],
                                              gpsT[8],
                                              gpsT[9],
                                              gpsT[10],
                                              gpsT[11],
                                              gpsT[12],
                                              gpsT[13],
                                              gpsT[14],
                                              gpsT[15],
                                              gpsT[16],
                                              gpsT[17],
                                              gpsT[18])
            except Exception as E:
                print(E)
            time.sleep(.001)

    def heartBeat(self):
        """Drains the queue for accuracy"""
        while True:
            self.m.recv_msg()
            time.sleep(.001)

    def webApp(self):
        """Run a webapp to handle simple post requests"""
        print('[~] Launching webapp')
        if self.args.wport is None:
            wport = 10000
        else:
            wport = int(self.args.wport)
        self.server = ThreadingHTTPServer((self.args.s, wport), httpHandler)
        self.server.serve_forever()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', action = 'store_true', help = 'Do not background')
    parser.add_argument('-m', help = 'Mavlink IP', required = True)
    parser.add_argument('-s', help = 'Webserver IP', required = True)
    parser.add_argument('--mport', help = 'Mavlink port [Default is 9898]')
    parser.add_argument('--wport', help = 'Webserver port [Default is 10000]')

    args = parser.parse_args()

    ## Grab parent obj
    f = Parent(args)
    gList_ = f.gList
    Heartbeat(f)
    Webapp(f)
    Gpsloop(f)

    ## Acquire current GPS location and set as home
    homeLoc = f.m.location()
    f.homeLat = homeLoc.lat
    f.homeLon = homeLoc.lng
    print('[~] GPS acquired')

    ## Server mode
    if args.d is True:
        print('[~] Server mode launched')
        while True:
            time.sleep(1000)
