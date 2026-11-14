#!/usr/bin/env python3

'''
Take off an ArduCopter to a given altitude in GUIDED mode, hover, then land.

Shows the sequence a copter needs before it will take off from a script:
wait for a position estimate, switch to GUIDED, arm, send
MAV_CMD_NAV_TAKEOFF and watch GLOBAL_POSITION_INT until the climb is done.

Try it against SITL first:

    sim_vehicle.py -v ArduCopter
    python3 takeoff_land.py --connection udp:127.0.0.1:14550 --altitude 5

On a real vehicle fly outdoors with a GPS lock and keep an RC transmitter
ready to take over. If the script fails while armed it switches to LAND.
'''

import time
from argparse import ArgumentParser

from pymavlink import mavutil


def connect(address, baudrate, timeout=30):
    '''connect and wait for a heartbeat from the autopilot'''
    print("Connecting to %s" % address)
    master = mavutil.mavlink_connection(address, baud=baudrate)
    deadline = time.time() + timeout
    while time.time() < deadline:
        # ignore heartbeats from ground stations or cameras on the same link
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            master.target_system = msg.get_srcSystem()
            master.target_component = msg.get_srcComponent()
            print("Heartbeat from system %u component %u" %
                  (master.target_system, master.target_component))
            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            return master
    raise TimeoutError("No autopilot heartbeat from %s" % address)


def show_statustext(msg):
    if msg is not None and msg.get_type() == 'STATUSTEXT':
        print("  autopilot: %s" % msg.text)


def latest(master, msg_type, timeout=5):
    '''return the newest message of a type

    Telemetry arrives faster than this script reads it, so drain anything
    already queued before waiting for the next message.
    '''
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        show_statustext(msg)
    msg = master.recv_match(type=msg_type, blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError("No %s received" % msg_type)
    return msg


def autopilot_heartbeat(master, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1)
        show_statustext(msg)
        if (msg is not None and msg.get_type() == 'HEARTBEAT' and
                msg.get_srcSystem() == master.target_system and
                msg.get_srcComponent() == master.target_component):
            return msg
    raise TimeoutError("Lost heartbeat from the autopilot")


def is_armed(master):
    msg = autopilot_heartbeat(master)
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def send_command(master, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, timeout=5):
    '''send a COMMAND_LONG, return True if it was accepted'''
    master.mav.command_long_send(
        master.target_system, master.target_component,
        command, 0, p1, p2, p3, p4, p5, p6, p7)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=1)
        show_statustext(msg)
        if msg is not None and msg.get_type() == 'COMMAND_ACK' and msg.command == command:
            return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def wait_position_estimate(master, timeout=120):
    '''GUIDED flight needs a position estimate (GPS lock and EKF ready)'''
    print("Waiting for position estimate")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=['EKF_STATUS_REPORT', 'STATUSTEXT'], blocking=True, timeout=2)
        show_statustext(msg)
        if (msg is not None and msg.get_type() == 'EKF_STATUS_REPORT' and
                msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS):
            return
    raise TimeoutError("No position estimate")


def set_mode(master, mode, timeout=30):
    mode_id = master.mode_mapping()[mode]
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.set_mode(mode_id)
        if autopilot_heartbeat(master).custom_mode == mode_id:
            print("Mode %s" % mode)
            return
    raise TimeoutError("Could not change mode to %s" % mode)


def arm(master, timeout=60):
    print("Arming")
    deadline = time.time() + timeout
    while time.time() < deadline:
        # retried because pre-arm checks can take a while to pass
        if send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, timeout=3):
            while not is_armed(master):
                pass
            print("Armed")
            return
        time.sleep(2)
    raise TimeoutError("Arming was refused")


def takeoff(master, altitude, timeout=60):
    print("Taking off to %.1fm" % altitude)
    if not send_command(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=altitude):
        raise RuntimeError("Takeoff was rejected")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = latest(master, 'GLOBAL_POSITION_INT')
        alt = msg.relative_alt * 0.001
        print("  altitude %.1fm" % alt)
        # wait for the climb to finish, not just to pass the altitude
        if abs(alt - altitude) < 0.3 and abs(msg.vz) < 10:
            return
        time.sleep(0.5)
    raise TimeoutError("Did not reach takeoff altitude")


def land(master, timeout=120):
    set_mode(master, 'LAND')
    print("Landing")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not is_armed(master):
            print("Landed and disarmed")
            return
        time.sleep(1)
    raise TimeoutError("Landing did not complete")


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udp:127.0.0.1:14550",
                        help="MAVLink connection string, e.g. /dev/ttyACM0 or tcp:127.0.0.1:5760")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--altitude", type=float, default=3.0, help="takeoff altitude in metres")
    parser.add_argument("--hover", type=float, default=5.0, help="seconds to hover before landing")
    args = parser.parse_args()

    master = connect(args.connection, args.baudrate)
    try:
        wait_position_estimate(master)
        set_mode(master, 'GUIDED')
        arm(master)
        takeoff(master, args.altitude)
        print("Hovering for %.0fs" % args.hover)
        time.sleep(args.hover)
        land(master)
    finally:
        try:
            if is_armed(master):
                print("Still armed, switching to LAND")
                land(master)
        except Exception as ex:
            print("Could not confirm landing (%s), take manual control" % ex)


if __name__ == '__main__':
    main()
