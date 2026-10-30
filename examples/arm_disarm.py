#!/usr/bin/env python3

'''
Arm a vehicle, hold for a few seconds, then disarm.

Shows how to send MAV_CMD_COMPONENT_ARM_DISARM, check the COMMAND_ACK and
confirm the armed state from the autopilot's HEARTBEAT. STATUSTEXT messages
are printed so a failed pre-arm check explains itself.

The vehicle is armed in whatever mode it is already in. REMOVE THE
PROPELLERS before running this against a real vehicle.

    python3 arm_disarm.py --connection /dev/ttyACM0
    python3 arm_disarm.py --connection tcp:127.0.0.1:5760
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
            return master
    raise TimeoutError("No autopilot heartbeat from %s" % address)


def is_armed(master, timeout=5):
    '''armed state from the next autopilot heartbeat'''
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            print("  autopilot: %s" % msg.text)
        elif (msg.get_srcSystem() == master.target_system and
              msg.get_srcComponent() == master.target_component):
            return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    raise TimeoutError("Lost heartbeat from the autopilot")


def arm_disarm(master, arm, timeout=5):
    '''send the arm (True) or disarm (False) command, return True if accepted'''
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if arm else 0, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            print("  autopilot: %s" % msg.text)
        elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def wait_armed(master, armed, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if is_armed(master) == armed:
            return
    raise TimeoutError("Vehicle did not %s" % ("arm" if armed else "disarm"))


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udp:127.0.0.1:14550",
                        help="MAVLink connection string, e.g. /dev/ttyACM0 or tcp:127.0.0.1:5760")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--hold", type=float, default=5.0, help="seconds to stay armed")
    parser.add_argument("--retries", type=int, default=15,
                        help="arm attempts, 2 seconds apart, while pre-arm checks complete")
    args = parser.parse_args()

    master = connect(args.connection, args.baudrate)

    print("Arming")
    for attempt in range(args.retries):
        if arm_disarm(master, True):
            break
        time.sleep(2)
    else:
        raise SystemExit("Arming was refused, see the autopilot messages above")
    wait_armed(master, True)
    print("Armed")

    # the autopilot may disarm by itself first if left idle (DISARM_DELAY)
    end = time.time() + args.hold
    while time.time() < end and is_armed(master):
        pass

    print("Disarming")
    arm_disarm(master, False)
    wait_armed(master, False)
    print("Disarmed")


if __name__ == '__main__':
    main()
