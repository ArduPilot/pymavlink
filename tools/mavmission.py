#!/usr/bin/env python3

'''
extract mavlink mission from log

AP_FLAKE8_CLEAN
'''
from pymavlink import mavutil
from pymavlink import mavwp

class MAVMission():
    def __init__(self, filepath : str) -> None:
        self.filepath : str = filepath

    def run(self) -> mavwp.MAVWPLoader:
        '''parses the log, returns a MAVWPLoader primed with content'''
        mlog = mavutil.mavlink_connection(self.filepath)

        wp = mavwp.MAVWPLoader()

        num_wps = None
        while True:
            m = mlog.recv_match(type=set([
                'MISSION_ITEM',
                'CMD',
                'WAYPOINT',
                'MISSION_ITEM_INT',
            ]))
            if m is None:
                break
            if m.get_type() == 'CMD':
                try:
                    frame = m.Frame
                except AttributeError:
                    print("Warning: assuming frame is GLOBAL_RELATIVE_ALT")
                    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                num_wps = m.CTot
                m = mavutil.mavlink.MAVLink_mission_item_message(
                    0,
                    0,
                    m.CNum,
                    frame,
                    m.CId,
                    0, 1,
                    m.Prm1, m.Prm2, m.Prm3, m.Prm4,
                    m.Lat, m.Lng, m.Alt
                )

            if m.get_type() == 'MISSION_ITEM_INT':
                m = mavutil.mavlink.MAVLink_mission_item_message(
                    m.target_system,
                    m.target_component,
                    m.seq,
                    m.frame,
                    m.command,
                    m.current,
                    m.autocontinue,
                    m.param1,
                    m.param2,
                    m.param3,
                    m.param4,
                    m.x*1.0e-7,
                    m.y*1.0e-7,
                    m.z,
                )
            if m.current >= 2:
                continue

            while m.seq > wp.count():
                print("Adding dummy WP %u" % wp.count())
                wp.set(m, wp.count())
            wp.set(m, m.seq)

        if num_wps is not None:
            while wp.count() > num_wps:
                wp.remove(wp.wp(wp.count() - 1))

        return wp

def main() -> None:
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--output", default='mission.txt', help="output file")
    parser.add_argument("log", metavar="LOG")

    args = parser.parse_args()

    mm = MAVMission(args.log)
    wp = mm.run()

    wp.save(args.output)
    print("Saved %u waypoints to %s" % (wp.count(), args.output))

if __name__ == '__main__':
    main()
