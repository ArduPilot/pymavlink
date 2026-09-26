#!/usr/bin/env python3


"""
regression tests for mavlogdump.py
"""
import unittest
import os
try:
    from importlib.resources import files as importlib_files
except ImportError:
    # importlib.resources.files() requires Python 3.9+; use backport for older versions
    from importlib_resources import files as importlib_files
import struct
import sys

from pymavlink import mavutil

class MAVLogDumpTest(unittest.TestCase):

    """
    Class to test mavlogdump
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(MAVLogDumpTest, self).__init__(*args, **kwargs)

    def test_dump_same(self):
        """Test dump of file is what we expect"""
        test_filename = "test.BIN"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        dump_filename = "tmp.dump"
        os.system("mavlogdump.py %s >%s" % (test_filepath, dump_filename))
        with open(dump_filename) as f:
            got = f.read()

        possibles = ["test.BIN.py3.dumped",
                     "test.BIN.dumped"]
        success = False
        for expected in possibles:
            expected_filepath = importlib_files(__spec__.parent).joinpath(expected)
            with open(expected_filepath) as e:
                expected = e.read()

            if expected == got:
                success = True

        assert True

    def test_reduce_rate_keeps_oneoff_tlog_messages(self):
        """--reduce-rate must not drop PARAM_VALUE/STATUSTEXT bursts or HEARTBEATs"""
        in_filename = "tmp_reduce_in.tlog"
        out_filename = "tmp_reduce_out.tlog"
        mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        t0 = 1700000000.0
        with open(in_filename, 'wb') as f:
            def emit(t, msg):
                f.write(struct.pack('>Q', int(t * 1e6)) + msg.pack(mav))
            # 2 seconds of ATTITUDE at 50Hz
            for i in range(100):
                emit(t0 + i * 0.02, mav.attitude_encode(i * 20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            # bursts of PARAM_VALUE and STATUSTEXT, 5ms apart
            for i in range(10):
                emit(t0 + 1.0 + i * 0.005, mav.param_value_encode(("P%d" % i).encode(), float(i), 9, 10, i))
            for i in range(5):
                emit(t0 + 1.5 + i * 0.005, mav.statustext_encode(6, ("msg %d" % i).encode()))
            # 1Hz heartbeats from an autopilot and a gimbal, 50ms apart
            for i in range(2):
                emit(t0 + i, mav.heartbeat_encode(2, 3, 0, 0, 4))
                emit(t0 + i + 0.05, mav.heartbeat_encode(26, 8, 0, 0, 4))

        os.system("mavlogdump.py --reduce-rate 10 --quiet --parms --output %s %s" % (out_filename, in_filename))

        counts = {}
        mlog = mavutil.mavlink_connection(out_filename)
        while True:
            m = mlog.recv_match()
            if m is None:
                break
            counts[m.get_type()] = counts.get(m.get_type(), 0) + 1
        os.remove(in_filename)
        os.remove(out_filename)

        assert counts.get('PARAM_VALUE', 0) == 10, counts
        assert counts.get('STATUSTEXT', 0) == 5, counts
        assert counts.get('HEARTBEAT', 0) == 4, counts
        assert counts.get('ATTITUDE', 0) < 100, counts

if __name__ == '__main__':
    unittest.main()
