#!/usr/bin/env python3

"""
test that log readers and mavlink connections work as context managers
"""

import os
import unittest

from pymavlink import DFReader
from pymavlink import mavutil


class ContextManagerTest(unittest.TestCase):
    '''with-statement support releases file handles and sockets on exit'''

    def dataflash_path(self):
        return os.path.join(os.path.dirname(__file__), 'test.BIN')

    def test_dfreader_binary(self):
        with DFReader.DFReader_binary(self.dataflash_path()) as dflog:
            self.assertIsNotNone(dflog.recv_msg())
        self.assertTrue(dflog.filehandle.closed)
        self.assertTrue(dflog.data_map.closed)

    def test_mavlink_connection_dataflash(self):
        with mavutil.mavlink_connection(self.dataflash_path()) as mlog:
            self.assertIsNotNone(mlog.recv_msg())
        self.assertTrue(mlog.filehandle.closed)

    def test_mavfile(self):
        with mavutil.mavlink_connection('udpout:127.0.0.1:14550') as conn:
            self.assertNotEqual(conn.port.fileno(), -1)
        self.assertEqual(conn.port.fileno(), -1)


if __name__ == '__main__':
    unittest.main()
