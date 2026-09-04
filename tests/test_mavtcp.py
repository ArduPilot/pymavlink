#!/usr/bin/env python3

"""
test mavutil.mavtcp connect/close/recv behaviour against a real local listener
"""

import errno
import socket
import time
import unittest

from pymavlink import mavutil


class MavtcpTest(unittest.TestCase):
    '''mavutil.mavtcp tests using a real local TCP listener'''

    TIMEOUT = 5.0

    def setUp(self):
        # bind port 0 and read back what the OS assigned, rather than probing
        # for a free port and closing it again - that leaves a window for
        # something else to take the port before we rebind it.
        self.listener = mavutil.mavlink_connection('tcpin:127.0.0.1:0')
        self.port_num = self.listener.listen.getsockname()[1]
        self.conn = mavutil.mavlink_connection('tcp:127.0.0.1:%u' % self.port_num,
                                               autoreconnect=False)

    def tearDown(self):
        self.conn.close()
        self.listener.close()

    def wait_for_listener(self):
        '''drive the listener until it has accepted the connection

        mavtcpin accepts inside recv(), so pumping recv() is how the accept is
        driven through its own API. Its listening socket is non-blocking, so
        this needs to be retried rather than called once.
        '''
        deadline = time.time() + self.TIMEOUT
        while self.listener.port is None and time.time() < deadline:
            self.listener.recv()
            if self.listener.port is None:
                time.sleep(0.01)
        self.assertIsNotNone(self.listener.port,
                             "listener did not accept within %.1fs" % self.TIMEOUT)

    def recv_some(self, conn=None):
        '''read until some data arrives or we run out of time'''
        conn = conn if conn is not None else self.conn
        deadline = time.time() + self.TIMEOUT
        data = b""
        while not data and time.time() < deadline:
            data += conn.recv()
            if not data:
                time.sleep(0.01)
        return data

    def test_connect_sets_port_and_fd(self):
        self.assertIsNotNone(self.conn.port)
        self.assertEqual(self.conn.fd, self.conn.port.fileno())

    def test_close_clears_port(self):
        self.conn.close()
        self.assertIsNone(self.conn.port)
        # self.fd is deliberately not asserted on here: mavtcp does not maintain
        # it across close()/do_connect(), and making that consistent (here and
        # in the other mavfile subclasses) is a separate change.

    def test_close_is_idempotent(self):
        self.conn.close()
        # second call must not raise now that self.port is already None
        self.conn.close()
        self.assertIsNone(self.conn.port)

    def test_recv_after_close_reports_not_connected(self):
        self.conn.close()
        # close() nulls self.port and reconnect() is a no-op with
        # autoreconnect=False, so there is no socket and nothing will reopen
        # one. That is permanent, so it is reported rather than returned as
        # b"" - which recv() uses for "no data right now, retry".
        with self.assertRaises(OSError) as caught:
            self.conn.recv()
        self.assertEqual(caught.exception.errno, errno.ENOTCONN)
        # ENOTCONN keeps errno inspectable; a bare ConnectionError would not
        self.assertNotIsInstance(caught.exception, ConnectionError)

    def test_recv_when_port_is_none(self):
        # exercise the guard directly, without relying on close() being what
        # produced the None. This is a unit test of _ensure_port() itself; the
        # reachable route to this state is close(), covered above.
        self.conn.port.close()
        self.conn.port = None
        with self.assertRaises(OSError) as caught:
            self.conn.recv()
        self.assertEqual(caught.exception.errno, errno.ENOTCONN)

    def test_select_after_close_does_not_raise(self):
        self.conn.close()
        # self.fd is left holding a closed descriptor, so select.select()
        # fails with EBADF; mavfile.select() catches that and reports "not
        # readable" rather than propagating
        self.assertFalse(self.conn.select(0))

    def test_recv_receives_data(self):
        self.wait_for_listener()
        self.listener.write(b"hello")
        self.assertEqual(self.recv_some(), b"hello")

    def test_write_after_close_is_noop(self):
        self.conn.close()
        # must not raise; with autoreconnect=False there is nothing to write to
        self.conn.write(b"test")

    def test_autoreconnect_reestablishes_connection(self):
        '''autoreconnect=True reconnects on recv() after the link is gone'''
        # a plain socket listener rather than mavtcpin: this needs two
        # successive server-side connections, and mavtcpin only tracks one at a
        # time and re-accepts only after its own recv() hits an error. settimeout
        # also makes accept() block up to TIMEOUT, so no polling is needed.
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.addCleanup(listener.close)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', 0))
        listener.listen(2)
        listener.settimeout(self.TIMEOUT)

        conn = mavutil.mavlink_connection(
            'tcp:127.0.0.1:%u' % listener.getsockname()[1], autoreconnect=True)
        self.addCleanup(conn.close)
        (first, _addr) = listener.accept()
        self.addCleanup(first.close)

        # drop the link from under it the way a peer restart would
        conn.port.close()
        conn.port = None

        conn.recv()
        self.assertIsNotNone(conn.port, "recv() should have reconnected")

        (second, _addr) = listener.accept()
        self.addCleanup(second.close)
        second.send(b"again")
        self.assertEqual(self.recv_some(conn), b"again")


if __name__ == '__main__':
    unittest.main()
