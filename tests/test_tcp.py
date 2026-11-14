#!/usr/bin/env python3

'''TCP link tests for mavtcp reconnect and peer-close behaviour'''

import io
import socket
import threading
import time
import unittest
from contextlib import redirect_stdout

from pymavlink import mavutil


class TcpPeer(object):
    '''local TCP server that accepts one client at a time'''

    def __init__(self, port=0):
        self.listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen.bind(("127.0.0.1", port))
        self.listen.listen(1)
        # accept() has to notice close() from the test thread. A short timeout
        # beats depending on close() waking a blocked accept().
        self.listen.settimeout(0.2)
        self.port = self.listen.getsockname()[1]
        self.client = None
        self._accepting = True
        self.thread = threading.Thread(target=self._accept_loop)
        self.thread.daemon = True
        self.thread.start()

    def _accept_loop(self):
        while self._accepting:
            try:
                client, _ = self.listen.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.client = client

    def wait_for_client(self, timeout=2.0):
        deadline = time.time() + timeout
        while self.client is None and time.time() < deadline:
            time.sleep(0.01)
        if self.client is None:
            raise RuntimeError("timed out waiting for a TCP client")
        return self.client

    def close(self):
        self._accepting = False
        client = self.client
        self.client = None
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()
        try:
            self.listen.close()
        except OSError:
            pass
        self.thread.join(timeout=2)


class TcpLinkTest(unittest.TestCase):
    '''mavtcp must report one EOF and be able to reconnect'''

    def setUp(self):
        self.server = TcpPeer()
        self.addCleanup(self.server.close)
        self.conn = None

    def tearDown(self):
        if self.conn is not None:
            self.conn.close()

    def _connect(self, autoreconnect):
        self.conn = mavutil.mavlink_connection(
            "tcp:127.0.0.1:%d" % self.server.port,
            autoreconnect=autoreconnect,
            retries=1,
            source_system=1,
        )
        self.server.wait_for_client()
        return self.conn

    def test_peer_close_raises_once_without_autoreconnect(self):
        conn = self._connect(autoreconnect=False)
        self.server.close()

        output = io.StringIO()
        with redirect_stdout(output):
            with self.assertRaises(EOFError):
                conn.recv()
            with self.assertRaises(EOFError):
                conn.recv()

        self.assertEqual(output.getvalue().count("EOF on TCP socket"), 1)
        self.assertIsNone(conn.port)
        self.assertIsNone(conn.fd)

        # The closed socket must not stay readable. select() sleeps when fd is None.
        started = time.monotonic()
        conn.select(0.3)
        self.assertGreaterEqual(time.monotonic() - started, 0.2)

    def test_autoreconnect_after_server_restarts(self):
        conn = self._connect(autoreconnect=True)
        conn._reconnect_interval = 0.1
        port = self.server.port
        self.server.close()

        output = io.StringIO()
        started = time.monotonic()
        with redirect_stdout(output):
            for _ in range(20):
                self.assertEqual(conn.recv(), b"")
        self.assertLess(time.monotonic() - started, 0.5)
        self.assertEqual(output.getvalue().count("EOF on TCP socket"), 1)
        self.assertEqual(output.getvalue().count("Attempting reconnect"), 1)
        self.assertIsNone(conn.port)
        self.assertIsNone(conn.fd)

        restarted = TcpPeer(port)
        self.addCleanup(restarted.close)
        deadline = time.time() + 3
        with redirect_stdout(io.StringIO()):
            while conn.port is None and time.time() < deadline:
                self.assertEqual(conn.recv(), b"")
                time.sleep(0.02)
        self.assertIsNotNone(conn.port)
        self.assertEqual(conn.fd, conn.port.fileno())

        client = restarted.wait_for_client()
        client.settimeout(2)
        conn.write(b"ping")
        self.assertEqual(client.recv(16), b"ping")
        client.sendall(b"pong")

        data = b""
        deadline = time.time() + 2
        while data != b"pong" and time.time() < deadline:
            chunk = conn.recv(16)
            if chunk:
                data += chunk
            else:
                time.sleep(0.01)
        self.assertEqual(data, b"pong")


if __name__ == "__main__":
    unittest.main()
