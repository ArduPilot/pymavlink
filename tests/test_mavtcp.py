#!/usr/bin/env python3

import errno
import unittest
from pathlib import Path
import sys
import importlib.util

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    repo_root = Path(__file__).resolve().parents[1]
    pkg_spec = importlib.util.spec_from_file_location(
        "pymavlink",
        repo_root / "__init__.py",
        submodule_search_locations=[str(repo_root)],
    )
    assert pkg_spec is not None and pkg_spec.loader is not None
    pymavlink_pkg = importlib.util.module_from_spec(pkg_spec)
    sys.modules["pymavlink"] = pymavlink_pkg
    pkg_spec.loader.exec_module(pymavlink_pkg)
    from pymavlink import mavutil


class DummyMav:
    @staticmethod
    def bytes_needed():
        return 1


class FakeSocket:
    def __init__(self, recv_data=b"", recv_error=None):
        self._recv_data = recv_data
        self._recv_error = recv_error
        self.closed = False
        self.sent = []

    def recv(self, _n):
        if self._recv_error is not None:
            raise self._recv_error
        return self._recv_data

    def send(self, buf):
        self.sent.append(buf)
        return len(buf)

    def close(self):
        self.closed = True


def build_mavtcp(port, autoreconnect=False):
    tcp = object.__new__(mavutil.mavtcp)
    tcp.port = port
    tcp.autoreconnect = autoreconnect
    tcp.mav = DummyMav()
    return tcp


class TestMavTcpDisconnectHandling(unittest.TestCase):
    def test_recv_raises_connection_error_when_disconnected_without_autoreconnect(self):
        tcp = build_mavtcp(port=None, autoreconnect=False)

        with self.assertRaises(ConnectionError):
            tcp.recv()

    def test_eof_marks_socket_disconnected(self):
        sock = FakeSocket(recv_data=b"")
        tcp = build_mavtcp(port=sock, autoreconnect=False)

        data = tcp.recv(1)

        self.assertEqual(b"", data)
        self.assertTrue(sock.closed)
        self.assertIsNone(tcp.port)

        with self.assertRaises(ConnectionError):
            tcp.recv(1)

    def test_connection_reset_marks_socket_disconnected(self):
        sock = FakeSocket(recv_error=OSError(errno.ECONNRESET, "connection reset"))
        tcp = build_mavtcp(port=sock, autoreconnect=False)

        with self.assertRaises(OSError):
            tcp.recv(1)

        self.assertTrue(sock.closed)
        self.assertIsNone(tcp.port)


if __name__ == "__main__":
    unittest.main()
