#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

import os
import socket
import tempfile
import threading
import unittest

from pymavlink import mavutil


class TestUnixDomainSocket(unittest.TestCase):
    def test_stream_transport(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            socket_path = os.path.join(temporary_directory, "mavlink.sock")
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            listener.bind(socket_path)
            listener.listen(1)

            received = []

            def exchange_data():
                connection, _ = listener.accept()
                try:
                    received.append(connection.recv(7))
                    connection.sendall(b"response")
                finally:
                    connection.close()

            server = threading.Thread(target=exchange_data, daemon=True)
            server.start()
            mav = mavutil.mavlink_connection("uds:" + socket_path)
            try:
                mav.write(b"request")
                self.assertTrue(mav.select(1))
                self.assertEqual(mav.recv(8), b"response")
            finally:
                mav.close()
                server.join(1)
                listener.close()

            self.assertFalse(server.is_alive())
            self.assertEqual(received, [b"request"])

    def test_unix_alias(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            socket_path = os.path.join(temporary_directory, "mavlink.sock")
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            listener.bind(socket_path)
            listener.listen(1)

            mav = mavutil.mavlink_connection("unix:" + socket_path)
            connection, _ = listener.accept()
            try:
                self.assertIsInstance(mav, mavutil.mavuds)
            finally:
                mav.close()
                connection.close()
                listener.close()

    def test_reconnect(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            socket_path = os.path.join(temporary_directory, "mavlink.sock")
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            listener.bind(socket_path)
            listener.listen(1)
            replacement_ready = threading.Event()

            def replace_server():
                connection, _ = listener.accept()
                connection.close()
                listener.close()
                os.unlink(socket_path)

                replacement = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                replacement.bind(socket_path)
                replacement.listen(1)
                replacement_ready.set()
                connection, _ = replacement.accept()
                try:
                    connection.sendall(b"reconnected")
                finally:
                    connection.close()
                    replacement.close()

            server = threading.Thread(target=replace_server, daemon=True)
            server.start()
            mav = mavutil.mavlink_connection("uds:" + socket_path, autoreconnect=True)
            try:
                self.assertTrue(replacement_ready.wait(1))
                self.assertTrue(mav.select(1))
                self.assertEqual(mav.recv(1), b"")
                self.assertTrue(mav.select(1))
                self.assertEqual(mav.recv(11), b"reconnected")
            finally:
                mav.close()
                server.join(1)
            self.assertFalse(server.is_alive())


class TestMulticast(unittest.TestCase):
    # a port of our own, so as not to talk to a SITL on the usual one
    DEVICE = 'mcast:239.255.145.50:14659:127.0.0.1'

    def test_interface(self):
        a = mavutil.mavlink_connection(self.DEVICE, source_system=1)
        b = mavutil.mavlink_connection(self.DEVICE, source_system=2)
        try:
            # both directions stay on loopback: the sender's address is ours
            self.assertEqual(a.port_out.getsockname()[0], '127.0.0.1')
            a.mav.heartbeat_send(0, 0, 0, 0, 0)
            b.mav.heartbeat_send(0, 0, 0, 0, 0)
            got_a = a.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            got_b = b.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            # and each one ignores what it sent itself
            self.assertIsNotNone(got_a)
            self.assertEqual(got_a.get_srcSystem(), 2)
            self.assertIsNotNone(got_b)
            self.assertEqual(got_b.get_srcSystem(), 1)
        finally:
            a.close()
            b.close()


if __name__ == '__main__':
    unittest.main()
