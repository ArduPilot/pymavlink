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


class TestTCPInput(unittest.TestCase):
    def test_eof_returns_to_listener_and_accepts_next_client(self):
        mav = mavutil.mavtcpin("127.0.0.1:0")
        address = mav.listen.getsockname()
        first_client = socket.create_connection(address)
        second_client = None
        try:
            # Accept a client that does not send any MAVLink data.  It remains
            # connected until it closes; mavtcpin must not impose a protocol
            # timeout on an otherwise idle stream.
            mav.recv()
            self.assertIsNotNone(mav.port)

            first_client.close()
            first_client = None
            self.assertTrue(mav.select(1))
            self.assertEqual(mav.recv(), b"")
            self.assertIsNone(mav.port)
            self.assertEqual(mav.fd, mav.listen.fileno())

            second_client = socket.create_connection(address)
            self.assertTrue(mav.select(1))
            mav.recv()
            self.assertIsNotNone(mav.port)
        finally:
            if first_client is not None:
                first_client.close()
            if second_client is not None:
                second_client.close()
            mav.close()


if __name__ == '__main__':
    unittest.main()
