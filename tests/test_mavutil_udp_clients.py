#!/usr/bin/env python3

"""Tests for bounded UDP server client tracking."""

import socket
import time
import unittest

from pymavlink import mavutil


class UDPClientTrackingTest(unittest.TestCase):
    def receive_message(self, server):
        for _ in range(100):
            message = server.recv_msg()
            if message is not None:
                return message
            time.sleep(0.01)
        self.fail("UDP server did not receive the test message")

    def send_heartbeat(self, client):
        client.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
            3,
        )

    def receive_raw(self, client):
        for _ in range(100):
            try:
                return client.port.recvfrom(16)[0]
            except BlockingIOError:
                time.sleep(0.01)
        self.fail("UDP client did not receive the server reply")

    def test_rejects_invalid_client_limit(self):
        with self.assertRaises(ValueError):
            mavutil.mavudp("127.0.0.1:0", max_clients=0)

    def test_mavlink_connection_forwards_client_limit(self):
        server = mavutil.mavlink_connection(
            "udpin:127.0.0.1:0", udp_max_clients=3
        )
        try:
            self.assertEqual(server.max_clients, 3)
        finally:
            server.close()

    def test_legacy_udp_forms_forward_client_limit(self):
        for device in ("udp:127.0.0.1:0", "127.0.0.1:0"):
            server = mavutil.mavlink_connection(
                device, input=True, udp_max_clients=3
            )
            try:
                self.assertEqual(server.max_clients, 3)
            finally:
                server.close()

    def test_invalid_datagram_is_not_recorded(self):
        server = mavutil.mavudp("127.0.0.1:0", max_clients=2)
        attacker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            attacker.sendto(b"X", server.port.getsockname())
            self.assertEqual(self.receive_message(server).get_type(), "BAD_DATA")
            self.assertEqual(server.clients, set())
            self.assertEqual(server.clients_last_alive, {})
        finally:
            attacker.close()
            server.close()

    def test_new_sources_are_bounded(self):
        server = mavutil.mavudp("127.0.0.1:0", max_clients=2)
        clients = []
        try:
            destination = server.port.getsockname()
            for _ in range(3):
                client = mavutil.mavudp(
                    f"{destination[0]}:{destination[1]}", input=False
                )
                clients.append(client)
                self.send_heartbeat(client)
                self.assertEqual(self.receive_message(server).get_type(), "HEARTBEAT")

            newest_port = clients[-1].port.getsockname()[1]
            self.assertEqual(len(server.clients), 2)
            self.assertEqual(len(server.clients_last_alive), 2)
            self.assertIn(newest_port, {address[1] for address in server.clients})
        finally:
            for client in clients:
                client.close()
            server.close()

    def test_valid_clients_receive_server_writes(self):
        server = mavutil.mavudp("127.0.0.1:0", max_clients=2)
        clients = []
        try:
            destination = server.port.getsockname()
            for _ in range(2):
                client = mavutil.mavudp(
                    f"{destination[0]}:{destination[1]}", input=False
                )
                clients.append(client)
                self.send_heartbeat(client)
                self.assertEqual(self.receive_message(server).get_type(), "HEARTBEAT")

            server.write(b"Y")
            self.assertEqual([self.receive_raw(client) for client in clients], [b"Y", b"Y"])
        finally:
            for client in clients:
                client.close()
            server.close()


if __name__ == "__main__":
    unittest.main()
