#!/usr/bin/env python3

"""
BDD integration scenarios for MAVFTP over a real loopback UDP link.

The unit scenarios in ``test_mavftp.py`` use an in-memory transport so that
the FTP state machine can be tested deterministically.  These scenarios keep
the transport real and verify MAVLink framing, UDP delivery, and the FTP
request/reply exchange together.

SPDX-FileCopyrightText: 2024-2026 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
"""

# The raw packet decoder intentionally mirrors the unit-test helper.
# pylint: disable=duplicate-code

import struct
import threading
import unittest

from pymavlink import mavutil
from pymavlink.mavftp import (
    FTP_OP,
    MAVFTP,
    FtpError,
    OP_Ack,
    OP_BurstReadFile,
    OP_ListDirectory,
    OP_ListDirectoryWithTime,
    OP_Nack,
    OP_OpenFileRO,
    OP_RemoveFile,
    OP_ResetSessions,
    OP_TerminateSession,
)


def parse_ftp_payload(payload):
    """Decode the FTP bytes carried in a real MAVLink message."""
    seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = struct.unpack(
        "<HBBBBBBI", bytes(payload[:12])
    )
    return FTP_OP(
        seq,
        session,
        opcode,
        size,
        req_opcode,
        burst_complete,
        offset,
        bytearray(payload[12 : 12 + size]),
    )


class FTPUDPResponder(threading.Thread):
    """Small real-MAVLink server implementing the replies used by scenarios."""

    def __init__(self, master):
        super().__init__(daemon=True)
        self.master = master
        self.stop_event = threading.Event()
        self.requests = []
        self.envelopes = []
        self.error = None
        self.error_event = threading.Event()
        self.reject_replies = False

    def stop(self):
        self.stop_event.set()

    def run(self):
        try:
            while not self.stop_event.is_set():
                message = self.master.recv_match(
                    type="FILE_TRANSFER_PROTOCOL", blocking=True, timeout=0.05
                )
                if message is None:
                    continue
                self.envelopes.append(
                    (
                        message.target_network,
                        message.target_system,
                        message.target_component,
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                    )
                )
                request = parse_ftp_payload(message.payload)
                self.requests.append(request)
                if request.opcode == OP_ResetSessions:
                    self.send_reply(message, request, OP_Ack, session=0)
                elif request.opcode == OP_RemoveFile:
                    self.send_reply(message, request, OP_Ack)
                elif request.opcode == OP_ListDirectoryWithTime:
                    self.send_reply(
                        message,
                        request,
                        OP_Nack,
                        payload=bytes([FtpError.UnknownCommand]),
                    )
                elif request.opcode == OP_ListDirectory:
                    if request.offset == 0:
                        self.send_reply(
                            message,
                            request,
                            OP_Ack,
                            payload=b"Dlogs\x00Ffile.txt\t4",
                        )
                    else:
                        self.send_reply(
                            message,
                            request,
                            OP_Nack,
                            payload=bytes([FtpError.EndOfFile]),
                        )
                elif request.opcode == OP_OpenFileRO:
                    self.send_reply(
                        message,
                        request,
                        OP_Ack,
                        session=42,
                        payload=struct.pack("<I", 4),
                    )
                elif request.opcode == OP_BurstReadFile:
                    self.send_reply(
                        message,
                        request,
                        OP_Ack,
                        payload=b"data",
                        burst_complete=1,
                        session=42,
                    )
                elif request.opcode == OP_TerminateSession:
                    self.send_reply(message, request, OP_Ack)
        except Exception as error:  # pragma: no cover - asserted by the test thread  # noqa: BLE001 # pylint: disable=broad-exception-caught
            self.error = error
            self.error_event.set()
            self.stop_event.set()

    def send_reply(  # pylint: disable=too-many-arguments
        self,
        message,
        request,
        opcode,
        *,
        session=None,
        payload=b"",
        burst_complete=0,
    ):
        reply = FTP_OP(
            seq=(request.seq + 1) & 0xFFFF,
            session=request.session if session is None else session,
            opcode=opcode,
            size=len(payload),
            req_opcode=request.opcode,
            burst_complete=burst_complete,
            offset=request.offset,
            payload=bytearray(payload),
        ).pack()
        reply.extend(b"\0" * (251 - len(reply)))
        target_system = 251 if self.reject_replies else message.get_srcSystem()
        target_component = 191 if self.reject_replies else message.get_srcComponent()
        self.master.mav.file_transfer_protocol_send(0, target_system, target_component, reply)


class TestMAVFTPUDP(unittest.TestCase):
    """Feature: MAVFTP interoperates with a real MAVLink UDP transport."""

    def setUp(self):
        try:
            self.server = mavutil.mavlink_connection(
                "udpin:127.0.0.1:0", source_system=1, source_component=1
            )
            server_port = self.server.port.getsockname()[1]
            self.client = mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{server_port}",
                source_system=250,
                source_component=190,
            )
        except OSError as error:
            self._close_connections()
            raise unittest.SkipTest(f"loopback UDP unavailable: {error}")
        except Exception:
            self._close_connections()
            raise

        self.responder = FTPUDPResponder(self.server)
        self.responder.start()
        try:
            self.ftp = MAVFTP(self.client, target_system=1, target_component=1)
            # Keep retries short enough for a failed responder to surface, but
            # allow normal thread scheduling and CI load to deliver a packet.
            self.ftp.ftp_settings.read_retry_time = 0.2
            self.ftp.ftp_settings.idle_detection_time = 0.5
            self.ftp.ftp_settings.retry_time = 0.5
        except Exception:
            self.responder.stop()
            self.responder.join(timeout=1)
            self._close_connections()
            raise

    def tearDown(self):
        self.responder.stop()
        self.responder.join(timeout=1)
        self._close_connections()
        if self.responder.error is not None:
            raise self.responder.error

    def _close_connections(self):
        for connection in (
            getattr(self, "client", None),
            getattr(self, "server", None),
        ):
            if connection is not None:
                connection.close()

    def test_real_udp_remove_file_round_trip(self):
        """Given a real UDP link, when rm is sent, then MAVLink and FTP replies complete it."""
        result = self.ftp.cmd_rm(["remote.bin"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIsNone(self.responder.error)
        self.assertEqual(
            [(request.opcode, request.seq, request.payload) for request in self.responder.requests],
            [(OP_ResetSessions, 0, b""), (OP_RemoveFile, 1, b"remote.bin")],
        )
        self.assertEqual(
            self.responder.envelopes,
            [(0, 1, 1, 250, 190), (0, 1, 1, 250, 190)],
        )

    def test_real_udp_list_round_trip_decodes_entries(self):
        """Given a real UDP link, when list is sent, then paged directory entries are decoded end-to-end."""
        result = self.ftp.cmd_list(["logs"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIsNone(self.responder.error)
        self.assertEqual(
            [(entry.name, entry.is_dir, entry.size_b) for entry in result.directory_listing],
            [("logs", True, 0), ("file.txt", False, 4)],
        )
        self.assertEqual(
            [(request.opcode, request.offset, request.payload) for request in self.responder.requests],
            [
                (OP_ResetSessions, 0, b""),
                (OP_ListDirectoryWithTime, 0, b"logs"),
                (OP_ListDirectory, 0, b"logs"),
                (OP_ListDirectory, 2, b"logs"),
            ],
        )
        self.assertEqual(
            self.responder.envelopes,
            [(0, 1, 1, 250, 190)] * 4,
        )

    def test_real_udp_wrong_target_reply_is_rejected(self):
        """Given a reply addressed to another component, when rm waits, then it is not accepted as completion."""
        self.responder.reject_replies = True

        result = self.ftp.cmd_rm(["remote.bin"])

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(self.responder.requests[-1].opcode, OP_RemoveFile)

    def test_real_udp_download_round_trip_uses_remote_session(self):
        """Given a real UDP link, when get receives a file, then data and session termination complete end-to-end."""
        self.assertEqual(self.ftp.cmd_get(["remote.bin", "-"]).error_code, FtpError.Success)
        result = self.ftp.process_ftp_reply("get", timeout=5)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(self.ftp.get_result, b"data")
        self.assertEqual(
            [(request.opcode, request.session, request.offset) for request in self.responder.requests],
            [
                (OP_ResetSessions, 0, 0),
                (OP_OpenFileRO, 0, 0),
                (OP_BurstReadFile, 42, 0),
                (OP_TerminateSession, 42, 0),
            ],
        )
        self.assertEqual(self.responder.envelopes, [(0, 1, 1, 250, 190)] * 4)


if __name__ == "__main__":
    unittest.main()
