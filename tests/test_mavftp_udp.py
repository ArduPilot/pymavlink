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
import sys
import tempfile
import threading
import unittest
from io import BytesIO
from unittest.mock import patch

from pymavlink import mavutil
from pymavlink.mavftp import (
    DirectoryEntry,
    FTP_OP,
    MAX_INITIAL_RETRIES,
    MAVFTP,
    FtpError,
    OP_Ack,
    OP_BurstReadFile,
    OP_CalcFileCRC32,
    OP_CreateFile,
    OP_ListDirectory,
    OP_ListDirectoryWithTime,
    OP_Nack,
    OP_OpenFileRO,
    OP_RemoveFile,
    OP_ResetSessions,
    OP_TerminateSession,
    OP_WriteFile,
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


class BinaryStdout:
    """Capture the binary stdout stream used by ``get ... -``."""

    def __init__(self):
        self.buffer = BytesIO()

    def write(self, text):
        return len(text)

    def flush(self):
        pass


class FTPUDPResponder(  # pylint: disable=too-many-instance-attributes
    threading.Thread
):
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
        self.silent_timestamp_listing = False
        self.crc_files = {}
        self.uploads = {}

    def stop(self):
        self.stop_event.set()

    def run(self):  # pylint: disable=too-many-branches
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
                    if not self.silent_timestamp_listing:
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
                elif request.opcode == OP_CalcFileCRC32:
                    name = bytes(request.payload)
                    if name not in self.crc_files:
                        self.send_reply(
                            message,
                            request,
                            OP_Nack,
                            payload=bytes([FtpError.FileNotFound]),
                        )
                    else:
                        self.send_reply(
                            message,
                            request,
                            OP_Ack,
                            payload=struct.pack("<I", self.crc_files[name]),
                        )
                elif request.opcode == OP_CreateFile:
                    self.uploads[42] = bytearray()
                    self.send_reply(message, request, OP_Ack, session=42)
                elif request.opcode == OP_WriteFile:
                    uploaded = self.uploads.setdefault(request.session, bytearray())
                    end = request.offset + len(request.payload)
                    if len(uploaded) < end:
                        uploaded.extend(b"\0" * (end - len(uploaded)))
                    uploaded[request.offset:end] = request.payload
                    self.send_reply(
                        message,
                        request,
                        OP_Ack,
                        session=request.session,
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
        except OSError as error:
            self._close_connections()
            raise unittest.SkipTest(f"loopback UDP unavailable: {error}")
        try:
            server_port = self.server.port.getsockname()[1]
            self.client = mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{server_port}",
                source_system=250,
                source_component=190,
            )
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
            # Leave enough quiet time for the responder thread to drain a
            # batched datagram before idle fallback reports a result.
            self.ftp.ftp_settings.idle_detection_time = 2.0
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
        self.ftp.ftp_settings.idle_detection_time = 2.0
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

    def test_real_udp_silent_timestamp_listing_falls_back(self):
        """A real UDP server that drops timestamps is reached by baseline fallback."""
        self.responder.silent_timestamp_listing = True
        self.ftp.ftp_settings.list_time_timeout = 0.6
        self.ftp.ftp_settings.list_retries = 0

        result = self.ftp.cmd_list(["logs"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIsNone(self.responder.error)
        self.assertEqual(
            result.directory_listing,
            [
                DirectoryEntry("logs", True, 0),
                DirectoryEntry("file.txt", False, 4),
            ],
        )
        self.assertEqual(
            [
                (request.opcode, request.seq, request.offset, bytes(request.payload))
                for request in self.responder.requests
            ],
            [
                (OP_ResetSessions, 0, 0, b""),
                (OP_ListDirectoryWithTime, 1, 0, b"logs"),
                (OP_ListDirectory, 2, 0, b"logs"),
                (OP_ListDirectory, 3, 2, b"logs"),
            ],
        )

    def test_real_udp_wrong_target_reply_is_rejected(self):
        """Wrong-target replies are ignored while the request retry budget runs out."""
        self.responder.reject_replies = True
        # A delayed sample from a prior command must not make this five-second
        # caller timeout omit the latter steps of the initial retry ladder.
        self.ftp.rtt_valid = True
        self.ftp.rtt = 0.3
        self.ftp.rttvar = 0.15

        result = self.ftp.cmd_rm(["remote.bin"])

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)
        remove_requests = [
            request
            for request in self.responder.requests
            if request.opcode == OP_RemoveFile
        ]
        self.assertEqual(len(remove_requests), MAX_INITIAL_RETRIES + 1)
        self.assertEqual(
            [request.seq for request in remove_requests],
            [remove_requests[0].seq] * (MAX_INITIAL_RETRIES + 1),
        )

    def test_real_udp_download_round_trip_uses_remote_session(self):
        """Given a real UDP link, when get receives a file, then data and session termination complete end-to-end."""
        self.assertEqual(self.ftp.cmd_get(["remote.bin", "-"]).error_code, FtpError.Success)
        stdout = BinaryStdout()
        with patch.object(sys, "stdout", stdout):
            result = self.ftp.process_ftp_reply("get", timeout=5)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(stdout.buffer.getvalue(), b"data")
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

    def test_real_udp_crc_compare_round_trip(self):
        """CRC comparison exchanges multiple real MAVLink FTP requests."""
        # CRC requests are synchronous and do not have a transfer-session
        # retry path; leave enough idle time for a busy test runner to wake
        # the responder thread without turning a valid reply into a timeout.
        self.ftp.ftp_settings.idle_detection_time = 3.0
        self.ftp.ftp_settings.retry_time = 2.0
        with tempfile.TemporaryDirectory() as temp_dir:
            files = {
                "match.bin": b"matching payload",
                "different.bin": b"local payload",
                "missing.bin": b"not on vehicle",
            }
            for name, payload in files.items():
                with open(f"{temp_dir}/{name}", "wb") as local_file:
                    local_file.write(payload)
            self.responder.crc_files[b"/remote/match.bin"] = MAVFTP.local_file_crc(
                f"{temp_dir}/match.bin"
            )
            self.responder.crc_files[b"/remote/different.bin"] = 0

            result = self.ftp.cmd_crccmp([f"{temp_dir}/*.bin", "/remote"])

            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(
                self.ftp.crccmp_results,
                ["DIFFER", "MATCH", "MISSING"],
            )
            self.assertEqual(
                [request.opcode for request in self.responder.requests],
                [
                    OP_ResetSessions,
                    OP_CalcFileCRC32,
                    OP_CalcFileCRC32,
                    OP_CalcFileCRC32,
                ],
            )

    def test_real_udp_upload_round_trip_uses_batched_writes(self):
        """Multiple upload blocks sent over UDP are decoded and acknowledged."""
        payload = b"0123456789abcdefghijABCDEFGHIJ"
        self.ftp.ftp_settings.write_size = 10
        self.ftp.ftp_settings.write_qsize = 3
        self.ftp.ftp_settings.idle_detection_time = 2.0
        # The responder runs in a separate thread.  Leave enough time for it
        # to drain the batched datagram before the upload retry logic resends
        # a block and makes the request-count assertions nondeterministic.
        self.ftp.ftp_settings.retry_time = 2.0

        result = self.ftp.cmd_put(
            ["unused", "/remote/upload.bin"],
            fh=BytesIO(payload),
        )
        self.assertEqual(result.error_code, FtpError.Success)
        result = self.ftp.process_ftp_reply("put", timeout=5)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(bytes(self.responder.uploads[42]), payload)
        write_requests = [
            request
            for request in self.responder.requests
            if request.opcode == OP_WriteFile
        ]
        self.assertEqual(len(write_requests), 3)
        self.assertEqual(
            sorted({request.offset for request in write_requests}),
            [0, 10, 20],
        )
        self.assertEqual(
            {
                request.offset: (request.session, bytes(request.payload))
                for request in write_requests
            },
            {
                0: (42, b"0123456789"),
                10: (42, b"abcdefghij"),
                20: (42, b"ABCDEFGHIJ"),
            },
        )
        self.assertEqual(
            [request.opcode for request in self.responder.requests],
            [
                OP_ResetSessions,
                OP_CreateFile,
                OP_WriteFile,
                OP_WriteFile,
                OP_WriteFile,
                OP_TerminateSession,
            ],
        )
        self.assertEqual(self.responder.requests[1].payload, b"/remote/upload.bin")
        self.assertEqual(
            [
                (request.seq, request.session, request.offset)
                for request in self.responder.requests[2:]
            ],
            [(2, 42, 0), (3, 42, 10), (4, 42, 20), (5, 42, 0)],
        )


if __name__ == "__main__":
    unittest.main()
