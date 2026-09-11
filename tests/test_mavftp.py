#!/usr/bin/env python3

'''
MAVLink File Transfer Protocol support test - https://mavlink.io/en/services/ftp.html

SPDX-FileCopyrightText: 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
'''

import logging
import os
import socket
import struct
import tempfile
import time
import unittest
from argparse import Namespace
from io import BytesIO, StringIO

from unittest.mock import MagicMock, patch
from pymavlink import mavftp as mavftp_module
from pymavlink import mavutil
from pymavlink.mavftp import (
    BURST_REPLY_SEQUENCE_WINDOW,
    DirectoryEntry,
    FTP_OP,
    FTP_SEQ_MODULUS,
    FTP_SESSION_MODULUS,
    MAX_READ_GAPS,
    MAVFTP,
    MAVFTPSetting,
    MAVFTPSettings,
    FtpError,
    MAVFTPReturn,
    OP_Ack,
    OP_BurstReadFile,
    OP_CalcFileCRC32,
    OP_CreateFile,
    OP_CreateDirectory,
    OP_ListDirectory,
    OP_ListDirectoryWithTime,
    OP_Nack,
    OP_OpenFileRO,
    OP_ReadFile,
    OP_RemoveFile,
    OP_RemoveDirectory,
    OP_Rename,
    OP_ResetSessions,
    OP_TerminateSession,
    OP_WriteFile,
    create_argument_parser,
    local_file_crc,
)

# pylint: disable=protected-access,too-many-lines,duplicate-code


class FakeFTPMessage:  # pylint: disable=too-few-public-methods
    """Minimal FILE_TRANSFER_PROTOCOL message for reply-loop tests."""

    def __init__(self, op):
        self.payload = op.pack()
        self.target_system = 1
        self.target_component = 1

    @staticmethod
    def get_type():
        return "FILE_TRANSFER_PROTOCOL"


class RawFTPMessage:  # pylint: disable=too-few-public-methods
    """Minimal raw FTP message used to exercise malformed-packet handling."""

    def __init__(self, payload):
        self.payload = payload
        self.target_system = 1
        self.target_component = 1

    @staticmethod
    def get_type():
        return "FILE_TRANSFER_PROTOCOL"


class FakeMAV:  # pylint: disable=too-few-public-methods
    """Record FTP sends without requiring a MAVLink transport."""

    def __init__(self):
        self.sent = []

    def file_transfer_protocol_send(self, *args):
        self.sent.append(args)


class BatchLink:  # pylint: disable=too-few-public-methods
    """Collect raw writes from the upload batching path."""

    def __init__(self):
        self.port = type("Port", (), {"type": socket.SOCK_DGRAM})()
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)


class BatchMAV:  # pylint: disable=too-few-public-methods
    """Minimal MAVLink encoder that writes through a replaceable link."""

    def __init__(self, link):
        self.file = link

    def file_transfer_protocol_send(self, _network, _target_system, _target_component, payload):
        self.file.write(b"F" + bytes(payload))


class FakeMaster:  # pylint: disable=too-few-public-methods
    """Serve a predetermined sequence of FTP replies."""

    source_system = 1
    source_component = 1

    def __init__(self, replies):
        self.mav = FakeMAV()
        self.replies = replies
        self.empty_polls = 0
        self.recv_calls = []

    def recv_match(self, **_kwargs):
        self.recv_calls.append(_kwargs)
        if self.empty_polls:
            self.empty_polls -= 1
            return None
        if self.replies:
            return self.replies.pop(0)
        return None


class StaleCRCReplyMaster(FakeMaster):  # pylint: disable=too-few-public-methods
    """Return one stale CRC reply before the reply to the active request."""

    def __init__(self, crc):
        super().__init__([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        self.crc = crc
        self.stale_sent = False
        self.current_sent = False

    def recv_match(self, **kwargs):
        if self.mav.sent and self.mav.sent[-1][-1][3] == OP_CalcFileCRC32:
            request_payload = self.mav.sent[-1][-1]
            request_seq = struct.unpack_from("<H", request_payload)[0]
            if not self.stale_sent:
                self.stale_sent = True
                return ftp_reply(
                    request_seq + 7,
                    OP_Ack,
                    OP_CalcFileCRC32,
                    payload=struct.pack("<I", self.crc ^ 1),
                )
            if not self.current_sent:
                self.current_sent = True
                return ftp_reply(
                    request_seq + 1,
                    OP_Ack,
                    OP_CalcFileCRC32,
                    payload=struct.pack("<I", self.crc),
                )
        return super().recv_match(**kwargs)


class AllocatingSessionReplayMaster(FakeMaster):  # pylint: disable=too-few-public-methods
    """Server model that reuses session 0 and replays a prior burst packet."""

    def __init__(self):
        super().__init__([])
        self.sent_index = 0
        self.download_count = 0
        self.stale_reply = None

    def recv_match(self, **kwargs):
        while self.sent_index < len(self.mav.sent):
            payload = self.mav.sent[self.sent_index][-1]
            self.sent_index += 1
            (seq, session, opcode, size, _req_opcode, _burst_complete, _pad, offset) = (
                struct.unpack("<HBBBBBBI", payload[:12])
            )
            request = FTP_OP(
                seq, session, opcode, size, 0, 0, offset, bytearray(payload[12 : 12 + size])
            )
            reply_seq = (request.seq + 1) % FTP_SEQ_MODULUS
            if request.opcode == OP_ResetSessions:
                self.replies.append(ftp_reply(reply_seq, OP_Ack, OP_ResetSessions))
            elif request.opcode == OP_OpenFileRO:
                self.download_count += 1
                self.replies.append(
                    ftp_reply(reply_seq, OP_Ack, OP_OpenFileRO, payload=[63, 1, 0, 0])
                )
            elif request.opcode == OP_BurstReadFile:
                data = b"A" if self.download_count == 1 else b"B"
                replies = [
                    ftp_reply(reply_seq + index, OP_Ack, OP_BurstReadFile,
                              payload=data * size, offset=index * size,
                              burst_complete=index == 3)
                    for index in range(3)
                ]
                replies.append(
                    ftp_reply(reply_seq + 3, OP_Ack, OP_BurstReadFile,
                              payload=data * (size - 1), offset=3 * size,
                              burst_complete=1)
                )
                if self.download_count == 1:
                    self.stale_reply = replies[-1]
                else:
                    self.replies.append(self.stale_reply)
                self.replies.extend(replies)
            elif request.opcode == OP_TerminateSession:
                self.replies.append(ftp_reply(reply_seq, OP_Ack, OP_TerminateSession))
        return super().recv_match(**kwargs)


def ftp_reply(  # pylint: disable=too-many-arguments
    seq, opcode, req_opcode, payload=None, offset=0, burst_complete=0, session=0
):
    """Create a parsed FTP response represented as a minimal MAVLink message."""
    data = bytearray(payload) if payload is not None else bytearray()
    return FakeFTPMessage(
        FTP_OP(
            seq=seq,
            session=session,
            opcode=opcode,
            size=len(data),
            req_opcode=req_opcode,
            burst_complete=burst_complete,
            offset=offset,
            payload=data,
        )
    )


class TestMAVFTPReplyCompletion(unittest.TestCase):  # pylint: disable=too-many-public-methods
    """Regression tests for FTP replies, retries, and session cleanup."""

    @staticmethod
    def make_ftp(replies, list_time=0):
        master = FakeMaster(
            [ftp_reply(1, OP_Ack, OP_ResetSessions)] + replies
        )
        ftp = MAVFTP(master, target_system=1, target_component=1)
        if list_time is not None:
            ftp.ftp_settings.list_time = list_time
        ftp.ftp_settings.read_retry_time = 0.01
        ftp.ftp_settings.idle_detection_time = 0.02
        ftp.ftp_settings.retry_time = 0.2
        return ftp, master

    def test_terminate_ignores_reply_for_wrong_target_or_session(self):
        """TerminateSession accepts replies only from its target and session."""
        for target_system, session in ((99, 0), (1, 1)):
            with self.subTest(target_system=target_system, session=session):
                ftp, master = self.make_ftp([])
                ftp.pending_terminate_seq = ftp.seq
                reply = ftp_reply(
                    ftp.seq + 1,
                    OP_Ack,
                    OP_TerminateSession,
                    session=session,
                )
                reply.target_system = target_system
                master.replies.append(reply)

                result = ftp.process_ftp_reply("TerminateSession")

                self.assertEqual(result.error_code, FtpError.Fail)
                self.assertEqual(ftp.pending_terminate_seq, ftp.seq)

    def test_reply_from_another_vehicle_is_rejected(self):
        """A colliding session reply from another vehicle cannot complete a command."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_RemoveFile, 0, 0, 0, 0, bytearray())
        reply = ftp_reply(2, OP_Ack, OP_RemoveFile)
        reply.get_srcSystem = lambda: 2
        reply.get_srcComponent = lambda: 1

        result = ftp._MAVFTP__mavlink_packet(reply)

        self.assertEqual(result.error_code, FtpError.InvalidSession)

    def test_status_reports_a_transfer_during_open_handshake(self):
        """A transfer remains visible before its remote file handle is opened."""
        ftp, _master = self.make_ftp([])
        ftp.transfer_active = True

        with self.assertLogs(level="INFO") as logs:
            result = ftp.cmd_status()

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIn("Transfer in progress", "\n".join(logs.output))

    @staticmethod
    def sent_request_sequences(master, opcode):
        """Return FTP request sequence numbers sent for an opcode."""
        return [
            struct.unpack_from("<H", sent[-1])[0]
            for sent in master.mav.sent
            if sent[-1][3] == opcode
        ]

    @staticmethod
    def sent_requests(master):
        """Decode the outgoing FTP requests, excluding MAVLink padding."""
        requests = []
        for sent in master.mav.sent:
            payload = sent[-1]
            seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = (
                struct.unpack_from("<HBBBBBBI", payload)
            )
            requests.append(
                FTP_OP(
                    seq,
                    session,
                    opcode,
                    size,
                    req_opcode,
                    burst_complete,
                    offset,
                    bytearray(payload[12 : 12 + size]),
                )
            )
        return requests

    def test_simple_commands_encode_paths_and_complete(self):
        """Given a simple public command, when dispatched, then it emits and completes the exact wire request."""
        cases = (
            ("rm", ["remote"], OP_RemoveFile, b"remote", 0),
            ("rmdir", ["directory"], OP_RemoveDirectory, b"directory", 0),
            ("mkdir", ["directory"], OP_CreateDirectory, b"directory", 0),
            ("rename", ["old", "new"], OP_Rename, b"old\x00new", 0),
            ("crc", ["remote"], OP_CalcFileCRC32, b"remote", 0),
        )
        for command, arguments, opcode, expected_payload, expected_offset in cases:
            with self.subTest(command=command):
                reply_payload = struct.pack("<I", 0x12345678) if opcode == OP_CalcFileCRC32 else None
                ftp, master = self.make_ftp(
                    [ftp_reply(2, OP_Ack, opcode, payload=reply_payload)]
                )

                result = ftp.cmd_ftp([command, *arguments])

                self.assertEqual(result.error_code, FtpError.Success)
                request = self.sent_requests(master)[-1]
                self.assertEqual(request.seq, 1)
                self.assertEqual(request.opcode, opcode)
                self.assertEqual(request.session, 0)
                self.assertEqual(request.offset, expected_offset)
                self.assertEqual(request.payload, expected_payload)
                self.assertEqual(request.size, len(expected_payload))
                self.assertEqual(master.mav.sent[-1][:3], (0, 1, 1))
                self.assertEqual(master.replies, [])

    def test_simple_commands_complete_without_idle_tail(self):
        """Single-reply commands return without waiting for link idleness."""
        cases = (
            ("rm", ["remote"], OP_RemoveFile, None),
            ("rmdir", ["directory"], OP_RemoveDirectory, None),
            ("mkdir", ["directory"], OP_CreateDirectory, None),
            ("rename", ["old", "new"], OP_Rename, None),
            ("crc", ["remote"], OP_CalcFileCRC32, struct.pack("<I", 0x12345678)),
        )
        for command, arguments, opcode, payload in cases:
            with self.subTest(command=command):
                ftp, master = self.make_ftp(
                    [ftp_reply(2, OP_Ack, opcode, payload=payload)]
                )
                ftp.ftp_settings.idle_detection_time = 1.0
                polls_before = len(master.recv_calls)

                result = ftp.cmd_ftp([command, *arguments])

                self.assertEqual(result.error_code, FtpError.Success)
                self.assertEqual(len(master.recv_calls), polls_before + 1)

    def test_crc_ack_with_invalid_payload_size_fails_cleanly(self):
        """Given a CRC ACK with a non-four-byte payload, when handled, then it reports invalid data size."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_crc(["remote.bin"])

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_CalcFileCRC32, payload=b"bad")
        )

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)
        self.assertEqual(result.operation_name, "CalcFileCRC32")

    def test_malformed_ftp_header_returns_invalid_data_size(self):
        """Given a FILE_TRANSFER_PROTOCOL payload shorter than its header, when parsed, then it fails without raising."""
        ftp, _master = self.make_ftp([])

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            RawFTPMessage(b"\x00" * 3)
        )

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_declared_ftp_payload_larger_than_bytes_returns_invalid_data_size(self):
        """Given an FTP header declaring unavailable payload bytes, when parsed, then it fails without partial decoding."""
        ftp, _master = self.make_ftp([])
        malformed_header = struct.pack("<HBBBBBBI", 2, 0, OP_Ack, 4, OP_RemoveFile, 0, 0, 0)

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            RawFTPMessage(malformed_header)
        )

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_process_rejects_malformed_ftp_header_without_raising(self):
        """Given a malformed reply in the receive loop, when processed, then it returns invalid data size."""
        ftp, _master = self.make_ftp([RawFTPMessage(b"\x00" * 3)])

        result = ftp.process_ftp_reply("RemoveFile", timeout=1)

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_packet_loss_settings_drop_packets_at_the_expected_boundary(self):
        """Given 100-percent TX or RX loss, when a burst packet is handled, then no payload is accepted or written."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.session = 7
        ftp.burst_size = 80
        ftp.ftp_settings.pkt_loss_tx = 100
        tx_result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(2, 7, OP_Ack, 1, OP_BurstReadFile, 1, 0, bytearray(b"x")), None
        )
        self.assertEqual(tx_result.error_code, FtpError.Fail)
        self.assertEqual(ftp.fh.getvalue(), b"")

        ftp.ftp_settings.pkt_loss_tx = 0
        ftp.ftp_settings.pkt_loss_rx = 100
        ftp.pending_burst_offset = 0
        ftp.pending_burst_seq = 2
        ftp.pending_burst_request = FTP_OP(1, 7, OP_BurstReadFile, 80, 0, 0, 0, None)
        rx_result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_BurstReadFile, payload=b"x", session=7)
        )
        self.assertEqual(rx_result.error_code, FtpError.Fail)
        self.assertEqual(ftp.fh.getvalue(), b"")

    def test_put_missing_local_file_reports_open_failure_and_clears_state(self):
        """Given a missing local source, when put starts, then it reports open failure without creating upload state."""
        ftp, _master = self.make_ftp([])

        result = ftp.cmd_put(["/path/that/does/not/exist", "remote.bin"])

        self.assertEqual(result.error_code, FtpError.FailToOpenLocalFile)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.write_list)

    def test_download_staging_open_failure_terminates_the_remote_session(self):
        """A staging-file failure after OpenFileRO terminates the session."""
        ftp, master = self.make_ftp(
            [ftp_reply(3, OP_Ack, OP_TerminateSession, session=0)]
        )
        ftp.cmd_get(["remote.bin", "destination.bin"])

        with patch("pymavlink.mavftp.tempfile.mkstemp", side_effect=OSError("no staging file")):
            result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[4, 0, 0, 0], session=7)
            )

        self.assertEqual(result.error_code, FtpError.FileNotFound)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.filename)
        self.assertIsNone(ftp.temp_filename)
        self.assertEqual(master.replies, [])

    def test_reset_nack_clears_pending_handshake_and_preserves_error(self):
        """Given a pending reset, when the server NACKs it, then the handshake closes and its error is returned."""
        ftp, _master = self.make_ftp([])
        ftp.pending_reset_seq = 1

        result = ftp._MAVFTP__handle_reset_sessions_reply(  # pylint: disable=protected-access
            FTP_OP(2, 0, OP_Nack, 1, OP_ResetSessions, 0, 0, bytearray([FtpError.FileProtected])),
            None,
        )

        self.assertEqual(result.error_code, FtpError.FileProtected)
        self.assertIsNone(ftp.pending_reset_seq)

    def test_extract_params_decodes_names_and_supports_sort_modes(self):
        """Given packed parameter entries, when extracted, then names/types are decoded with requested ordering."""
        pdata = [
            (b"Z_PARAM", 2.0, 4),
            (b"A_10", 10.0, 4),
            (b"A_2", 2.0, 4),
            (b"A2", 2.0, 4),
        ]

        missionplanner = MAVFTP.extract_params(pdata, "missionplanner")
        mavproxy = MAVFTP.extract_params(pdata, "mavproxy")
        unsorted = MAVFTP.extract_params(pdata, "none")

        self.assertEqual(list(missionplanner), ["A_10", "A_2", "A2", "Z_PARAM"])
        self.assertEqual(list(mavproxy), ["A2", "A_10", "A_2", "Z_PARAM"])
        self.assertEqual(list(unsorted), ["Z_PARAM", "A_10", "A_2", "A2"])
        self.assertEqual(missionplanner["A_2"], (2.0, 4))

    def test_list_decodes_entries_sorts_them_and_advances_offset(self):
        """Given a paged directory listing, when list completes, then sorted entries and the next offset are returned."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_ListDirectory, payload=b"Fzeta\t7\x00Dalpha\x00Fbeta\tbad"),
                ftp_reply(3, OP_Nack, OP_ListDirectory, payload=[FtpError.EndOfFile], offset=3),
            ]
        )

        result = ftp.cmd_list(["logs"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            [(entry.name, entry.is_dir, entry.size_b) for entry in result.directory_listing],
            [("alpha", True, 0), ("beta", False, 0), ("zeta", False, 7)],
        )
        requests = self.sent_requests(master)
        self.assertEqual(
            [(request.opcode, request.offset) for request in requests[-2:]],
            [(OP_ListDirectory, 0), (OP_ListDirectory, 3)],
        )
        self.assertEqual([request.payload for request in requests[-2:]], [b"logs", b"logs"])
        self.assertEqual(master.replies, [])

    def test_download_publishes_complete_data_and_reports_progress(self):
        """Given a complete remote file, when get succeeds, then exact data is published and progress reaches 100%."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "download.bin")
            ftp, master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=struct.pack("<I", 4), session=8),
                    ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"data", burst_complete=1, session=8),
                    ftp_reply(4, OP_Ack, OP_TerminateSession, session=8),
                ]
            )
            progress = []

            self.assertEqual(
                ftp.cmd_get(["remote.bin", destination], progress_callback=progress.append).error_code,
                FtpError.Success,
            )
            result = ftp.process_ftp_reply("get", timeout=1)

            self.assertEqual(result.error_code, FtpError.Success)
            with open(destination, "rb") as downloaded:
                self.assertEqual(downloaded.read(), b"data")
            self.assertEqual(progress, [1.0])
            self.assertIsNone(ftp.fh)
            self.assertIsNone(ftp.temp_filename)
            requests = self.sent_requests(master)
            transfer_requests = [
                request
                for request in requests
                if request.opcode in {OP_OpenFileRO, OP_BurstReadFile, OP_TerminateSession}
            ]
            self.assertEqual(transfer_requests[0].opcode, OP_OpenFileRO)
            self.assertEqual(transfer_requests[0].payload, b"remote.bin")
            self.assertEqual(transfer_requests[1].opcode, OP_BurstReadFile)
            self.assertTrue(
                all(request.opcode == OP_TerminateSession for request in transfer_requests[2:])
            )
            self.assertEqual(master.replies, [])

    def test_upload_sends_exact_data_at_the_allocated_session(self):
        """Given local bytes, when put receives its allocated session, then its write has exact bytes, offset, and session."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_CreateFile, session=9),
                ftp_reply(3, OP_Ack, OP_WriteFile, offset=0, session=9),
                ftp_reply(4, OP_Ack, OP_TerminateSession, session=9),
            ]
        )

        self.assertEqual(ftp.cmd_put(["local.bin", "remote.bin"], fh=BytesIO(b"payload")).error_code, FtpError.Success)
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        write = next(request for request in self.sent_requests(master) if request.opcode == OP_WriteFile)
        self.assertEqual((write.session, write.offset, write.payload), (9, 0, b"payload"))
        self.assertEqual(write.size, len(b"payload"))
        self.assertEqual(master.replies, [])

    def test_getparams_decodes_and_writes_values_and_defaults(self):
        """Given a parameter archive, when getparams completes, then both requested files are decoded and written."""
        parameter_data = (
            struct.pack("<HHH", 0x671C, 1, 1)
            + struct.pack("<BB", 0x14, 3 << 4)
            + b"RATE"
            + struct.pack("<ff", 1.5, 2.5)
        )
        with tempfile.TemporaryDirectory() as tempdir:
            values_path = os.path.join(tempdir, "values.param")
            defaults_path = os.path.join(tempdir, "defaults.param")
            ftp, master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=struct.pack("<I", len(parameter_data)), session=3),
                    ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=parameter_data, burst_complete=1, session=3),
                    ftp_reply(4, OP_Ack, OP_TerminateSession, session=3),
                ]
            )
            progress = []

            self.assertEqual(
                ftp.cmd_getparams(
                    [values_path, defaults_path],
                    progress_callback=progress.append,
                    sort_type="missionplanner",
                    add_datatype_comments=True,
                    add_timestamp_comment=False,
                ).error_code,
                FtpError.Success,
            )
            result = ftp.process_ftp_reply("getparams", timeout=1)

            self.assertEqual(result.error_code, FtpError.Success)
            with open(values_path, encoding="utf-8") as values_file:
                self.assertEqual(values_file.read(), "RATE,1.5  # 32-bit float\n")
            with open(defaults_path, encoding="utf-8") as defaults_file:
                self.assertEqual(defaults_file.read(), "RATE,2.5  # 32-bit float\n")
            self.assertEqual(progress, [1.0])
            open_request = next(
                request
                for request in self.sent_requests(master)
                if request.opcode == OP_OpenFileRO
            )
            self.assertEqual(open_request.payload, b"@PARAM/param.pck?withdefaults=1")

    def test_cancel_resets_transfer_state_and_uses_active_session(self):
        """Given an active download, when cancel is requested, then its session is terminated and state is cleared."""
        ftp, master = self.make_ftp(
            [ftp_reply(2, OP_Ack, OP_TerminateSession, session=6)]
        )
        callback_notifications = []
        ftp.session = 6
        ftp.fh = BytesIO(b"partial")
        ftp.filename = "remote.bin"
        ftp.callback = callback_notifications.append
        ftp.read_gaps = [(0, 3)]

        result = ftp.cmd_ftp(["cancel"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(self.sent_requests(master)[-1].opcode, OP_TerminateSession)
        self.assertEqual(self.sent_requests(master)[-1].seq, 1)
        self.assertEqual(self.sent_requests(master)[-1].session, 6)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.filename)
        self.assertEqual(ftp.read_gaps, [])
        self.assertEqual(callback_notifications, [None])
        self.assertEqual(ftp.session, 7)
        self.assertIsNone(ftp.pending_terminate_seq)
        self.assertEqual(master.replies, [])

    def test_cleanup_survives_callback_exceptions(self):
        """Given callbacks that raise during cancellation, when the session is terminated, then cleanup still completes."""
        ftp, _master = self.make_ftp(
            [ftp_reply(2, OP_Ack, OP_TerminateSession, session=6)]
        )
        ftp.session = 6
        ftp.fh = BytesIO(b"partial")
        ftp.filename = "remote.bin"

        def failing_callback(_value):
            raise RuntimeError("callback failed")

        ftp.callback = failing_callback
        ftp.callback_progress = failing_callback
        ftp.put_callback = failing_callback
        ftp.put_callback_progress = failing_callback

        result = ftp.cmd_cancel()

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.callback)
        self.assertIsNone(ftp.callback_progress)
        self.assertIsNone(ftp.put_callback)
        self.assertIsNone(ftp.put_callback_progress)
        self.assertIsNone(ftp.pending_terminate_seq)

    def test_path_commands_reject_non_ascii_without_sending_a_request(self):
        """Non-ASCII paths fail before sending a protocol request."""
        cases = (
            ("list", ["rémote"]),
            ("rm", ["rémote"]),
            ("rmdir", ["rémote"]),
            ("mkdir", ["rémote"]),
            ("rename", ["old", "nëw"]),
            ("crc", ["rémote"]),
        )
        for command, arguments in cases:
            with self.subTest(command=command):
                ftp, master = self.make_ftp([])

                result = ftp.cmd_ftp([command, *arguments])

                self.assertEqual(result.error_code, FtpError.InvalidArguments)
                self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_path_commands_reject_oversized_payloads_without_sending(self):
        """Oversized paths fail before sending a protocol request."""
        oversized = "a" * (mavftp_module.MAX_Payload + 1)
        cases = (
            ("list", [oversized]),
            ("rm", [oversized]),
            ("rmdir", [oversized]),
            ("mkdir", [oversized]),
            ("get", [oversized]),
            ("crc", [oversized]),
            ("put", ["local", oversized]),
        )
        for command, arguments in cases:
            with self.subTest(command=command):
                ftp, master = self.make_ftp([])
                source = BytesIO(b"payload") if command == "put" else None

                if source is None:
                    result = ftp.cmd_ftp([command, *arguments])
                else:
                    result = ftp.cmd_put(arguments, fh=source)

                self.assertEqual(result.error_code, FtpError.InvalidArguments)
                self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

        ftp, master = self.make_ftp([])
        result = ftp.cmd_rename(["a" * 120, "b" * 120])
        self.assertEqual(result.error_code, FtpError.InvalidArguments)
        self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_read_rejects_negative_ranges_before_packing_a_request(self):
        """Given a negative read offset or size, when read is called, then it returns no data without sending a request."""
        for size, offset in ((-1, 0), (1, -1)):
            with self.subTest(size=size, offset=offset):
                ftp, master = self.make_ftp([])

                self.assertIsNone(ftp.read("remote", size, offset))
                self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_read_zero_size_returns_empty_data_after_session_handshake(self):
        """Given a zero-length read, when the remote session handshake completes, then read returns empty data."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[4, 0, 0, 0], session=7),
                ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"x", burst_complete=1, session=7),
                ftp_reply(4, OP_Ack, OP_TerminateSession, session=7),
            ]
        )

        self.assertEqual(ftp.read("remote", 0, 0), b"")
        self.assertTrue(
            any(request.opcode == OP_OpenFileRO for request in self.sent_requests(master))
        )
        self.assertEqual(master.replies, [])

    def test_status_and_unknown_dispatch_do_not_send_file_requests(self):
        """Given no transfer, when status or an unknown command is dispatched, then no file request is emitted."""
        ftp, master = self.make_ftp([])

        status = ftp.cmd_ftp(["status"])
        unknown = ftp.cmd_ftp(["does-not-exist"])

        self.assertEqual(status.error_code, FtpError.Success)
        self.assertEqual(unknown.error_code, FtpError.InvalidArguments)
        self.assertEqual([request.opcode for request in self.sent_requests(master)], [OP_ResetSessions])

    def test_dispatcher_routes_get_put_and_getparams_to_their_protocol_starters(self):
        """Given each transfer command, when dispatched through cmd_ftp, then its protocol starter is invoked."""
        ftp, master = self.make_ftp([])
        get_result = ftp.cmd_ftp(["get", "remote.bin", "-"])
        self.assertEqual(get_result.error_code, FtpError.Success)
        self.assertEqual(self.sent_requests(master)[-1].opcode, OP_OpenFileRO)

        # A new instance keeps each scenario independent and avoids treating
        # a transfer started by one dispatcher branch as an active duplicate.
        ftp, master = self.make_ftp([])
        with tempfile.NamedTemporaryFile() as source:
            source.write(b"x")
            source.flush()
            put_result = ftp.cmd_ftp(["put", source.name, "remote.bin"])
            self.assertEqual(put_result.error_code, FtpError.Success)
            self.assertEqual(self.sent_requests(master)[-1].opcode, OP_CreateFile)
            ftp.fh.close()
            ftp.fh = None

        ftp, master = self.make_ftp([])
        with tempfile.TemporaryDirectory() as tempdir:
            getparams_result = ftp.cmd_ftp(["getparams", os.path.join(tempdir, "values.param")])
            self.assertEqual(getparams_result.error_code, FtpError.Success)
            self.assertEqual(self.sent_requests(master)[-1].opcode, OP_OpenFileRO)
            self.assertEqual(self.sent_requests(master)[-1].payload, b"@PARAM/param.pck")

    def test_open_file_ack_uses_allocated_session(self):
        """A non-echoing OpenFileRO server session is used for BurstReadFile."""
        ftp, master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])

        ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[1, 0, 0, 0], session=42)
        )

        self.assertEqual(ftp.session, 42)
        self.assertEqual(master.mav.sent[-1][-1][2], 42)

    def test_allocated_session_discards_replayed_prior_download_burst_reply(self):
        """A reused allocated session cannot let old data into the next read."""
        master = AllocatingSessionReplayMaster()
        ftp = MAVFTP(master, target_system=1, target_component=1)
        ftp.ftp_settings.read_retry_time = 0.01
        ftp.ftp_settings.idle_detection_time = 0.02
        ftp.ftp_settings.retry_time = 0.2

        for expected in (b"A" * 319, b"B" * 319):
            ftp.cmd_get(
                ["remote", "-"],
                callback=lambda _fh: MAVFTPReturn("Get", FtpError.Success),
            )
            result = ftp.process_ftp_reply("get", timeout=1)
            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(ftp.get_result, expected)

    def test_cmd_get_clears_range_read_state(self):
        """A normal download must not inherit a prior range-read offset."""
        ftp, master = self.make_ftp([])
        ftp.requested_offset = 123
        ftp.requested_size = 2

        try:
            ftp.cmd_get(["remote", "download"])
            ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[4, 0, 0, 0], session=7)
            )

            self.assertEqual(ftp.requested_offset, 0)
            self.assertEqual(ftp.requested_size, 4)
            self.assertEqual(ftp.fh.tell(), 0)
            self.assertEqual(struct.unpack_from("<I", master.mav.sent[-1][-1], 8)[0], 0)
        finally:
            ftp._MAVFTP__release_staging()  # pylint: disable=protected-access

    def test_create_file_ack_uses_allocated_session(self):
        """A non-echoing CreateFile server session is used for WriteFile."""
        ftp, master = self.make_ftp([])
        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"))

        ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_CreateFile, session=37)
        )

        self.assertEqual(ftp.session, 37)
        self.assertEqual(master.mav.sent[-1][-1][2], 37)

    def test_put_waits_for_create_ack_before_sending_write(self):
        """A write cannot race the CreateFile request on a real transport."""
        ftp, master = self.make_ftp([])

        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"payload"))
        ftp._MAVFTP__idle_task()  # pylint: disable=protected-access

        self.assertEqual(self.sent_request_sequences(master, OP_WriteFile), [])
        self.assertEqual(self.sent_request_sequences(master, OP_CreateFile), [1])

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_CreateFile, session=37)
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(self.sent_request_sequences(master, OP_WriteFile), [2])

    def test_cmd_set_rejects_unsafe_transfer_settings(self):
        """Transfer settings must remain valid for the FTP state machine."""
        ftp, _master = self.make_ftp([])

        for setting, value in (
            ("write_size", "0"),
            ("write_size", "240"),
            ("write_qsize", "0"),
            ("max_backlog", "0"),
            ("retry_time", "0.1"),
            ("idle_detection_time", "0.01"),
            ("read_retry_time", "3.7"),
        ):
            with self.subTest(setting=setting, value=value):
                result = ftp.cmd_set([setting, value])
                self.assertEqual(result.error_code, FtpError.InvalidArguments)

    def test_cmd_set_rejects_an_integer_too_large_for_float(self):
        """An overflow while normalising an API-provided integer is invalid input."""
        ftp, _master = self.make_ftp([])

        result = ftp.cmd_set(["debug", 10**1000])

        self.assertEqual(result.error_code, FtpError.InvalidArguments)

    def test_put_rejects_invalid_write_size(self):
        """An API-set invalid write size must not reach division or packet packing."""
        ftp, _master = self.make_ftp([])
        ftp.ftp_settings._vars["write_size"].value = 0  # pylint: disable=protected-access

        result = ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"))

        self.assertEqual(result.error_code, FtpError.InvalidArguments)

    def test_settings_reject_invalid_values_from_library_callers(self):
        """Library callers cannot install unsafe settings after construction."""
        ftp, _master = self.make_ftp([])

        with self.assertRaises(ValueError):
            ftp.ftp_settings.write_size = 0
        with self.assertRaises(ValueError):
            ftp.ftp_settings.retry_time = 0.1

    def test_settings_append_and_accessor_cannot_bypass_validation(self):
        """Settings mutation remains validated and encapsulated."""
        settings = MAVFTPSettings([("retry_time", float, 0.5)])

        with self.assertRaises(ValueError):
            settings.append(("retry_time", float, 0.1))
        self.assertEqual(settings.retry_time, 0.5)

        setting = settings.get_setting("retry_time")
        setting.value = 0.1
        self.assertEqual(settings.retry_time, 0.5)

        supplied_setting = MAVFTPSetting("retry_time", float, 0.5)
        supplied_settings = MAVFTPSettings([supplied_setting])
        supplied_setting.value = 0.1
        self.assertEqual(supplied_settings.retry_time, 0.5)

    def test_settings_constructor_rejects_invalid_values(self):
        """Settings collections reject invalid cross-setting values."""
        with self.assertRaises(ValueError):
            MAVFTPSettings(
                [
                    ("idle_detection_time", float, 1.0),
                    ("read_retry_time", float, 1.0),
                ]
            )

    def test_settings_constructor_normalizes_declared_type(self):
        """Setting defaults and values use the declared setting type."""
        setting = MAVFTPSetting("custom", int, 3.9)
        settings = MAVFTPSettings([setting])

        self.assertEqual(setting.value, 3)
        self.assertEqual(setting.default, 3)
        self.assertEqual(settings.get_setting("custom").value, 3)
        self.assertEqual(settings.custom, 3)

    def test_settings_accepts_arbitrarily_large_integer(self):
        """Large integer settings do not overflow finiteness validation."""
        settings = MAVFTPSettings([("max_backlog", int, 10**1000)])

        self.assertEqual(settings.max_backlog, 10**1000)

    def test_timeout_handling_does_not_use_assertions(self):
        """Invalid timing combinations return errors instead of assertions."""
        ftp, _master = self.make_ftp([ftp_reply(1, OP_Ack, OP_ResetSessions)])

        result = ftp.process_ftp_reply("RemoveFile", timeout=0.01)

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)

    def test_gap_read_nack_preserves_server_error(self):
        """A failed gap repair must not turn a ReadFile NACK into success."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        result = ftp._MAVFTP__handle_reply_read(
            FTP_OP(
                1,
                0,
                OP_Nack,
                1,
                OP_ReadFile,
                0,
                0,
                bytearray([FtpError.FileNotFound]),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.FileNotFound)
        self.assertEqual(terminated, [True])

    def test_unexpected_short_gap_ack_reports_failure(self):
        """A short reply for an unknown gap must not report a completed read."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_gaps = [(4, 2)]
        ftp.read_gap_times = {(4, 2): 1}
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        result = ftp._MAVFTP__handle_reply_read(
            FTP_OP(1, 0, OP_Ack, 1, OP_ReadFile, 0, 0, bytearray(b"x")),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(terminated, [True])

    def test_malformed_directory_entry_reports_invalid_data(self):
        """A malformed file listing entry must not crash reply processing."""
        ftp, _master = self.make_ftp([])

        result = ftp._MAVFTP__handle_list_reply(
            FTP_OP(
                1,
                0,
                OP_Ack,
                len(b"Fmissing-size"),
                OP_ListDirectory,
                0,
                0,
                bytearray(b"Fmissing-size"),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_directory_listing_with_time_preserves_metadata(self):
        """The optional listing extension returns file modification times."""
        ftp, _master = self.make_ftp([])
        ftp.list_with_time = True
        ftp.last_op = FTP_OP(1, 0, OP_ListDirectoryWithTime, 1, 0, 0, 0, bytearray(b"/"))

        result = ftp._MAVFTP__handle_list_reply(
            FTP_OP(
                2,
                0,
                OP_Ack,
                0,
                OP_ListDirectoryWithTime,
                0,
                0,
                bytearray(b"Ffile.bin\t42\t1700000000\x00Dlogs\t0\t1700000001\x00"),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Success)
        file_entry = next(entry for entry in ftp.list_temp_result if not entry.is_dir)
        directory_entry = next(entry for entry in ftp.list_temp_result if entry.is_dir)
        self.assertEqual(file_entry.mtime, 1700000000)
        self.assertEqual(file_entry.size_b, 42)
        self.assertEqual(directory_entry.name, "logs")
        self.assertEqual(directory_entry.mtime, 1700000001)

    def test_directory_listing_with_time_falls_back_for_old_servers(self):
        """Old servers are retried with the standard listing opcode."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Nack, OP_ListDirectoryWithTime, payload=[FtpError.Fail]),
                ftp_reply(3, OP_Nack, OP_ListDirectory, payload=[FtpError.EndOfFile]),
            ]
        )
        ftp.ftp_settings.list_time = 1

        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.list_result, [])
        self.assertEqual(master.replies, [])

    def test_directory_listing_requests_timestamps_by_default(self):
        """The default client enables the extension for capable servers."""
        ftp, master = self.make_ftp(
            [ftp_reply(2, OP_Nack, OP_ListDirectoryWithTime, payload=[FtpError.EndOfFile])],
            list_time=None,
        )
        self.assertEqual(ftp.ftp_settings.list_time, 1)
        self.assertEqual(ftp.cmd_list([]).error_code, FtpError.Success)
        self.assertEqual(master.mav.sent[-1][-1][3], OP_ListDirectoryWithTime)

    def test_silent_timestamp_listing_falls_back_after_retries(self):
        """Servers that drop unknown listing opcodes use the baseline retry path."""
        ftp, _master = self.make_ftp([], list_time=1)
        ftp.ftp_settings.list_time_timeout = 1
        ftp.ftp_settings.list_retries = 1
        ftp.list_with_time = True
        ftp.last_op = FTP_OP(1, 0, OP_ListDirectoryWithTime, 1, 0, 0, 0, bytearray(b"/"))
        ftp.last_op_time = 0
        ftp.last_send_time = 0

        with patch("pymavlink.mavftp.time.time", return_value=1):
            self.assertFalse(ftp._MAVFTP__idle_task())
        self.assertEqual(ftp.list_time_retries, 1)
        self.assertEqual(ftp.last_op.opcode, OP_ListDirectoryWithTime)

        with patch("pymavlink.mavftp.time.time", return_value=2):
            self.assertFalse(ftp._MAVFTP__idle_task())
        self.assertEqual(ftp.last_op.opcode, OP_ListDirectory)
        self.assertFalse(ftp.list_time_supported)

    def test_directory_listing_display_includes_timestamp(self):
        """The CLI formatter makes returned timestamps visible to users."""
        result = MAVFTPReturn(
            "ListDirectory",
            FtpError.Success,
            directory_listing=[DirectoryEntry("file.bin", False, 42, 1700000000)],
        )
        with self.assertLogs(level="INFO") as logs:
            result.display_message()
        self.assertTrue(any("file.bin\t42\t" in message for message in logs.output))

    def test_local_crc_matches_vehicle_algorithm(self):
        """Local CRCs use the same raw CRC32 convention as the vehicle."""
        with tempfile.NamedTemporaryFile() as local_file:
            local_file.write(b"mavftp")
            local_file.flush()
            ftp, _master = self.make_ftp([])
            self.assertEqual(local_file_crc(local_file.name), 0x0960C765)
            self.assertEqual(ftp.local_file_crc(local_file.name), 0x0960C765)
            self.assertEqual(
                ftp.cmd_crclocal([local_file.name]).error_code, FtpError.Success
            )

    def test_crccmp_reports_match_difference_and_missing_files(self):
        """CRC comparison checks each basename and preserves result categories."""
        with tempfile.TemporaryDirectory() as temp_dir:
            paths = {}
            for name, data in (("a.bin", b"match"), ("b.bin", b"different"), ("c.bin", b"missing")):
                path = os.path.join(temp_dir, name)
                with open(path, "wb") as local_file:
                    local_file.write(data)
                paths[name] = path

            a_crc = local_file_crc(paths["a.bin"])
            b_crc = local_file_crc(paths["b.bin"])
            master = FakeMaster(
                [
                    ftp_reply(1, OP_Ack, OP_ResetSessions),
                    ftp_reply(2, OP_Ack, OP_CalcFileCRC32, payload=struct.pack("<I", a_crc)),
                    ftp_reply(3, OP_Ack, OP_CalcFileCRC32, payload=struct.pack("<I", b_crc ^ 1)),
                    ftp_reply(4, OP_Nack, OP_CalcFileCRC32, payload=[FtpError.FileNotFound]),
                ]
            )
            ftp = MAVFTP(master, target_system=1, target_component=1)

            result = ftp.cmd_crccmp([os.path.join(temp_dir, "*.bin"), "/remote"])

            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(ftp.crccmp_results, ["MATCH", "DIFFER", "MISSING"])
            crc_requests = [
                request
                for request in self.sent_requests(master)
                if request.opcode == OP_CalcFileCRC32
            ]
            self.assertEqual(
                [request.payload for request in crc_requests],
                [b"/remote/a.bin", b"/remote/b.bin", b"/remote/c.bin"],
            )

    def test_crccmp_ignores_stale_crc_reply(self):
        """A delayed CRC reply cannot be reported for the active comparison."""
        with tempfile.NamedTemporaryFile(suffix=".bin") as local_file:
            local_file.write(b"crc regression")
            local_file.flush()
            crc = local_file_crc(local_file.name)
            master = StaleCRCReplyMaster(crc)
            ftp = MAVFTP(master, target_system=1, target_component=1)

            result = ftp.cmd_crccmp([local_file.name, "/remote"])

            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(ftp.crccmp_results, ["MATCH"])
            self.assertTrue(master.stale_sent)
            self.assertTrue(master.current_sent)

    def test_crccmp_rejects_duplicate_basenames_before_sending(self):
        """Comparing duplicate basenames is rejected before any CRC request."""
        with tempfile.TemporaryDirectory() as temp_dir:
            first_dir = os.path.join(temp_dir, "first")
            second_dir = os.path.join(temp_dir, "second")
            os.makedirs(first_dir)
            os.makedirs(second_dir)
            for directory in (first_dir, second_dir):
                with open(os.path.join(directory, "same.bin"), "wb") as local_file:
                    local_file.write(b"same")
            ftp, master = self.make_ftp([])

            result = ftp.cmd_crccmp([os.path.join(temp_dir, "*", "same.bin"), "/remote"])

            self.assertEqual(result.error_code, FtpError.InvalidArguments)
            self.assertEqual(len(master.mav.sent), 1)

    def test_crccmp_rejects_remote_names_that_would_be_truncated(self):
        """The comparison uses the server-safe filename limit, not the raw payload limit."""
        with tempfile.TemporaryDirectory() as temp_dir:
            name = "x" * (mavftp_module.MAX_FTP_NAME - len("/remote/")) + ".bin"
            path = os.path.join(temp_dir, name)
            with open(path, "wb") as local_file:
                local_file.write(b"too long")
            ftp, master = self.make_ftp([])

            result = ftp.cmd_crccmp([path, "/remote"])

            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertEqual(ftp.crccmp_results, ["ERROR"])
            self.assertEqual(len(master.mav.sent), 1)

    def test_rtt_adapts_retry_timeout_after_delayed_reply(self):
        """A delayed first reply increases the retry timeout for poor links."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_crc(["remote.bin"])
        baseline_timeout = ftp.retry_timeout()
        request_seq = ftp.last_op.seq
        ftp.send_times[request_seq] = time.time() - 2.0

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(request_seq + 1, OP_Ack, OP_CalcFileCRC32, payload=struct.pack("<I", 1))
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertTrue(ftp.rtt_valid)
        self.assertGreater(ftp.retry_timeout(), baseline_timeout)

    def test_batch_writes_are_split_at_network_mtu(self):
        """Several FTP packets are combined without creating an oversized datagram."""
        ftp, _master = self.make_ftp([])
        link = BatchLink()
        ftp.master.mav = BatchMAV(link)
        operations = [
            FTP_OP(ftp.seq, ftp.session, OP_WriteFile, 1, 0, 0, index, bytearray([index]))
            for index in range(8)
        ]

        ftp._MAVFTP__send_batch(operations)  # pylint: disable=protected-access

        self.assertGreater(len(link.writes), 1)
        self.assertTrue(all(len(write) <= mavftp_module.MAX_NETWORK_BATCH for write in link.writes))
        self.assertEqual(sum(len(write) for write in link.writes), 8 * 252)

    def test_new_features_are_exposed_by_cli(self):
        """The timestamp, CRC, and comparison options are parser-visible."""
        args = create_argument_parser().parse_args(
            ["--list_time", "1", "--crccmp_timeout", "8", "crccmp", "*.bin", "/remote"]
        )
        self.assertEqual(args.list_time, 1)
        self.assertEqual(args.crccmp_timeout, 8.0)
        self.assertEqual(args.command, "crccmp")
        self.assertEqual(args.arg1, "*.bin")
        self.assertEqual(args.arg2, "/remote")

        list_args = create_argument_parser().parse_args(["list"])
        self.assertEqual(list_args.list_time, 1)
        self.assertEqual(list_args.list_time_timeout, 3.0)
        self.assertEqual(list_args.list_retries, 3)

    def test_write_nack_preserves_server_error(self):
        """WriteFile NACKs retain their precise protocol error code."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
            FTP_OP(
                1,
                0,
                OP_Nack,
                2,
                OP_WriteFile,
                0,
                0,
                bytearray([FtpError.FailErrno, 13]),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.FailErrno)
        self.assertEqual(result.system_error, 13)
        self.assertEqual(terminated, [True])

    def test_gap_read_retry_reuses_request_sequence(self):
        """Retransmitting a lost ReadFile reply keeps the original sequence."""
        ftp, master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_gaps = [(0, 2)]
        ftp.read_gap_times = {(0, 2): 0}

        ftp._MAVFTP__send_gap_read((0, 2))  # pylint: disable=protected-access
        ftp._MAVFTP__send_gap_read((0, 2))  # pylint: disable=protected-access

        self.assertEqual(self.sent_request_sequences(master, OP_ReadFile), [1, 1])

    def test_open_retry_reuses_request_sequence(self):
        """Retransmitting an unanswered OpenFileRO preserves its sequence."""
        ftp, master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])
        ftp.op_start = 0

        with patch("pymavlink.mavftp.time.time", return_value=1):
            ftp._MAVFTP__idle_task()  # pylint: disable=protected-access

        self.assertEqual(self.sent_request_sequences(master, OP_OpenFileRO), [1, 1])

    def test_write_retry_reuses_request_sequence(self):
        """Retransmitting a lost WriteFile reply keeps the original sequence."""
        ftp, master = self.make_ftp([])
        ftp.fh = BytesIO(b"x")
        ftp.filename = "remote"
        ftp.write_list = {0}
        ftp.write_block_size = 1
        ftp.write_total = 1

        ftp._MAVFTP__send_more_writes()  # pylint: disable=protected-access
        ftp.write_last_send = 0
        ftp._MAVFTP__send_more_writes()  # pylint: disable=protected-access

        self.assertEqual(self.sent_request_sequences(master, OP_WriteFile), [1, 1])

    def test_process_timeout_terminates_active_session(self):
        """A reply-loop timeout closes an opened remote file session."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )
        setattr(
            ftp,
            "_MAVFTP__idle_task",
            lambda: False,
        )

        result = ftp.process_ftp_reply("get", timeout=0.03)

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)
        self.assertEqual(terminated, [True])

    def test_read_timeout_terminates_active_session(self):
        """The synchronous read API also closes its session on timeout."""
        ftp, _master = self.make_ftp([])
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        clock_calls = [0]

        def fake_time():
            clock_calls[0] += 1
            return 0 if clock_calls[0] <= 20 else 6

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            self.assertIsNone(ftp.read("remote", 1))

        self.assertEqual(terminated, [True])

    def test_read_sector_returns_only_requested_range_without_local_output(self):
        """A sector read starts at its offset and must not publish the remote path."""
        with tempfile.TemporaryDirectory() as tempdir:
            previous_cwd = os.getcwd()
            os.chdir(tempdir)
            try:
                ftp, master = self.make_ftp(
                    [
                        ftp_reply(
                            2,
                            OP_Ack,
                            OP_OpenFileRO,
                            payload=[8, 0, 0, 0],
                            session=7,
                        ),
                        ftp_reply(
                            3,
                            OP_Ack,
                            OP_BurstReadFile,
                            payload=b"defgh",
                            offset=3,
                            burst_complete=1,
                            session=7,
                        ),
                        ftp_reply(4, OP_Ack, OP_TerminateSession, session=7),
                    ]
                )

                self.assertEqual(ftp.read_sector("remote", 3, 2), b"de")
                self.assertFalse(os.path.exists("remote"))
                burst_request = next(
                    sent[-1]
                    for sent in master.mav.sent
                    if sent[-1][3] == OP_BurstReadFile
                )
                self.assertEqual(struct.unpack_from("<I", burst_request, 8)[0], 3)
            finally:
                os.chdir(previous_cwd)

    def test_read_sector_stops_after_requested_range_in_full_burst(self):
        """A full burst must not continue after a small range is satisfied."""
        ftp, master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_to_memory = True
        ftp.requested_offset = 0
        ftp.requested_size = 2
        ftp.op_start = 1
        ftp.burst_size = 80
        ftp.session = 7
        ftp.pending_burst_request = FTP_OP(
            seq=1,
            session=7,
            opcode=OP_BurstReadFile,
            size=80,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )

        result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                seq=2,
                session=7,
                opcode=OP_Ack,
                size=80,
                req_opcode=OP_BurstReadFile,
                burst_complete=1,
                offset=0,
                payload=bytearray(b"x" * 80),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertTrue(ftp.done)
        self.assertEqual(ftp.get_result, b"xx")
        self.assertEqual(master.mav.sent[-1][-1][3], OP_TerminateSession)
        self.assertNotIn(
            OP_BurstReadFile, [sent[-1][3] for sent in master.mav.sent[1:]]
        )

    def test_full_download_waits_for_eof_when_size_is_underestimated(self):
        """A full download must not finish at an estimated remote file size."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"data")
        ftp.filename = "-"
        ftp.requested_size = 4
        ftp.read_total = 4
        ftp.op_start = 1
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        self.assertFalse(ftp._MAVFTP__check_read_finished())

    def test_read_sector_memory_uses_range_relative_offset(self):
        """A range read buffer must scale with the range, not remote offset."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_to_memory = True
        ftp.requested_offset = 1024 * 1024

        ftp._MAVFTP__write_payload(  # pylint: disable=protected-access
            FTP_OP(
                seq=1,
                session=0,
                opcode=OP_Ack,
                size=2,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=ftp.requested_offset,
                payload=bytearray(b"xy"),
            )
        )

        self.assertEqual(ftp.fh.getvalue(), b"xy")

    def test_full_download_result_ignores_estimated_remote_size(self):
        """A full download publishes bytes received past the advertised size."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"x" * 200)
        ftp.filename = "-"
        ftp.requested_size = 80
        ftp.read_total = 200
        ftp.reached_eof = True
        ftp.op_start = 1
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        self.assertTrue(ftp._MAVFTP__check_read_finished())
        self.assertEqual(ftp.get_result, b"x" * 200)

    def test_read_sector_relative_buffer_preserves_remote_gap_offsets(self):
        """Compact buffers still track absolute remote offsets for gaps."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_to_memory = True
        ftp.requested_offset = 100
        ftp.requested_size = 4
        ftp.burst_size = 2
        ftp.op_start = 1
        ftp.session = 7
        ftp.pending_burst_request = FTP_OP(
            seq=1,
            session=7,
            opcode=OP_BurstReadFile,
            size=2,
            req_opcode=0,
            burst_complete=0,
            offset=100,
            payload=None,
        )

        first = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                seq=2,
                session=7,
                opcode=OP_Ack,
                size=2,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=102,
                payload=bytearray(b"cd"),
            ),
            None,
        )
        self.assertEqual(first.error_code, FtpError.Success)
        self.assertEqual(ftp.read_gaps, [(100, 2)])

        second = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                seq=3,
                session=7,
                opcode=OP_Ack,
                size=2,
                req_opcode=OP_BurstReadFile,
                burst_complete=1,
                offset=100,
                payload=bytearray(b"ab"),
            ),
            None,
        )
        self.assertEqual(second.error_code, FtpError.Success)
        self.assertEqual(ftp.get_result, b"abcd")

    def test_put_returns_after_completion_before_late_write_reply(self):
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_CreateFile),
                ftp_reply(3, OP_Ack, OP_WriteFile, offset=0),
                ftp_reply(4, OP_Ack, OP_TerminateSession),
                ftp_reply(3, OP_Ack, OP_WriteFile, offset=0),
            ]
        )

        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"))
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(len(master.replies), 1)

    def test_put_rejects_non_ascii_remote_name_before_claiming_file(self):
        """A rejected remote name must not leave upload state in progress."""
        ftp, master = self.make_ftp([])
        master.mav.sent.clear()

        with tempfile.NamedTemporaryFile() as local_file:
            result = ftp.cmd_put(
                [local_file.name, "r\N{LATIN SMALL LETTER E WITH ACUTE}mote"]
            )

        self.assertEqual(result.error_code, FtpError.InvalidArguments)
        self.assertIsNone(ftp.write_list)
        self.assertIsNone(ftp.fh)
        self.assertEqual(master.mav.sent, [])

        retry_result = ftp.cmd_put(["local", "remote"], fh=BytesIO(b"payload"))

        self.assertEqual(retry_result.error_code, FtpError.Success)

    def test_list_returns_after_eof_before_late_error(self):
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Nack, OP_ListDirectory, payload=[FtpError.EndOfFile]),
                ftp_reply(3, OP_Nack, OP_ListDirectory, payload=[FtpError.Fail]),
            ]
        )

        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(len(master.replies), 1)

    def test_stale_list_ack_does_not_resend_remove(self):
        ftp, master = self.make_ftp(
            [ftp_reply(2, OP_Nack, OP_ListDirectory, payload=[FtpError.EndOfFile])]
        )
        self.assertEqual(ftp.cmd_list([]).error_code, FtpError.Success)

        # A delayed list ACK arrives while waiting for RemoveFile. It must not
        # be dispatched to __handle_list_reply(), which would resend last_op
        # (the RemoveFile request) with a new sequence number.
        master.replies.extend(
            [
                ftp_reply(2, OP_Ack, OP_ListDirectory),
                ftp_reply(3, OP_Ack, OP_RemoveFile),
            ]
        )
        result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(master.replies, [])
        self.assertEqual(len(master.mav.sent), 3)

    def test_out_of_order_burst_reply_is_dispatched(self):
        ftp, _master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[81, 0, 0, 0]),
                ftp_reply(
                    3,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x" * 80,
                    burst_complete=1,
                ),
                # The next burst starts at offset 80. This delayed duplicate
                # from the completed burst must not reach its handler.
                ftp_reply(
                    3,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x" * 80,
                    burst_complete=1,
                ),
                ftp_reply(5, OP_Ack, OP_BurstReadFile, payload=b"y", offset=80, burst_complete=1),
                ftp_reply(6, OP_Ack, OP_TerminateSession),
            ]
        )

        ftp.cmd_get(
            ["remote", "-"],
            callback=lambda _fh: MAVFTPReturn("Get", FtpError.Success),
        )
        result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.duplicates, 0)

    def test_stale_burst_reply_sequence_is_discarded_for_reused_session(self):
        """A delayed burst packet must not match a new request in session 0."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.pending_burst_offset = 0
        ftp.pending_burst_seq = 11
        ftp.pending_burst_request = FTP_OP(
            seq=10,
            session=0,
            opcode=OP_BurstReadFile,
            size=80,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(
                10,
                OP_Ack,
                OP_BurstReadFile,
                payload=b"stale",
                offset=0,
                session=0,
            )
        )

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(ftp.fh.getvalue(), b"")

    def test_burst_reply_before_pending_offset_is_discarded(self):
        """A delayed reply must not repair a gap before the active burst."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"a" * 80 + b"\0" * 80 + b"b" * 80)
        ftp.fh.seek(240)
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 240
        ftp.read_total = 160
        ftp.op_start = 1
        ftp.reached_eof = True
        ftp.read_gaps = [(80, 80)]
        ftp.read_gap_times = {(80, 80): 123}
        ftp.pending_burst_offset = 240
        ftp.pending_burst_seq = 2
        ftp.pending_burst_request = FTP_OP(1, 0, OP_BurstReadFile, 80, 0, 0, 240, None)

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_BurstReadFile, payload=b"s" * 80, offset=80)
        )

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(ftp.fh.getvalue(), b"a" * 80 + b"\0" * 80 + b"b" * 80)
        self.assertEqual(ftp.fh.tell(), 240)
        self.assertEqual(ftp.read_gaps, [(80, 80)])
        self.assertEqual(ftp.read_gap_times, {(80, 80): 123})
        self.assertEqual(ftp.read_total, 160)
        self.assertEqual(ftp.duplicates, 0)
        self.assertFalse(ftp.read_complete)

    def test_burst_reply_requires_pending_offset(self):
        """Burst packets are ignored until a request establishes its offset."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.pending_burst_seq = 2

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_BurstReadFile, payload=b"stale", offset=0)
        )

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(ftp.fh.getvalue(), b"")

    def test_out_of_order_replies_in_one_burst_fill_the_gap(self):
        """Burst reply sequencing is a floor, not a per-reply ratchet."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 240
        ftp.burst_size = 80
        ftp.op_start = 1
        ftp.pending_burst_offset = 0
        ftp.pending_burst_seq = 2
        ftp.pending_burst_request = FTP_OP(
            seq=1,
            session=0,
            opcode=OP_BurstReadFile,
            size=80,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )

        for seq, offset, payload in (
            (2, 0, b"a" * 80),
            (4, 160, b"c" * 80),
            (3, 80, b"b" * 80),
        ):
            result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(seq, OP_Ack, OP_BurstReadFile, payload=payload, offset=offset)
            )
            self.assertEqual(result.error_code, FtpError.Success)

        self.assertEqual(ftp.read_gaps, [])
        self.assertEqual(ftp.get_result, b"a" * 80 + b"b" * 80 + b"c" * 80)

    def test_long_burst_advances_its_trailing_sequence_floor(self):
        """A burst longer than half the uint16 sequence space remains accepted."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 32770
        ftp.burst_size = 1
        ftp.op_start = 1
        ftp._MAVFTP__send(  # pylint: disable=protected-access
            FTP_OP(
                seq=ftp.seq,
                session=0,
                opcode=OP_BurstReadFile,
                size=1,
                req_opcode=0,
                burst_complete=0,
                offset=0,
                payload=None,
            )
        )

        for sequence in range(2, 32771):
            result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(
                    sequence,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x",
                    offset=sequence - 2,
                )
            )
            self.assertEqual(result.error_code, FtpError.Success)

        self.assertEqual(ftp.fh.getvalue(), b"x" * 32769)
        self.assertLessEqual(
            (32770 - ftp.pending_burst_seq) & 0xFFFF,
            BURST_REPLY_SEQUENCE_WINDOW,
        )

    def test_burst_accepts_replies_across_uint16_sequence_wrap(self):
        """A burst remains accepted when reply sequence numbers wrap."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 21
        ftp.burst_size = 1
        ftp.op_start = 1
        ftp.seq = 65530
        ftp._MAVFTP__send(  # pylint: disable=protected-access
            FTP_OP(
                seq=ftp.seq,
                session=0,
                opcode=OP_BurstReadFile,
                size=1,
                req_opcode=0,
                burst_complete=0,
                offset=0,
                payload=None,
            )
        )

        for offset in range(20):
            result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(
                    (65531 + offset) & 0xFFFF,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x",
                    offset=offset,
                )
            )
            self.assertEqual(result.error_code, FtpError.Success)

        self.assertEqual(ftp.fh.getvalue(), b"x" * 20)

    def test_retry_straggler_is_repairable_until_expected_reply(self):
        """A retry writes stragglers without letting them change its floor."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 80
        ftp.burst_size = 40
        ftp.op_start = 1
        request = FTP_OP(
            seq=1,
            session=0,
            opcode=OP_BurstReadFile,
            size=40,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )
        ftp._MAVFTP__send(request)  # pylint: disable=protected-access
        ftp._MAVFTP__send(request, retry=True)  # pylint: disable=protected-access

        straggler = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(5000, OP_Ack, OP_BurstReadFile, payload=b"b" * 40, offset=40)
        )
        self.assertEqual(straggler.error_code, FtpError.Success)
        self.assertTrue(ftp.pending_burst_retry)
        self.assertEqual(ftp.pending_burst_seq, 2)
        self.assertEqual(ftp.seq, 2)
        self.assertEqual(ftp.read_gaps, [(0, 40)])
        first_restarted = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_BurstReadFile, payload=b"a" * 40, offset=0)
        )
        self.assertEqual(first_restarted.error_code, FtpError.Success)
        self.assertFalse(ftp.pending_burst_retry)
        self.assertEqual(ftp.read_gaps, [])
        self.assertEqual(ftp.get_result, b"a" * 40 + b"b" * 40)

    def test_retry_first_reply_loss_leaves_a_repairable_gap(self):
        """Later restarted packets remain usable if the first reply is lost."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 120
        ftp.burst_size = 40
        ftp.op_start = 1
        request = FTP_OP(1, 0, OP_BurstReadFile, 40, 0, 0, 0, None)
        ftp._MAVFTP__send(request)  # pylint: disable=protected-access
        ftp._MAVFTP__send(request, retry=True)  # pylint: disable=protected-access

        for seq, offset, payload in ((3, 40, b"b" * 40), (4, 80, b"c" * 40)):
            result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(seq, OP_Ack, OP_BurstReadFile, payload=payload, offset=offset)
            )
            self.assertEqual(result.error_code, FtpError.Success)

        self.assertTrue(ftp.pending_burst_retry)
        self.assertEqual(ftp.pending_burst_seq, 2)
        self.assertEqual(ftp.read_gaps, [(0, 40)])
        self.assertEqual(ftp.fh.getvalue(), b"\0" * 40 + b"b" * 40 + b"c" * 40)

    def test_retry_with_lost_first_reply_advances_long_stream_floor(self):
        """A lost retry reply cannot make a stream fail after 32767 packets."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 32770
        ftp.burst_size = 1
        ftp.op_start = 1
        request = FTP_OP(1, 0, OP_BurstReadFile, 1, 0, 0, 0, None)
        ftp._MAVFTP__send(request)  # pylint: disable=protected-access
        ftp._MAVFTP__send(request, retry=True)  # pylint: disable=protected-access

        for sequence in range(3, 32772):
            result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(
                    sequence,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x",
                    offset=sequence - 2,
                )
            )
            self.assertEqual(result.error_code, FtpError.Success)

        self.assertLessEqual(
            (32771 - ftp.pending_burst_seq) & 0xFFFF,
            BURST_REPLY_SEQUENCE_WINDOW,
        )

    def test_retry_stragglers_do_not_poison_floor_after_first_reply(self):
        """A restarted burst retains its next expected reply after stragglers."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_size = 120
        ftp.burst_size = 40
        ftp.op_start = 1
        request = FTP_OP(1, 0, OP_BurstReadFile, 40, 0, 0, 0, None)
        ftp._MAVFTP__send(request)  # pylint: disable=protected-access
        ftp._MAVFTP__send(request, retry=True)  # pylint: disable=protected-access

        self.assertEqual(
            ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(2, OP_Ack, OP_BurstReadFile, payload=b"a" * 40, offset=0)
            ).error_code,
            FtpError.Success,
        )
        for seq, offset in ((5000, 80), (5001, 120)):
            self.assertEqual(
                ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                    ftp_reply(seq, OP_Ack, OP_BurstReadFile, payload=b"x" * 40, offset=offset)
                ).error_code,
                FtpError.Success,
            )

        self.assertEqual(ftp.pending_burst_seq, 2)
        self.assertEqual(
            ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"b" * 40, offset=40)
            ).error_code,
            FtpError.Success,
        )

    def test_burst_reply_rejects_unbounded_gap_allocation(self):
        """A far-ahead burst reply must not allocate unbounded read gaps."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.burst_size = 239
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)
        far_offset = (MAX_READ_GAPS + 1) * ftp.burst_size

        result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                seq=1,
                session=0,
                opcode=OP_Ack,
                size=1,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=far_offset,
                payload=bytearray(b"x"),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)
        self.assertEqual(ftp.read_gaps, [])

    def test_burst_reply_rejects_cumulative_gap_allocation(self):
        """Successive far-ahead replies must respect the total gap bound."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.burst_size = 239
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)
        first_offset = MAX_READ_GAPS * ftp.burst_size

        first_result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                seq=1,
                session=0,
                opcode=OP_Ack,
                size=1,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=first_offset,
                payload=bytearray(b"x"),
            ),
            None,
        )
        second_result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                seq=2,
                session=0,
                opcode=OP_Ack,
                size=1,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=first_offset + 1 + (MAX_READ_GAPS * ftp.burst_size),
                payload=bytearray(b"x"),
            ),
            None,
        )

        self.assertEqual(first_result.error_code, FtpError.Success)
        self.assertEqual(second_result.error_code, FtpError.InvalidDataSize)
        self.assertEqual(len(ftp.read_gaps), MAX_READ_GAPS)

    def test_stalled_burst_retries_from_current_read_position(self):
        """A stalled burst retains its sequence but resumes at the next byte."""
        ftp, master = self.make_ftp([])
        ftp.fh = BytesIO(b"x" * 120)
        ftp.fh.seek(80)
        ftp.filename = "-"
        ftp.last_burst_read = 10
        ftp.pending_burst_seq = 501
        ftp.pending_burst_request = FTP_OP(
            seq=17,
            session=0,
            opcode=OP_BurstReadFile,
            size=40,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )
        next_request_sequence = ftp.seq

        with patch("pymavlink.mavftp.time.time", return_value=11):
            ftp._MAVFTP__idle_task()

        request = master.mav.sent[-1][-1]
        self.assertEqual(request[3], OP_BurstReadFile)
        self.assertEqual(struct.unpack_from("<H", request)[0], 17)
        self.assertEqual(struct.unpack_from("<I", request, 8)[0], 80)
        self.assertEqual(ftp.pending_burst_offset, 80)
        self.assertEqual(ftp.pending_burst_seq, 18)
        self.assertEqual(ftp.seq, next_request_sequence)

    def test_stalled_range_burst_retry_uses_absolute_remote_offset(self):
        """A range retry includes the requested remote offset in its resume point."""
        ftp, master = self.make_ftp([])
        ftp.fh = BytesIO(b"x" * 120)
        ftp.fh.seek(80)
        ftp.filename = "-"
        ftp.read_to_memory = True
        ftp.requested_offset = 1000
        ftp.last_burst_read = 10
        ftp.pending_burst_seq = 501
        ftp.pending_burst_request = FTP_OP(
            seq=17,
            session=0,
            opcode=OP_BurstReadFile,
            size=40,
            req_opcode=0,
            burst_complete=0,
            offset=1000,
            payload=None,
        )

        with patch("pymavlink.mavftp.time.time", return_value=11):
            ftp._MAVFTP__idle_task()

        request = master.mav.sent[-1][-1]
        self.assertEqual(struct.unpack_from("<I", request, 8)[0], 1080)

    def test_out_of_order_gap_reply_is_dispatched(self):
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_gaps = [(0, 2), (2, 2)]
        ftp.read_gap_times = {(0, 2): 0, (2, 2): 0}

        ftp._MAVFTP__send_gap_read((0, 2))  # pylint: disable=protected-access
        ftp._MAVFTP__send_gap_read((2, 2))  # pylint: disable=protected-access

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(3, OP_Ack, OP_ReadFile, payload=b"cd", offset=2)
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.read_gaps, [(0, 2)])
        self.assertEqual(ftp.fh.getvalue(), b"\x00\x00cd")

        stale_result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(99, OP_Ack, OP_ReadFile, payload=b"zz", offset=0)
        )

        self.assertEqual(stale_result.error_code, FtpError.Fail)
        self.assertEqual(ftp.read_gaps, [(0, 2)])
        self.assertEqual(ftp.fh.getvalue(), b"\x00\x00cd")

    def test_out_of_order_final_gap_reply_reports_success(self):
        """A successful final gap repair completes a read when it is not last_op."""
        ftp, master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.op_start = 1
        ftp.requested_size = 240
        ftp.burst_size = 239
        ftp.reached_eof = True
        ftp.read_gaps = [(0, 120), (120, 120)]
        ftp.read_gap_times = {(0, 120): 0, (120, 120): 0}

        ftp._MAVFTP__send_gap_read((0, 120))
        ftp._MAVFTP__send_gap_read((120, 120))
        # A burst request was sent after the gap requests, so neither gap
        # reply matches last_op even though both remain active requests.
        ftp._MAVFTP__send(
            FTP_OP(
                ftp.seq,
                ftp.session,
                OP_BurstReadFile,
                239,
                0,
                0,
                240,
                None,
            )
        )
        master.replies.extend(
            [
                ftp_reply(3, OP_Ack, OP_ReadFile, payload=b"b" * 120, offset=120),
                ftp_reply(2, OP_Ack, OP_ReadFile, payload=b"a" * 120, offset=0),
                ftp_reply(5, OP_Ack, OP_TerminateSession),
            ]
        )

        result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertTrue(ftp.read_complete)
        self.assertEqual(ftp.read_gaps, [])
        self.assertEqual(ftp.get_result, b"a" * 120 + b"b" * 120)

    def test_stale_write_reply_is_discarded(self):
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_CreateFile),
                ftp_reply(99, OP_Ack, OP_WriteFile, offset=0),
            ]
        )

        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"))
        ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            master.replies.pop(0)
        )
        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            master.replies.pop(0)
        )

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertIsNotNone(ftp.write_list)
        self.assertEqual(ftp.write_acks, 0)

    def test_empty_put_reports_complete_progress(self):
        """An empty upload completes from CreateFile without a WriteFile ACK."""
        ftp, master = self.make_ftp([])
        progress = []

        ftp.cmd_put(
            ["local", "remote"],
            fh=BytesIO(),
            progress_callback=progress.append,
        )
        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_CreateFile)
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(progress, [1.0])
        self.assertEqual(ftp.write_total, 0)
        self.assertEqual(self.sent_request_sequences(master, OP_WriteFile), [])

    def test_noncurrent_write_nack_fails_upload(self):
        ftp, _master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_CreateFile),
                ftp_reply(
                    3,
                    OP_Nack,
                    OP_WriteFile,
                    payload=[FtpError.FileProtected],
                    offset=0,
                ),
                ftp_reply(5, OP_Ack, OP_TerminateSession),
            ]
        )

        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x" * 160))
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.FileProtected)

    def test_remove_accepts_16_bit_sequence_wrap(self):
        ftp, master = self.make_ftp([])
        ftp.seq = FTP_SEQ_MODULUS - 1
        master.replies.append(ftp_reply(0, OP_Ack, OP_RemoveFile))

        result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.Success)

    def test_remove_sequence_255_advances_to_256(self):
        """A 16-bit sequence continues from 255 to 256 without wrapping."""
        ftp, master = self.make_ftp([])
        ftp.seq = 255
        master.replies.append(ftp_reply(256, OP_Ack, OP_RemoveFile))

        result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.Success)

    def test_ftp_wrap_moduli_match_protocol_field_widths(self):
        self.assertEqual(FTP_SEQ_MODULUS, 65536)
        self.assertEqual(FTP_SESSION_MODULUS, 256)

    def test_cancel_wraps_session_255_to_0(self):
        """A completed session advances from 255 to the protocol's zero ID."""
        ftp, master = self.make_ftp([])
        ftp.session = FTP_SESSION_MODULUS - 1
        master.replies.append(
            ftp_reply(2, OP_Ack, OP_TerminateSession, session=ftp.session)
        )

        result = ftp.cmd_cancel()

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.session, 0)

    def test_wrong_session_reply_is_not_retained(self):
        ftp, master = self.make_ftp([])
        master.replies.append(ftp_reply(2, OP_Ack, OP_RemoveFile, session=1))

        result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.Fail)

    def test_completed_put_skips_late_reply_after_termination_timeout(self):
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_WriteFile),
                ftp_reply(3, OP_Ack, OP_WriteFile),
            ]
        )
        ftp.pending_terminate_seq = 7

        def complete_operation(_message):
            ftp.completed_reply = (OP_WriteFile, 6)
            return MAVFTPReturn("WriteFile", FtpError.Success)

        setattr(ftp, "_MAVFTP__mavlink_packet", complete_operation)
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(len(master.replies), 1)

    def test_incomplete_burst_read_reports_timeout_on_idle(self):
        ftp, _master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[160, 0, 0, 0]),
                ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"x" * 80),
            ]
        )

        ftp.cmd_get(
            ["remote"],
            callback=lambda _fh: MAVFTPReturn("Get", FtpError.Success),
        )
        result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)
        self.assertIsNone(ftp.get_result)

    def test_callback_exception_terminates_upload(self):
        """Upload callback exceptions are reported after session cleanup."""
        ftp, _master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_CreateFile),
                ftp_reply(3, OP_Ack, OP_WriteFile, offset=0),
                ftp_reply(4, OP_Ack, OP_TerminateSession),
            ]
        )

        def failing_callback(_size):
            raise RuntimeError("upload callback failed")

        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"), callback=failing_callback)
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.write_list)

    def test_upload_progress_exception_terminates_upload(self):
        """Progress callback exceptions clean up and report failure."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_CreateFile),
                ftp_reply(3, OP_Ack, OP_WriteFile, offset=0),
                ftp_reply(4, OP_Ack, OP_TerminateSession),
            ]
        )

        def failing_progress(_progress):
            raise RuntimeError("progress failed")

        ftp.cmd_put(
            ["local", "remote"],
            fh=BytesIO(b"x"),
            progress_callback=failing_progress,
        )
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.write_list)
        self.assertIsNone(ftp.put_callback_progress)
        self.assertEqual(master.replies, [])

    def test_new_upload_clears_stale_callback_failure(self):
        """Starting a new upload clears a previous callback failure."""
        ftp, _master = self.make_ftp([])
        ftp.callback_failure = MAVFTPReturn("Put", FtpError.Fail)

        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"))

        self.assertIsNone(ftp.callback_failure)
        ftp.fh = None
        ftp.write_list = None

    def test_main_closes_transport_when_command_fails(self):
        """CLI failures cancel active FTP and close the MAVLink transport."""
        args = Namespace(
            loglevel="INFO",
            device="/dev/test",
            baudrate=115200,
            source_system=250,
            debug=0,
            list_time=0,
            list_time_timeout=3.0,
            list_retries=3,
            pkt_loss_tx=0,
            pkt_loss_rx=0,
            max_backlog=5,
            burst_read_size=80,
            write_size=80,
            write_qsize=5,
            idle_detection_time=3.7,
            read_retry_time=1.0,
            retry_time=0.5,
            command="status",
            arg1=None,
            arg2=None,
        )
        master = MagicMock(target_system=1, target_component=1)
        mav_ftp = MagicMock()
        mav_ftp._MAVFTP__has_active_session.return_value = True
        mav_ftp.cmd_ftp.side_effect = RuntimeError("command failed")

        with (
            patch.object(mavftp_module, "create_argument_parser") as create_parser,
            patch.object(
                mavftp_module,
                "auto_connect",
                return_value=Namespace(device="/dev/test"),
            ),
            patch.object(
                mavftp_module.mavutil,
                "mavlink_connection",
                return_value=master,
            ),
            patch.object(mavftp_module, "wait_heartbeat"),
            patch.object(mavftp_module, "MAVFTP", return_value=mav_ftp),
        ):
            create_parser.return_value.parse_args.return_value = args
            with self.assertRaises(RuntimeError):
                mavftp_module.main()

        mav_ftp.cmd_cancel.assert_called_once_with()
        master.close.assert_called_once_with()

    def test_main_forwards_getparams_output_options(self):
        """CLI getparams options reach the parameter-output implementation."""
        args = Namespace(
            loglevel="INFO",
            device="/dev/test",
            baudrate=115200,
            source_system=250,
            debug=0,
            list_time=0,
            list_time_timeout=3.0,
            list_retries=3,
            pkt_loss_tx=0,
            pkt_loss_rx=0,
            max_backlog=5,
            burst_read_size=80,
            write_size=80,
            write_qsize=5,
            idle_detection_time=3.7,
            read_retry_time=1.0,
            retry_time=0.5,
            command="getparams",
            arg1="values.param",
            arg2="defaults.param",
            sort="mavproxy",
            add_datatype_comments=True,
            add_timestamp_comment=True,
        )
        master = MagicMock(target_system=1, target_component=1)
        mav_ftp = MagicMock()
        mav_ftp._MAVFTP__has_active_session.return_value = False
        mav_ftp.cmd_getparams.return_value = MAVFTPReturn("GetParams", FtpError.Success)
        mav_ftp.process_ftp_reply.return_value = MAVFTPReturn(
            "GetParams", FtpError.Success
        )

        with (
            patch.object(mavftp_module, "create_argument_parser") as create_parser,
            patch.object(
                mavftp_module,
                "auto_connect",
                return_value=Namespace(device="/dev/test"),
            ),
            patch.object(
                mavftp_module.mavutil,
                "mavlink_connection",
                return_value=master,
            ),
            patch.object(mavftp_module, "wait_heartbeat"),
            patch.object(mavftp_module, "MAVFTP", return_value=mav_ftp),
        ):
            create_parser.return_value.parse_args.return_value = args
            with self.assertRaises(SystemExit) as exit_context:
                mavftp_module.main()

        self.assertEqual(exit_context.exception.code, 0)
        mav_ftp.cmd_getparams.assert_called_once_with(
            ["values.param", "defaults.param"],
            sort_type="mavproxy",
            add_datatype_comments=True,
            add_timestamp_comment=True,
        )

    def test_main_reports_invalid_settings_as_argument_error(self):
        """Invalid cross-setting CLI values are reported through the parser."""
        args = Namespace(
            loglevel="INFO",
            device="/dev/test",
            baudrate=115200,
            source_system=250,
            debug=0,
            list_time=0,
            list_time_timeout=3.0,
            list_retries=3,
            pkt_loss_tx=0,
            pkt_loss_rx=0,
            max_backlog=5,
            burst_read_size=80,
            write_size=80,
            write_qsize=5,
            idle_detection_time=1.0,
            read_retry_time=1.0,
            retry_time=0.5,
            command="status",
            arg1=None,
            arg2=None,
        )
        parser = MagicMock()
        parser.parse_args.return_value = args
        parser.error.side_effect = SystemExit(2)

        with (
            patch.object(mavftp_module, "create_argument_parser", return_value=parser),
            patch.object(
                mavftp_module,
                "auto_connect",
                return_value=Namespace(device="/dev/test"),
            ),
            patch.object(
                mavftp_module.mavutil,
                "mavlink_connection",
                return_value=MagicMock(target_system=1, target_component=1),
            ),
            patch.object(mavftp_module, "wait_heartbeat"),
        ):
            with self.assertRaises(SystemExit) as exit_context:
                mavftp_module.main()

        self.assertEqual(exit_context.exception.code, 2)
        parser.error.assert_called_once()

    def test_cancel_reports_termination_failure_when_remote_does_not_ack(self):
        """Cancel retries termination and reports a missing acknowledgement."""
        ftp, master = self.make_ftp([])
        ftp.session = 6
        ftp.fh = BytesIO(b"partial")
        ftp.filename = "remote.bin"

        result = ftp.cmd_cancel()

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(
            [request.opcode for request in self.sent_requests(master)],
            [OP_ResetSessions, OP_TerminateSession, OP_TerminateSession],
        )
        self.assertIsNone(ftp.pending_terminate_seq)
        self.assertEqual(ftp.session, 7)

    def test_callback_failure_does_not_publish_download(self):
        """Regression: a failing callback must not publish its temporary download."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = f"{tempdir}/param.pck"
            ftp, _master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[3, 0, 0, 0]),
                    ftp_reply(
                        3,
                        OP_Ack,
                        OP_BurstReadFile,
                        payload=b"bad",
                        burst_complete=1,
                    ),
                    ftp_reply(4, OP_Ack, OP_TerminateSession),
                ]
            )

            ftp.cmd_get(
                ["@PARAM/param.pck", destination],
                callback=lambda _fh: MAVFTPReturn("GetParams", FtpError.Fail),
            )
            result = ftp.process_ftp_reply("getparams", timeout=1)

            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertFalse(os.path.exists(destination))

    def test_callback_exception_terminates_download(self):
        """Callback exceptions are reported as FTP failures after session cleanup."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"data")
        ftp.filename = "-"
        ftp.op_start = 1
        ftp.requested_size = 4
        ftp.read_total = 4
        ftp.reached_eof = True
        terminated = []
        setattr(ftp, "_MAVFTP__terminate_session", lambda: terminated.append(True))

        def failing_callback(_fh):
            raise RuntimeError("decode failed")

        ftp.callback = failing_callback

        self.assertTrue(ftp._MAVFTP__check_read_finished())
        self.assertEqual(terminated, [True])
        self.assertIsNotNone(ftp.callback_failure)
        self.assertEqual(ftp.callback_failure.error_code, FtpError.Fail)

    def test_callback_success_does_not_publish_download(self):
        """Regression: callbacks consume all four advertised bytes without publishing."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = f"{tempdir}/param.pck"
            callback_data = []
            ftp, _master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[4, 0, 0, 0]),
                    ftp_reply(
                        3,
                        OP_Ack,
                        OP_BurstReadFile,
                        payload=b"data",
                        burst_complete=1,
                    ),
                    ftp_reply(4, OP_Ack, OP_TerminateSession),
                ]
            )

            def callback(fh):
                callback_data.append(fh.read())
                return MAVFTPReturn("GetParams", FtpError.Success)

            ftp.cmd_get(
                ["@PARAM/param.pck?withdefaults=1", destination],
                callback=callback,
            )
            result = ftp.process_ftp_reply("getparams", timeout=1)

            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(callback_data, [b"data"])
            self.assertFalse(os.path.exists(destination))

    def test_malformed_burst_nacks_are_decoded(self):
        for payload, expected_error in (
            (b"", FtpError.NoErrorCodeInPayload),
            (b"\xff", FtpError.InvalidErrorCode),
        ):
            with self.subTest(payload=payload):
                ftp, master = self.make_ftp(
                    [
                        ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[1, 0, 0, 0]),
                        ftp_reply(3, OP_Nack, OP_BurstReadFile, payload=payload),
                        ftp_reply(4, OP_Ack, OP_TerminateSession),
                    ]
                )
                ftp.cmd_get(
                    ["remote", "-"],
                    callback=lambda _fh: MAVFTPReturn("Get", FtpError.Success),
                )
                result = ftp.process_ftp_reply("get", timeout=1)

                self.assertEqual(result.error_code, expected_error)
                self.assertEqual(master.replies, [])


class TestMAVFTPParamDecode(unittest.TestCase):
    """Validate packed parameter name constraints."""

    @staticmethod
    def packed_param(name):
        # A float parameter with one name component and no defaults.
        header = struct.pack("<HHH", 0x671B, 1, 1)
        record = (
            struct.pack("<BB", 4, (len(name) - 1) << 4)
            + name
            + struct.pack("<f", 1.0)
        )
        return header + record

    def test_rejects_non_utf8_name(self):
        with self.assertLogs(level="ERROR") as logs:
            self.assertIsNone(MAVFTP.ftp_param_decode(self.packed_param(b"bad\xff")))
        self.assertIn("parameter name is not valid UTF-8", logs.output[0])

    def test_rejects_count_larger_than_total(self):
        first = self.packed_param(b"PARAM_A")[6:]
        second = self.packed_param(b"PARAM_B")[6:]
        data = struct.pack("<HHH", 0x671B, 2, 1) + first + second

        with self.assertLogs(level="ERROR") as logs:
            self.assertIsNone(MAVFTP.ftp_param_decode(data))

        self.assertIn("parameter count 2 exceeds total count 1", logs.output[0])

    def test_rejects_name_longer_than_16_bytes(self):
        header = struct.pack("<HHH", 0x671B, 2, 2)
        first = struct.pack("<BB", 4, 15 << 4) + b"A" * 16 + struct.pack("<f", 1.0)
        # Reuse 15 bytes of the previous name and append two bytes.
        second = struct.pack("<BB", 4, (1 << 4) | 15) + b"BC" + struct.pack("<f", 1.0)
        with self.assertLogs(level="ERROR") as logs:
            self.assertIsNone(MAVFTP.ftp_param_decode(header + first + second))
        self.assertIn("parameter name is too long", logs.output[0])

    def test_save_params_accepts_integer_and_string_datatype_ids(self):
        """Both public save_params datatype representations produce valid comments."""
        with tempfile.TemporaryDirectory() as tempdir:
            for datatype in (4, "4"):
                with self.subTest(datatype=datatype):
                    output = f"{tempdir}/params-{datatype}.txt"
                    MAVFTP.save_params(
                        {"TEST_PARAM": (1.0, datatype)},
                        output,
                        "missionplanner",
                        add_datatype_comments=True,
                        add_timestamp_comment=False,
                    )

                    with open(output, encoding="utf-8") as param_file:
                        self.assertEqual(
                            param_file.read(), "TEST_PARAM,1  # 32-bit float\n"
                        )


class TestMAVFTPPayloadDecoding(unittest.TestCase):
    """Test MAVFTP payload decoding"""

    def setUp(self):
        self.log_stream = StringIO()
        handler = logging.StreamHandler(self.log_stream)
        formatter = logging.Formatter('%(levelname)s: %(message)s')
        handler.setFormatter(formatter)
        logger = logging.getLogger()
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

        # Mock mavutil.mavlink_connection to simulate a connection
        self.mock_master = mavutil.mavlink_connection(device="udp:localhost:14550", source_system=1)

        # Initialize MAVFTP instance for testing
        self.mav_ftp = MAVFTP(self.mock_master, target_system=1, target_component=1)

    def tearDown(self):
        # Release the UDP socket so the next test can re-bind the port.
        self.mock_master.close()
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

    def test_logging(self):
        # Code that triggers logging
        logging.info("This is a test log message")

        # Flush and get log output
        log_output = self.log_stream.getvalue()

        # Assert to check if the expected log is in log_output
        self.assertIn("This is a test log message", log_output)

    @staticmethod
    def ftp_operation(seq: int, opcode: int, req_opcode: int, payload: bytearray) -> FTP_OP:
        return FTP_OP(seq=seq, session=1, opcode=opcode, size=0, req_opcode=req_opcode, burst_complete=0, offset=0,
                      payload=payload)

    def test_decode_ftp_ack_and_nack(self):
        # Test cases grouped by expected outcome
        # pylint: disable=line-too-long
        test_cases = [
            {
                "name": "Successful Operation",
                "op": self.ftp_operation(seq=1, opcode=OP_Ack, req_opcode=OP_ListDirectory, payload=None),
                "expected_message": "ListDirectory succeeded"
            },
            {
                "name": "Generic Failure",
                "op": self.ftp_operation(seq=2, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.Fail])),
                "expected_message": "ListDirectory failed, generic error"
            },
            {
                "name": "System Error",
                "op": self.ftp_operation(seq=3, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FailErrno, 1])),  # System error 1
                "expected_message": "ListDirectory failed, system error 1"
            },
            {
                "name": "Invalid Data Size",
                "op": self.ftp_operation(seq=4, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.InvalidDataSize])),
                "expected_message": "ListDirectory failed, invalid data size"
            },
            {
                "name": "Invalid Session",
                "op": self.ftp_operation(seq=5, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.InvalidSession])),
                "expected_message": "ListDirectory failed, session is not currently open"
            },
            {
                "name": "No Sessions Available",
                "op": self.ftp_operation(seq=6, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.NoSessionsAvailable])),
                "expected_message": "ListDirectory failed, no sessions available"
            },
            {
                "name": "End of File",
                "op": self.ftp_operation(seq=7, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.EndOfFile])),
                "expected_message": "ListDirectory failed, offset past end of file"
            },
            {
                "name": "Unknown Command",
                "op": self.ftp_operation(seq=8, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.UnknownCommand])),
                "expected_message": "ListDirectory failed, unknown command"
            },
            {
                "name": "File Exists",
                "op": self.ftp_operation(seq=9, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FileExists])),
                "expected_message": "ListDirectory failed, file/directory already exists"
            },
            {
                "name": "File Protected",
                "op": self.ftp_operation(seq=10, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FileProtected])),
                "expected_message": "ListDirectory failed, file/directory is protected"
            },
            {
                "name": "File Not Found",
                "op": self.ftp_operation(seq=11, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FileNotFound])),
                "expected_message": "ListDirectory failed, file/directory not found"
            },
            {
                "name": "No Error Code in Payload",
                "op": self.ftp_operation(seq=12, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=None),
                "expected_message": "ListDirectory failed, payload contains no error code"
            },
            {
                "name": "No Error Code in Nack",
                "op": self.ftp_operation(seq=13, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.Success])),
                "expected_message": "ListDirectory failed, no error code"
            },
            {
                "name": "No Filesystem Error in Payload",
                "op": self.ftp_operation(seq=14, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FailErrno])),
                "expected_message": "ListDirectory failed, file-system error missing in payload"
            },
            {
                "name": "Invalid Error Code",
                "op": self.ftp_operation(seq=15, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.InvalidErrorCode])),
                "expected_message": "ListDirectory failed, invalid error code"
            },
            {
                "name": "Payload Too Large",
                "op": self.ftp_operation(seq=16, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([0, 0, 0])),
                "expected_message": "ListDirectory failed, payload is too long"
            },
            {
                "name": "Invalid Opcode",
                "op": self.ftp_operation(seq=17, opcode=126, req_opcode=OP_ListDirectory, payload=None),
                "expected_message": "ListDirectory failed, invalid opcode 126"
            },
            {
                "name": "Unknown Opcode in Request",
                "op": self.ftp_operation(seq=19, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.UnknownCommand])),  # Assuming 100 is an unknown opcode
                "expected_message": "ListDirectory failed, unknown command"
            },
            {
                "name": "Payload with System Error",
                "op": self.ftp_operation(seq=20, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FailErrno, 2])),  # System error 2
                "expected_message": "ListDirectory failed, system error 2"
            },
            {
                "name": "Invalid Error Code in Payload",
                "op": self.ftp_operation(seq=21, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([105])),  # Assuming 105 is an invalid error code
                "expected_message": "ListDirectory failed, invalid error code 105"
            },
            {
                "name": "Invalid Opcode with Payload",
                "op": self.ftp_operation(seq=23, opcode=126, req_opcode=OP_ReadFile, payload=bytes([1, 1])),  # Invalid opcode with payload
                "expected_message": "ReadFile failed, invalid opcode 126"
            },
            # Add more test cases as needed...
        ]
        # pylint: enable=line-too-long

        for case in test_cases:
            ret = self.mav_ftp._MAVFTP__decode_ftp_ack_and_nack(case['op'])  # pylint: disable=protected-access
            ret.display_message()
            log_output = self.log_stream.getvalue().strip()
            self.assertIn(case["expected_message"], log_output,
                          f"Test {case['name']}: Expected {case['expected_message']} but got {log_output}")
            self.log_stream.seek(0)
            self.log_stream.truncate(0)

        # Invalid Arguments
        ret = MAVFTPReturn("Command arguments", FtpError.InvalidArguments)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Command arguments failed, invalid arguments", log_output, "Expected invalid arguments message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Test for unknown error code in display_message
        op = self.ftp_operation(seq=22, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([255]))
        ret = self.mav_ftp._MAVFTP__decode_ftp_ack_and_nack(op, "ListDirectory")  # pylint: disable=protected-access
        ret.error_code = 125  # Set error code to 125 to trigger unknown error message
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("ListDirectory failed, unknown error 125 in display_message()", log_output,
                      "Expected unknown error message for unknown error code")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Put already in progress
        ret = MAVFTPReturn("Put", FtpError.PutAlreadyInProgress)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Put failed, put already in progress", log_output, "Expected put already in progress message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Fail to open local file
        ret = MAVFTPReturn("Put", FtpError.FailToOpenLocalFile)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Put failed, failed to open local file", log_output, "Expected fail to open local file message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Remote Reply Timeout
        ret = MAVFTPReturn("Put", FtpError.RemoteReplyTimeout)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Put failed, remote reply timeout", log_output, "Expected remote reply timeout message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)


if __name__ == '__main__':
    unittest.main()
