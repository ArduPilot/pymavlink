#!/usr/bin/env python3

'''
MAVLink File Transfer Protocol support test - https://mavlink.io/en/services/ftp.html

SPDX-FileCopyrightText: 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
'''

import logging
import os
import struct
import tempfile
import unittest
from io import BytesIO, StringIO

from unittest.mock import patch
from pymavlink import mavutil
from pymavlink.mavftp import (
    FTP_OP,
    MAVFTP,
    FtpError,
    MAVFTPReturn,
    OP_Ack,
    OP_BurstReadFile,
    OP_CreateFile,
    OP_ListDirectory,
    OP_Nack,
    OP_OpenFileRO,
    OP_ReadFile,
    OP_RemoveFile,
    OP_ResetSessions,
    OP_TerminateSession,
    OP_WriteFile,
)

# pylint: disable=protected-access


class FakeFTPMessage:  # pylint: disable=too-few-public-methods
    """Minimal FILE_TRANSFER_PROTOCOL message for reply-loop tests."""

    def __init__(self, op):
        self.payload = op.pack()
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


class FakeMaster:  # pylint: disable=too-few-public-methods
    """Serve a predetermined sequence of FTP replies."""

    source_system = 1
    source_component = 1

    def __init__(self, replies):
        self.mav = FakeMAV()
        self.replies = replies
        self.empty_polls = 0

    def recv_match(self, **_kwargs):
        if self.empty_polls:
            self.empty_polls -= 1
            return None
        if self.replies:
            return self.replies.pop(0)
        return None


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
    def make_ftp(replies):
        master = FakeMaster(
            [ftp_reply(1, OP_Ack, OP_ResetSessions)] + replies
        )
        ftp = MAVFTP(master, target_system=1, target_component=1)
        ftp.ftp_settings.idle_detection_time = 0.02
        ftp.ftp_settings.read_retry_time = 0.01
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

    @staticmethod
    def sent_request_sequences(master, opcode):
        """Return FTP request sequence numbers sent for an opcode."""
        return [
            struct.unpack_from("<H", sent[-1])[0]
            for sent in master.mav.sent
            if sent[-1][3] == opcode
        ]

    def test_open_file_ack_uses_allocated_session(self):
        """A non-echoing OpenFileRO server session is used for BurstReadFile."""
        ftp, master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])

        ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[1, 0, 0, 0], session=42)
        )

        self.assertEqual(ftp.session, 42)
        self.assertEqual(master.mav.sent[-1][-1][2], 42)

    def test_create_file_ack_uses_allocated_session(self):
        """A non-echoing CreateFile server session is used for WriteFile."""
        ftp, master = self.make_ftp([])
        ftp.cmd_put(["local", "remote"], fh=BytesIO(b"x"))

        ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(2, OP_Ack, OP_CreateFile, session=37)
        )

        self.assertEqual(ftp.session, 37)
        self.assertEqual(master.mav.sent[-1][-1][2], 37)

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
                ftp_reply(4, OP_Ack, OP_BurstReadFile, payload=b"y", offset=80, burst_complete=1),
                ftp_reply(5, OP_Ack, OP_TerminateSession),
            ]
        )

        ftp.cmd_get(
            ["remote", "-"],
            callback=lambda _fh: MAVFTPReturn("Get", FtpError.Success),
        )
        result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.duplicates, 0)

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
        ftp.seq = 255
        master.replies.append(ftp_reply(256, OP_Ack, OP_RemoveFile))

        result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.Success)

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
