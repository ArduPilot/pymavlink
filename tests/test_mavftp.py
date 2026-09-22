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
import sys
import tempfile
import time
import unittest
from argparse import Namespace
from io import BytesIO, StringIO

from unittest.mock import MagicMock, patch
from pymavlink import mavftp as mavftp_module
from pymavlink import mavutil
from pymavlink.examples import mavftp_example
from pymavlink.mavftp import (
    BURST_REPLY_SEQUENCE_WINDOW,
    DirectoryEntry,
    FTP_OP,
    FTP_SEQ_MODULUS,
    FTP_SESSION_MODULUS,
    MAX_INITIAL_RETRIES,
    MAX_READ_GAPS,
    MAX_READ_RETRIES,
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
from pymavlink.tools.test_mavftp_hardware import _check_crc_result

# pylint: disable=protected-access,too-many-lines,duplicate-code


class FakeFTPMessage:
    """Minimal FILE_TRANSFER_PROTOCOL message for reply-loop tests."""

    def __init__(self, op):
        self.payload = op.pack()
        self.target_system = 1
        self.target_component = 1

    @staticmethod
    def get_type():
        return "FILE_TRANSFER_PROTOCOL"

    @staticmethod
    def get_srcSystem():  # pylint: disable=invalid-name
        return 1

    @staticmethod
    def get_srcComponent():  # pylint: disable=invalid-name
        return 1


class RawFTPMessage:
    """Minimal raw FTP message used to exercise malformed-packet handling."""

    def __init__(self, payload):
        self.payload = payload
        self.target_system = 1
        self.target_component = 1

    @staticmethod
    def get_type():
        return "FILE_TRANSFER_PROTOCOL"

    @staticmethod
    def get_srcSystem():  # pylint: disable=invalid-name
        return 1

    @staticmethod
    def get_srcComponent():  # pylint: disable=invalid-name
        return 1


class OSWithoutFchmod:  # pylint: disable=too-few-public-methods
    """Proxy for os that simulates Python versions without os.fchmod on Windows."""

    def __getattr__(self, name):
        if name == "fchmod":
            raise AttributeError(name)
        return getattr(os, name)


class FakeMAV:  # pylint: disable=too-few-public-methods
    """Record FTP sends without requiring a MAVLink transport."""

    def __init__(self):
        self.sent = []

    def file_transfer_protocol_send(self, *args):
        self.sent.append(args)


class BinaryStdout:
    """Capture the binary stdout stream used by ``get ... -``."""

    def __init__(self):
        self.buffer = BytesIO()

    def write(self, text):
        return len(text)

    def flush(self):
        pass


class BatchLink:  # pylint: disable=too-few-public-methods
    """Collect raw writes from the upload batching path."""

    def __init__(self):
        self.port = type("Port", (), {"type": socket.SOCK_DGRAM})()
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)


class mavtcp:  # pylint: disable=invalid-name,too-few-public-methods
    """Raw TCP-shaped link whose socket is temporarily back-pressured."""

    def __init__(self):
        self.port = MagicMock(type=socket.SOCK_STREAM)
        self.port.sendall.side_effect = BlockingIOError()
        self.handle_disconnect = MagicMock()

    def write(self, _data):
        raise AssertionError("raw stream path must use sendall")


class BatchMAV:
    """Minimal MAVLink encoder that writes through a replaceable link."""

    def __init__(self, link):
        self.file = link
        self.observed_files = []

    def file_transfer_protocol_send(self, _network, _target_system, _target_component, payload):
        self.file.write(b"F" + bytes(payload))

    def file_transfer_protocol_encode(self, _network, _target_system, _target_component, payload):
        self.observed_files.append(self.file)
        return b"F" + bytes(payload)


class SignedBatchMAV:
    """MAVLink sender that models a callback interleaving another send."""

    def __init__(self, link):
        self.file = link
        self.signing = Namespace(sign_outgoing=True)
        self.encoded = 0

    def file_transfer_protocol_send(
        self, _network, _target_system, _target_component, payload
    ):
        self.file.write(b"F" + bytes(payload))
        # MAVLink invokes send callbacks only after the packet has reached
        # its link. A callback may send a heartbeat on the same connection.
        self.file.write(b"H")

    def file_transfer_protocol_encode(self, *_args):
        self.encoded += 1
        raise AssertionError("signed packets must not be encoded ahead of their write")


class FakeMaster:  # pylint: disable=too-few-public-methods
    """Serve a predetermined sequence of FTP replies."""

    source_system = 1
    source_component = 1

    def __init__(self, replies, validate_replies=True):
        self.mav = FakeMAV()
        self.replies = replies
        self.validate_replies = validate_replies
        self.empty_polls = 0
        self.recv_calls = []

    @staticmethod
    def _decode_payload(payload):
        seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = (
            struct.unpack_from("<HBBBBBBI", payload)
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

    def _reply_matches_sent_request(self, reply):
        if not self.validate_replies or not isinstance(reply, FakeFTPMessage):
            return True
        if not self.mav.sent:
            return False
        response = self._decode_payload(reply.payload)
        return any(
            response.req_opcode == self._decode_payload(sent[-1]).opcode
            for sent in self.mav.sent
        )

    def recv_match(self, **_kwargs):
        self.recv_calls.append(_kwargs)
        if self.empty_polls:
            self.empty_polls -= 1
            return None
        if self.replies:
            reply = self.replies[0]
            if not self._reply_matches_sent_request(reply):
                return None
            return self.replies.pop(0)
        return None


class SilentTimestampListingMaster(FakeMaster):  # pylint: disable=too-few-public-methods
    """Drop timestamp listings, then serve the baseline listing opcode."""

    def __init__(self):
        super().__init__([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        self.sent_index = 0

    def recv_match(self, **kwargs):
        while self.sent_index < len(self.mav.sent):
            request = self._decode_payload(self.mav.sent[self.sent_index][-1])
            self.sent_index += 1
            reply_seq = (request.seq + 1) % FTP_SEQ_MODULUS
            if request.opcode == OP_ResetSessions:
                # The reset reply is already queued by the constructor.
                continue
            if request.opcode == OP_ListDirectoryWithTime:
                # Model a server that silently ignores this optional opcode.
                continue
            if request.opcode == OP_ListDirectory:
                if request.offset == 0:
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Ack,
                            OP_ListDirectory,
                            payload=b"Fone.bin\t1\x00Dlogs\x00",
                        )
                    )
                else:
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Nack,
                            OP_ListDirectory,
                            payload=[FtpError.EndOfFile],
                        )
                    )
                break
            if request.opcode == OP_TerminateSession:
                self.replies.append(ftp_reply(reply_seq, OP_Ack, OP_TerminateSession))
                break
        return super().recv_match(**kwargs)


class LostTimestampPageMaster(FakeMaster):  # pylint: disable=too-few-public-methods
    """Drop one timestamped follow-up page, then answer its retry."""

    def __init__(self):
        super().__init__([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        self.sent_index = 0
        self.dropped_page = False

    def recv_match(self, **kwargs):
        while self.sent_index < len(self.mav.sent):
            request = self._decode_payload(self.mav.sent[self.sent_index][-1])
            self.sent_index += 1
            reply_seq = (request.seq + 1) % FTP_SEQ_MODULUS
            if request.opcode == OP_ResetSessions:
                continue
            if request.opcode != OP_ListDirectoryWithTime:
                continue
            if request.offset == 0:
                payload = b"Fone.bin\t1\t1700000000\x00"
                self.replies.append(
                    ftp_reply(reply_seq, OP_Ack, OP_ListDirectoryWithTime, payload=payload)
                )
            elif request.offset == 1 and not self.dropped_page:
                self.dropped_page = True
                continue
            elif request.offset == 1:
                payload = b"Dlogs\t0\t1700000001\x00"
                self.replies.append(
                    ftp_reply(reply_seq, OP_Ack, OP_ListDirectoryWithTime, payload=payload)
                )
            else:
                self.replies.append(
                    ftp_reply(
                        reply_seq,
                        OP_Nack,
                        OP_ListDirectoryWithTime,
                        payload=[FtpError.EndOfFile],
                    )
                )
            break
        return super().recv_match(**kwargs)


class FlakyTimestampListingMaster(FakeMaster):  # pylint: disable=too-few-public-methods
    """Support timestamps while dropping the first page of the second list."""

    def __init__(self):
        super().__init__([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        self.sent_index = 0
        self.list_count = 0
        self.dropped_page = False

    def recv_match(self, **kwargs):
        while self.sent_index < len(self.mav.sent):
            request = self._decode_payload(self.mav.sent[self.sent_index][-1])
            self.sent_index += 1
            reply_seq = (request.seq + 1) % FTP_SEQ_MODULUS
            if request.opcode == OP_ResetSessions:
                continue
            if request.opcode == OP_ListDirectoryWithTime:
                if request.offset == 0 and self.list_count == 0:
                    self.list_count = 1
                    payload = b"Fone.bin\t1\t1700000000\x00"
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Ack,
                            OP_ListDirectoryWithTime,
                            payload=payload,
                        )
                    )
                elif request.offset == 0 and self.list_count == 1:
                    if not self.dropped_page:
                        self.dropped_page = True
                        continue
                    self.list_count = 2
                    payload = b"Fone.bin\t1\t1700000002\x00"
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Ack,
                            OP_ListDirectoryWithTime,
                            payload=payload,
                        )
                    )
                else:
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Nack,
                            OP_ListDirectoryWithTime,
                            payload=[FtpError.EndOfFile],
                        )
                    )
                break
            if request.opcode == OP_ListDirectory:
                if request.offset == 0:
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Ack,
                            OP_ListDirectory,
                            payload=b"Fone.bin\t1\x00",
                        )
                    )
                else:
                    self.replies.append(
                        ftp_reply(
                            reply_seq,
                            OP_Nack,
                            OP_ListDirectory,
                            payload=[FtpError.EndOfFile],
                        )
                    )
                break
        return super().recv_match(**kwargs)


class NoSessionsTimestampListingMaster(FakeMaster):  # pylint: disable=too-few-public-methods
    """Return NoSessionsAvailable once after timestamp support is known."""

    def __init__(self):
        super().__init__([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        self.sent_index = 0
        self.list_count = 0
        self.no_sessions_sent = False

    def recv_match(self, **kwargs):
        while self.sent_index < len(self.mav.sent):
            request = self._decode_payload(self.mav.sent[self.sent_index][-1])
            self.sent_index += 1
            reply_seq = (request.seq + 1) % FTP_SEQ_MODULUS
            if request.opcode == OP_ResetSessions:
                continue
            if request.opcode != OP_ListDirectoryWithTime:
                continue
            if request.offset == 0 and self.list_count == 0:
                self.list_count = 1
                payload = b"Fone.bin\t1\t1700000000\x00"
                self.replies.append(
                    ftp_reply(reply_seq, OP_Ack, OP_ListDirectoryWithTime, payload=payload)
                )
            elif request.offset == 0 and not self.no_sessions_sent:
                self.no_sessions_sent = True
                self.replies.append(
                    ftp_reply(
                        reply_seq,
                        OP_Nack,
                        OP_ListDirectoryWithTime,
                        payload=[FtpError.NoSessionsAvailable],
                    )
                )
            elif request.offset == 0:
                self.list_count = 2
                payload = b"Fone.bin\t1\t1700000002\x00"
                self.replies.append(
                    ftp_reply(reply_seq, OP_Ack, OP_ListDirectoryWithTime, payload=payload)
                )
            else:
                self.replies.append(
                    ftp_reply(
                        reply_seq,
                        OP_Nack,
                        OP_ListDirectoryWithTime,
                        payload=[FtpError.EndOfFile],
                    )
                )
            break
        return super().recv_match(**kwargs)


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
    def make_ftp(replies, list_time=0, validate_replies=True):
        master = FakeMaster(
            [ftp_reply(1, OP_Ack, OP_ResetSessions)] + replies,
            validate_replies=validate_replies,
        )
        ftp = MAVFTP(master, target_system=1, target_component=1)
        if list_time is not None:
            ftp.ftp_settings.list_time = list_time
        ftp.ftp_settings.read_retry_time = 0.01
        ftp.ftp_settings.idle_detection_time = 0.02
        ftp.ftp_settings.retry_time = 0.2
        return ftp, master

    def test_fake_master_withholds_reply_until_matching_request_is_sent(self):
        """The unit transport must not deliver an unrelated canned reply."""
        master = FakeMaster([ftp_reply(1, OP_Ack, OP_RemoveFile)])

        wrong_request = FTP_OP(0, 0, OP_Rename, 0, 0, 0, 0, None)
        master.mav.file_transfer_protocol_send(0, 1, 1, wrong_request.pack())
        self.assertIsNone(master.recv_match(type="FILE_TRANSFER_PROTOCOL"))
        self.assertEqual(len(master.replies), 1)

        request = FTP_OP(0, 0, OP_RemoveFile, 0, 0, 0, 0, None)
        master.mav.file_transfer_protocol_send(0, 1, 1, request.pack())

        self.assertIsNotNone(master.recv_match(type="FILE_TRANSFER_PROTOCOL"))
        self.assertEqual(master.replies, [])

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

    def test_delayed_terminate_ignores_reply_for_wrong_target(self):
        """RX-lagged wrong-target termination replies do not count as accepted."""
        class WrongTargetMaster:  # pylint: disable=too-few-public-methods
            """Serve one wrong-target reply after initialization."""

            source_system = 1
            source_component = 1

            def __init__(self, reply):
                self.mav = FakeMAV()
                self.reply = reply
                self.calls = 0

            def recv_match(self, **_kwargs):
                self.calls += 1
                if self.calls == 1:
                    return self.reply
                raise AssertionError("wrong-target delayed reply was treated as accepted")

        master = FakeMaster([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        ftp = MAVFTP(master, target_system=1, target_component=1)
        reply = ftp_reply(ftp.seq + 1, OP_Ack, OP_TerminateSession)
        reply.target_system = 99
        master = WrongTargetMaster(reply)
        ftp.master = master
        ftp.pending_terminate_seq = ftp.seq
        ftp.ftp_settings.pkt_lag_rx = 1
        original_idle_task = ftp.idle_task

        def flush_then_expire():
            if ftp.rx_delay_queue:
                _deadline, sequence, message = ftp.rx_delay_queue[0]
                ftp.rx_delay_queue[0] = (time.monotonic() - 1, sequence, message)
            original_idle_task()
            return True

        ftp.idle_task = flush_then_expire

        with patch.object(
            ftp,
            "_MAVFTP__op_parse",
            side_effect=AssertionError("wrong-target delayed payload was parsed"),
        ):
            result = ftp.process_ftp_reply("TerminateSession", timeout=1)

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(master.calls, 1)

    def test_delayed_wrong_target_malformed_reply_is_not_local_malformed(self):
        """A malformed delayed packet for another target remains unrelated traffic."""
        class WrongTargetMaster:  # pylint: disable=too-few-public-methods
            """Serve one malformed wrong-target reply."""

            source_system = 1
            source_component = 1

            def __init__(self, reply):
                self.mav = FakeMAV()
                self.reply = reply

            def recv_match(self, **_kwargs):
                return self.reply

        master = FakeMaster([ftp_reply(1, OP_Ack, OP_ResetSessions)])
        ftp = MAVFTP(master, target_system=1, target_component=1)
        reply = RawFTPMessage(b"bad")
        reply.target_system = 99
        master = WrongTargetMaster(reply)
        ftp.master = master
        ftp.pending_terminate_seq = ftp.seq
        ftp.ftp_settings.pkt_lag_rx = 1
        original_idle_task = ftp.idle_task

        def flush_then_expire():
            if ftp.rx_delay_queue:
                _deadline, sequence, message = ftp.rx_delay_queue[0]
                ftp.rx_delay_queue[0] = (time.monotonic() - 1, sequence, message)
            original_idle_task()
            return True

        ftp.idle_task = flush_then_expire

        result = ftp.process_ftp_reply("TerminateSession", timeout=1)

        self.assertEqual(result.error_code, FtpError.Fail)

    def test_delayed_wrong_vehicle_malformed_reply_is_not_local_malformed(self):
        """The delayed receive path filters the vehicle before parsing payloads."""
        foreign = RawFTPMessage(b"bad")
        setattr(foreign, "get_srcSystem", lambda: 2)
        local = FakeFTPMessage(
            FTP_OP(
                seq=1,
                session=0,
                opcode=OP_Ack,
                size=0,
                req_opcode=OP_ResetSessions,
                burst_complete=0,
                offset=0,
                payload=None,
            )
        )
        master = FakeMaster([foreign, local], validate_replies=False)
        ftp, _old_master = self.make_ftp([])
        ftp.master = master
        ftp.ftp_settings.pkt_lag_rx = 1.0
        ftp.last_op = FTP_OP(
            seq=0,
            session=ftp.session,
            opcode=OP_ResetSessions,
            size=0,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )
        ftp.pending_reset_seq = 0

        with (
            patch(
                "pymavlink.mavftp.time.monotonic",
                side_effect=[0.0, 0.0, 0.0, 1.0],
            ),
            patch.object(ftp, "_MAVFTP__idle_task", return_value=False),
        ):
            result = ftp.process_ftp_reply("ResetSessions", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)

    def test_delayed_wrong_session_terminate_reply_is_not_accepted(self):
        """A valid packet for another FTP session must not satisfy termination."""
        ftp, _master = self.make_ftp([])
        packet = FakeFTPMessage(
            FTP_OP(
                seq=1,
                session=(ftp.session + 1) % FTP_SESSION_MODULUS,
                opcode=OP_Ack,
                size=0,
                req_opcode=OP_TerminateSession,
                burst_complete=0,
                offset=0,
                payload=None,
            )
        )
        ftp.pending_terminate_seq = 0
        ftp.rx_delay_queue = [(0.0, 1, packet)]

        with (
            patch("pymavlink.mavftp.time.monotonic", return_value=1.0),
            patch.object(ftp.master, "recv_match", return_value=None),
            patch.object(ftp, "idle_task", return_value=True) as idle,
        ):
            result = ftp.process_ftp_reply("TerminateSession", timeout=1)

        idle.assert_called_once()
        self.assertEqual(result.error_code, FtpError.Fail)

    def test_reply_from_another_vehicle_is_rejected(self):
        """A colliding session reply from another vehicle cannot complete a command."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_RemoveFile, 0, 0, 0, 0, bytearray())
        previous_op = ftp.last_op
        reply = ftp_reply(2, OP_Ack, OP_RemoveFile)
        reply.get_srcSystem = lambda: 2  # pylint: disable=invalid-name
        reply.get_srcComponent = lambda: 1  # pylint: disable=invalid-name

        result = ftp._MAVFTP__mavlink_packet(reply)

        self.assertEqual(result.operation_name, "mavlink_packet")
        self.assertEqual(result.error_code, FtpError.InvalidSession)
        self.assertIs(ftp.last_op, previous_op)

    def test_stale_reply_does_not_delay_request_retry(self):
        """A stale reply cannot restart the active request's retry timer."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_ListDirectory, 0, 0, 0, 0, bytearray())
        ftp.last_op_reply = False
        ftp.last_op_time = 0
        send = MagicMock()
        setattr(ftp, "_MAVFTP__send", send)

        with patch("pymavlink.mavftp.time.time", return_value=10.0):
            result = ftp._MAVFTP__mavlink_packet(
                ftp_reply(99, OP_Ack, OP_ListDirectory)
            )
            ftp.idle_task()

        self.assertEqual(result.error_code, FtpError.Fail)
        send.assert_called_once_with(ftp.last_op, retry=True)

    def test_status_reports_a_transfer_during_open_handshake(self):
        """A transfer remains visible before its remote file handle is opened."""
        ftp, _master = self.make_ftp([])
        ftp.transfer_active = True

        with self.assertLogs(level="INFO") as logs:
            result = ftp.cmd_status()

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIn("Transfer in progress", "\n".join(logs.output))

    def test_status_reports_acknowledged_upload_bytes(self):
        """Upload status is based on remote acknowledgements, not file position."""
        ftp, _master = self.make_ftp([])
        ftp.transfer_active = True
        ftp.op_start = time.time() - 1.0
        ftp.filename = "remote.bin"
        ftp.write_list = {1}
        ftp.write_file_size = 200
        ftp.write_acked_bytes = 100

        with self.assertLogs(level="INFO") as logs:
            result = ftp.cmd_status()

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIn("Uploading remote.bin - 100/200 bytes 50.0%", "\n".join(logs.output))

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

    def test_crc_zero_timeout_uses_the_bounded_default(self):
        """An explicit zero timeout must not disable CRC's only deadline."""
        ftp, _master = self.make_ftp([])
        ftp.process_ftp_reply = MagicMock(
            return_value=MAVFTPReturn("CalcFileCRC32", FtpError.RemoteReplyTimeout)
        )

        ftp.cmd_crc(["remote.bin"], timeout=0)

        ftp.process_ftp_reply.assert_called_once_with("CalcFileCRC32", timeout=5.0)

    def test_hardware_crc_report_turns_missing_crc_into_runtime_error(self):
        """A CRC NACK must report a test failure instead of formatting None."""
        result = MAVFTPReturn("CalcFileCRC32", FtpError.FileNotFound)

        with self.assertRaises(RuntimeError):
            _check_crc_result(result, None, 0x12345678)

    def test_malformed_ftp_header_returns_invalid_data_size(self):
        """Given a FILE_TRANSFER_PROTOCOL payload shorter than its header, when parsed, then it fails without raising."""
        ftp, _master = self.make_ftp([])

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            RawFTPMessage(b"\x00" * 3)
        )

        self.assertEqual(result.operation_name, "mavlink_packet")
        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_declared_ftp_payload_larger_than_bytes_returns_invalid_data_size(self):
        """Given an FTP header declaring unavailable payload bytes, when parsed, then it fails without partial decoding."""
        ftp, _master = self.make_ftp([])
        malformed_header = struct.pack("<HBBBBBBI", 2, 0, OP_Ack, 4, OP_RemoveFile, 0, 0, 0)

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            RawFTPMessage(malformed_header)
        )

        self.assertEqual(result.operation_name, "mavlink_packet")
        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_process_rejects_malformed_ftp_header_without_raising(self):
        """Given a malformed reply in the receive loop, when processed, then it returns invalid data size."""
        ftp, _master = self.make_ftp([RawFTPMessage(b"\x00" * 3)])

        result = ftp.process_ftp_reply("RemoveFile", timeout=1)

        self.assertEqual(result.operation_name, "RemoveFile")
        self.assertEqual(result.error_code, FtpError.InvalidDataSize)

    def test_process_ignores_malformed_reply_for_another_gcs(self):
        """A malformed packet for another client cannot abort this client's receive loop."""
        malformed = RawFTPMessage(b"\x00" * 3)
        malformed.target_system = 99
        ftp, _master = self.make_ftp(
            [malformed, ftp_reply(2, OP_Ack, OP_RemoveFile)]
        )

        result = ftp.cmd_ftp(["rm", "remote"])

        self.assertEqual(result.error_code, FtpError.Success)

    def test_process_ignores_malformed_reply_from_another_vehicle(self):
        """A malformed packet from another vehicle cannot abort this client's receive loop."""
        malformed = RawFTPMessage(b"\x00" * 3)
        setattr(malformed, "get_srcSystem", lambda: 2)
        ftp, _master = self.make_ftp(
            [malformed, ftp_reply(2, OP_Ack, OP_RemoveFile)]
        )

        result = ftp.cmd_ftp(["rm", "remote"])

        self.assertEqual(result.error_code, FtpError.Success)

    def test_packet_loss_settings_drop_packets_at_the_expected_boundary(self):
        """TX loss drops sends and RX loss drops replies before they are handled."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.session = 7
        ftp.burst_size = 80
        ftp.ftp_settings.pkt_loss_tx = 100
        sent_before = len(ftp.master.mav.sent)
        ftp._MAVFTP__send(  # pylint: disable=protected-access
            FTP_OP(ftp.seq, ftp.session, OP_RemoveFile, 1, 0, 0, 0, bytearray(b"x"))
        )
        self.assertEqual(len(ftp.master.mav.sent), sent_before)

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

        with patch.object(MAVFTP, "_MAVFTP__create_staging_file", side_effect=OSError("no staging file")):
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
            with open(destination, "wb"):
                pass
            os.chmod(destination, 0o751)
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
            self.assertEqual(os.stat(destination).st_mode & 0o7777, 0o751)
            self.assertEqual(progress, [1.0])
            self.assertIsNone(ftp.get_result)
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

    def test_cmd_get_does_not_retain_a_second_copy_of_a_staged_download(self):
        """A destination-file get must remain streaming and leave get_result empty."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "download.bin")
            ftp, _master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=struct.pack("<I", 4), session=8),
                    ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"data", burst_complete=1, session=8),
                    ftp_reply(4, OP_Ack, OP_TerminateSession, session=8),
                ]
            )

            ftp.cmd_get(["remote.bin", destination])
            result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertIsNone(ftp.get_result)

    def test_staged_download_finalization_does_not_reread_the_file(self):
        """Publishing a destination must not allocate a whole-file result buffer."""
        ftp, _master = self.make_ftp([])
        ftp.fh = MagicMock()
        ftp.fh.tell.return_value = 4
        ftp.fh.fileno.return_value = 7
        ftp.fh.read.side_effect = AssertionError("staged downloads must not be reread")
        ftp.filename = "destination.bin"
        ftp.temp_filename = "staging.bin"
        ftp.op_start = time.time() - 1
        ftp.read_total = 4
        ftp.requested_size = 4
        ftp.remote_size_known = True
        ftp.reached_eof = True

        with (
            patch("pymavlink.mavftp.os.fstat", return_value=Namespace(st_size=4)),
            patch("pymavlink.mavftp.os.replace"),
            patch.object(ftp, "_MAVFTP__fsync_directory"),
            patch.object(ftp, "_MAVFTP__terminate_session"),
        ):
            self.assertTrue(ftp._MAVFTP__check_read_finished())  # pylint: disable=protected-access

        ftp.fh.read.assert_not_called()
        self.assertIsNone(ftp.get_result)

    def assert_download_finalization_failure(self, inject_failure):  # pylint: disable=too-many-locals
        """A terminal local-I/O failure must leave no staging file or remote session."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "download.bin")
            ftp, master = self.make_ftp(
                [
                    ftp_reply(
                        2,
                        OP_Ack,
                        OP_OpenFileRO,
                        payload=struct.pack("<I", 4),
                        session=8,
                    ),
                    ftp_reply(
                        3,
                        OP_Ack,
                        OP_BurstReadFile,
                        payload=b"data",
                        burst_complete=1,
                        session=8,
                    ),
                    ftp_reply(4, OP_Ack, OP_TerminateSession, session=8),
                ]
            )

            self.assertEqual(ftp.cmd_get(["remote.bin", destination]).error_code, FtpError.Success)
            finished_status = MagicMock()
            setattr(ftp, "_MAVFTP__finished_status", finished_status)
            staging_handle = None
            staging_filename = None
            undo_failure_injection = None
            original_recv_match = master.recv_match

            def capture_staging_handle(**kwargs):
                nonlocal staging_handle, staging_filename, undo_failure_injection
                requests = self.sent_requests(master)
                if (
                    staging_handle is None
                    and ftp.fh is not None
                    and requests[-1].opcode == OP_BurstReadFile
                ):
                    staging_handle = ftp.fh
                    staging_filename = ftp.temp_filename
                    undo_failure_injection = inject_failure(ftp, staging_handle)
                return original_recv_match(**kwargs)

            master.recv_match = capture_staging_handle

            try:
                result = ftp.process_ftp_reply("get", timeout=1)
            except OSError:
                result = None
            finally:
                assert staging_handle is not None  # noqa: S101
                assert staging_filename is not None  # noqa: S101
                staging_file_exists = os.path.exists(staging_filename)
                staging_handle_closed = staging_handle.closed
                sent_opcodes = [
                    request.opcode
                    for request in self.sent_requests(master)
                    if request.opcode
                    in {OP_OpenFileRO, OP_BurstReadFile, OP_TerminateSession}
                ]
                if undo_failure_injection is not None:
                    undo_failure_injection()
                if not staging_handle.closed:
                    staging_handle.close()
                if os.path.exists(staging_filename):
                    os.unlink(staging_filename)

            self.assertIsNotNone(result)
            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertTrue(staging_handle_closed)
            self.assertIsNone(ftp.fh)
            self.assertIsNone(ftp.temp_filename)
            self.assertFalse(os.path.exists(destination))
            self.assertFalse(staging_file_exists)
            self.assertEqual([OP_OpenFileRO, OP_BurstReadFile], sent_opcodes[:2])
            self.assertTrue(all(opcode == OP_TerminateSession for opcode in sent_opcodes[2:]))
            finished_status.assert_not_called()

    def test_download_final_flush_failure_cleans_up_staging_and_session(self):
        """A final buffered-write failure must remove staging data and terminate the remote session."""

        def fail_flush(ftp, staging_handle):
            failing_handle = MagicMock(wraps=staging_handle)
            failing_handle.flush.side_effect = OSError("disk full")
            ftp.fh = failing_handle

        self.assert_download_finalization_failure(fail_flush)

    def test_download_final_stat_failure_cleans_up_staging_and_session(self):
        """A staging-file size check failure must remove staging data and terminate the session."""

        def fail_stat(_ftp, _staging_handle):
            fstat_patch = patch(
                "pymavlink.mavftp.os.fstat", side_effect=OSError("disk full")
            )
            fstat_patch.start()
            return fstat_patch.stop

        self.assert_download_finalization_failure(fail_stat)

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
        """A valid parameter archive is decoded even when its stat size is an estimate."""
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
                    ftp_reply(
                        2,
                        OP_Ack,
                        OP_OpenFileRO,
                        payload=struct.pack("<I", len(parameter_data) * 2),
                        session=3,
                    ),
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
            self.assertEqual(progress, [0.5])
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
        oversized = "a" * (mavftp_module.MAX_FTP_NAME + 1)
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

        ftp, master = self.make_ftp([])
        result = ftp.cmd_rename(["a" * 119, "b" * 119])
        self.assertEqual(result.error_code, FtpError.InvalidArguments)
        self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_path_commands_reject_empty_and_nul_paths_without_sending(self):
        """Paths must not be empty or contain protocol field separators."""
        cases = (
            ("rm", [""]),
            ("mkdir", ["\x00"]),
            ("rename", ["old\x00unexpected", "new"]),
            ("rename", ["old", ""]),
        )
        for command, arguments in cases:
            with self.subTest(command=command, arguments=arguments):
                ftp, master = self.make_ftp([])

                result = ftp.cmd_ftp([command, *arguments])

                self.assertEqual(result.error_code, FtpError.InvalidArguments)
                self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_rename_and_getparams_require_exact_argument_counts(self):
        """Malformed API calls fail before starting a remote operation."""
        ftp, master = self.make_ftp([])
        result = ftp.cmd_rename(["old", "new", "ignored"])
        self.assertEqual(result.error_code, FtpError.InvalidArguments)
        self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

        for args in ([], ["values.param", "defaults.param", "ignored"]):
            with self.subTest(getparams_args=args):
                ftp, master = self.make_ftp([])
                result = ftp.cmd_getparams(args)
                self.assertEqual(result.error_code, FtpError.InvalidArguments)
                self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_write_ack_offset_must_match_sequence_request(self):
        """A valid reply sequence cannot acknowledge a different write block."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"ab")
        ftp.filename = "remote"
        ftp.write_list = {0, 1}
        ftp.write_block_size = 1
        ftp.write_total = 2
        ftp._MAVFTP__send_more_writes()  # pylint: disable=protected-access
        expected_offset = ftp.pending_write_replies[2]
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
            FTP_OP(
                2,
                0,
                OP_Ack,
                1,
                OP_WriteFile,
                0,
                expected_offset + ftp.write_block_size,
                bytearray(),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.InvalidDataSize)
        self.assertEqual(terminated, [True])
        self.assertEqual(ftp.write_acks, 0)

    def test_upload_aborts_when_source_shrinks_between_write_blocks(self):
        """A short local read must not be ACKed as a full remote write."""
        ftp, master = self.make_ftp([])
        source = BytesIO(b"ab")
        ftp.ftp_settings.write_size = 1
        ftp.ftp_settings.write_qsize = 1
        self.assertEqual(
            ftp.cmd_put(["local.bin", "remote.bin"], fh=source).error_code,
            FtpError.Success,
        )
        self.assertEqual(
            ftp._MAVFTP__handle_create_file_reply(  # pylint: disable=protected-access
                FTP_OP(2, 0, OP_Ack, 0, OP_CreateFile, 0, 0, bytearray()),
                None,
            ).error_code,
            FtpError.Success,
        )
        write_reply_seq = next(iter(ftp.pending_write_replies))
        source.truncate(1)

        with patch.object(
            ftp,
            "_MAVFTP__terminate_session",
            return_value=MAVFTPReturn("TerminateSession", FtpError.Success),
        ) as terminate:
            result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
                FTP_OP(
                    write_reply_seq,
                    0,
                    OP_Ack,
                    0,
                    OP_WriteFile,
                    0,
                    0,
                    bytearray(),
                ),
                None,
            )

        self.assertEqual(result.error_code, FtpError.Fail)
        terminate.assert_called_once()
        self.assertEqual(
            [request.offset for request in self.sent_requests(master) if request.opcode == OP_WriteFile],
            [0],
        )

    def test_write_nack_with_zero_offset_preserves_server_error(self):
        """NACK offsets are unspecified and must not be validated as ACKs."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"x" * 160)
        ftp.pending_write_replies[2] = 80
        terminated = []
        setattr(ftp, "_MAVFTP__terminate_session", lambda: terminated.append(True))

        result = ftp._MAVFTP__handle_write_reply(
            FTP_OP(2, 0, OP_Nack, 2, OP_WriteFile, 0, 0,
                   bytearray([FtpError.FailErrno, 28])),
            None,
        )

        self.assertEqual(result.error_code, FtpError.FailErrno)
        self.assertEqual(result.system_error, 28)
        self.assertEqual(terminated, [True])

    def test_initial_requests_retry_after_packet_loss_with_a_bounded_budget(self):
        """A lost initial request gets recovery retries before the operation fails."""
        ftp, _master = self.make_ftp([])
        ftp.ftp_settings.retry_time = 0.5
        send = MagicMock()
        terminate = MagicMock()
        setattr(ftp, "_MAVFTP__send", send)
        setattr(ftp, "_MAVFTP__terminate_session", terminate)

        for opcode in (
            OP_CreateFile,
            OP_RemoveFile,
            OP_RemoveDirectory,
            OP_Rename,
            OP_CreateDirectory,
        ):
            with self.subTest(opcode=opcode):
                ftp.last_op = FTP_OP(1, 0, opcode, 1, 0, 0, 0, bytearray(b"x"))
                ftp.last_op_reply = False
                ftp.last_op_time = 0
                ftp.last_send_time = 0
                ftp.request_retries = 0
                send.reset_mock()
                terminate.reset_mock()

                with patch("pymavlink.mavftp.time.time", return_value=1.0):
                    ftp.idle_task()

                send.assert_called_once_with(ftp.last_op, retry=True)
                terminate.assert_not_called()

                ftp.request_retries = 3
                with patch("pymavlink.mavftp.time.time", return_value=31.0):
                    ftp.idle_task()

                terminate.assert_called_once()

    def test_initial_request_retries_use_the_full_budget_before_idle_timeout(self):
        """A silent initial request gets every configured retransmission."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_RemoveFile, 1, 0, 0, 0, bytearray(b"x"))
        ftp.last_op_reply = False
        ftp.last_op_time = 0.0
        ftp.last_send_time = 0.0
        terminate = MagicMock()
        setattr(ftp, "_MAVFTP__terminate_session", terminate)

        for now in (1.01, 3.02):
            with self.subTest(now=now), patch(
                "pymavlink.mavftp.time.time", return_value=now
            ):
                ftp.idle_task()

        with patch("pymavlink.mavftp.time.time", return_value=3.05):
            self.assertFalse(ftp.idle_task())

        with patch("pymavlink.mavftp.time.time", return_value=7.03):
            ftp.idle_task()

        self.assertEqual(ftp.request_retries, MAX_INITIAL_RETRIES)
        terminate.assert_not_called()

        with patch("pymavlink.mavftp.time.time", return_value=11.04):
            ftp.idle_task()

        terminate.assert_called_once()

    def test_initial_retry_backoff_cap_precedes_idle_detection(self):
        """The final initial-request retry deadline must beat idle detection."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_RemoveFile, 1, 0, 0, 0, bytearray(b"x"))
        ftp.last_op_reply = False
        ftp.request_retries = MAX_INITIAL_RETRIES
        ftp.last_op_time = 0.0
        ftp.last_send_time = 0.0
        ftp.ftp_settings.retry_time = 0.2
        terminate = MagicMock()
        setattr(ftp, "_MAVFTP__terminate_session", terminate)

        with patch("pymavlink.mavftp.time.time", return_value=1.01):
            self.assertFalse(ftp.idle_task())

        terminate.assert_called_once()

    def test_silent_initial_request_reports_remote_reply_timeout(self):
        """An unanswered initial request is classified as a remote timeout."""
        ftp, _master = self.make_ftp([])
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)

    def test_initial_retry_ladder_preserves_rtt_sensitive_spacing(self):
        """A slow link keeps the full exponential ladder rather than compressing it."""
        ftp, master = self.make_ftp([])
        ftp.rtt_valid = True
        ftp.rtt = 0.3
        ftp.rttvar = 0.15
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            result = ftp.cmd_rm(["remote"])

        remove_requests = [
            sent for sent in master.mav.sent if sent[-1][3] == OP_RemoveFile
        ]
        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)
        self.assertEqual(len(remove_requests), MAX_INITIAL_RETRIES + 1)

    def test_initial_retry_ladder_waits_for_a_slow_link_before_retransmitting(self):
        """A mutation is not retried before the RTT-derived first interval."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_RemoveFile, 1, 0, 0, 0, bytearray(b"x"))
        ftp.last_op_reply = False
        ftp.last_op_time = 0.0
        ftp.rtt_valid = True
        ftp.rtt = 2.0
        ftp.rttvar = 0.5
        send = MagicMock()
        setattr(ftp, "_MAVFTP__send", send)

        with patch("pymavlink.mavftp.time.time", return_value=3.99):
            ftp.idle_task()
        send.assert_not_called()
        with patch("pymavlink.mavftp.time.time", return_value=4.01):
            ftp.idle_task()
        send.assert_called_once_with(ftp.last_op, retry=True)

    def test_crc_rejects_negative_and_nonfinite_timeouts_without_sending(self):
        """CRC's bounded zero default does not turn invalid values into waits."""
        for timeout in (-1, float("-inf")):
            with self.subTest(timeout=timeout):
                ftp, master = self.make_ftp([])

                result = ftp.cmd_crc(["remote"], timeout=timeout)

                self.assertEqual(result.error_code, FtpError.InvalidArguments)
                self.assertEqual(len(master.mav.sent), 1)  # ResetSessions only.

    def test_mutating_commands_reject_invalid_timeouts_before_sending(self):
        """Invalid public timeouts cannot issue a remote filesystem mutation."""
        commands = (
            ("cmd_rm", ["remote"], OP_RemoveFile),
            ("cmd_rmdir", ["remote"], OP_RemoveDirectory),
            ("cmd_rename", ["old", "new"], OP_Rename),
            ("cmd_mkdir", ["remote"], OP_CreateDirectory),
        )
        for timeout in (-1, float("nan"), float("inf")):
            for method_name, args, opcode in commands:
                with self.subTest(timeout=timeout, command=method_name):
                    ftp, master = self.make_ftp([])

                    result = getattr(ftp, method_name)(args, timeout=timeout)

                    self.assertEqual(result.error_code, FtpError.InvalidArguments)
                    self.assertNotIn(
                        opcode, [request.opcode for request in self.sent_requests(master)]
                    )

    def test_staging_file_preserves_existing_private_destination_mode(self):
        """A private destination remains private while its replacement is staged."""
        previous_umask = os.umask(0o022)
        try:
            with tempfile.TemporaryDirectory() as tempdir:
                destination = os.path.join(tempdir, "private.bin")
                with open(destination, "wb"):
                    pass
                os.chmod(destination, 0o600)
                ftp, _master = self.make_ftp([])
                ftp.cmd_get(["remote.bin", destination])

                result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                    ftp_reply(
                        2,
                        OP_Ack,
                        OP_OpenFileRO,
                        payload=struct.pack("<I", 4),
                        session=8,
                    )
                )

                self.assertEqual(result.error_code, FtpError.Success)
                self.assertIsNotNone(ftp.temp_filename)
                self.assertEqual(os.stat(ftp.temp_filename).st_mode & 0o777, 0o600)
                ftp._MAVFTP__release_staging()  # pylint: disable=protected-access
        finally:
            os.umask(previous_umask)

    @unittest.skipIf(os.name == "nt", "Windows does not apply POSIX umask modes")
    def test_download_new_destination_honors_umask_mode(self):
        """A newly created download destination receives the caller's umask-derived mode."""
        previous_umask = os.umask(0o027)
        try:
            with tempfile.TemporaryDirectory() as tempdir:
                destination = os.path.join(tempdir, "new-download.bin")
                ftp, _master = self.make_ftp(
                    [
                        ftp_reply(
                            2,
                            OP_Ack,
                            OP_OpenFileRO,
                            payload=struct.pack("<I", 4),
                            session=8,
                        ),
                        ftp_reply(
                            3,
                            OP_Ack,
                            OP_BurstReadFile,
                            payload=b"data",
                            burst_complete=1,
                            session=8,
                        ),
                        ftp_reply(4, OP_Ack, OP_TerminateSession, session=8),
                    ]
                )

                self.assertEqual(ftp.cmd_get(["remote.bin", destination]).error_code, FtpError.Success)
                result = ftp.process_ftp_reply("get", timeout=1)

                self.assertEqual(result.error_code, FtpError.Success)
                self.assertEqual(os.stat(destination).st_mode & 0o777, 0o640)
        finally:
            os.umask(previous_umask)

    def test_download_staging_receives_existing_destination_mode(self):
        """The staging file setup receives the mode of a destination being replaced."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "existing.bin")
            with open(destination, "wb"):
                pass
            os.chmod(destination, 0o600)
            expected_mode = os.stat(destination).st_mode & 0o7777
            ftp, _master = self.make_ftp([])
            ftp.cmd_get(["remote.bin", destination])

            try:
                with patch.object(
                    ftp,
                    "_MAVFTP__set_staging_mode",
                    wraps=ftp._MAVFTP__set_staging_mode,
                ) as set_staging_mode:
                    result = ftp._MAVFTP__mavlink_packet(
                        ftp_reply(
                            2,
                            OP_Ack,
                            OP_OpenFileRO,
                            payload=struct.pack("<I", 4),
                            session=8,
                        )
                    )

                self.assertEqual(result.error_code, FtpError.Success)
                self.assertEqual(set_staging_mode.call_args.args[2], expected_mode)
            finally:
                ftp._MAVFTP__release_staging()

    def test_download_existing_destination_without_fchmod_uses_chmod_fallback(self):
        """An existing destination remains downloadable on platforms without os.fchmod."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "existing.bin")
            with open(destination, "wb"):
                pass
            os.chmod(destination, 0o640)
            ftp, _master = self.make_ftp([])
            ftp.cmd_get(["remote.bin", destination])

            with patch.object(mavftp_module, "os", OSWithoutFchmod()):
                result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
                    ftp_reply(
                        2,
                        OP_Ack,
                        OP_OpenFileRO,
                        payload=struct.pack("<I", 4),
                        session=8,
                    )
                )

            self.assertEqual(result.error_code, FtpError.Success)
            self.assertIsNotNone(ftp.temp_filename)
            self.assertEqual(os.stat(ftp.temp_filename).st_mode & 0o777, 0o640)
            ftp._MAVFTP__release_staging()  # pylint: disable=protected-access

    def test_crccmp_has_an_overall_deadline(self):
        """A multi-file CRC comparison stops once its overall deadline expires."""
        with tempfile.TemporaryDirectory() as tempdir:
            first = os.path.join(tempdir, "a.bin")
            second = os.path.join(tempdir, "b.bin")
            for filename in (first, second):
                with open(filename, "wb") as output:
                    output.write(b"data")
            ftp, _master = self.make_ftp([])
            ftp.ftp_settings.crccmp_timeout = 1.0
            ftp.local_file_crc = MagicMock(return_value=1)

            def delayed_crc(_args, timeout=None):
                del timeout
                ftp.last_crc = 1
                return MAVFTPReturn("CalcFileCRC32", FtpError.Success)

            ftp.cmd_crc = MagicMock(side_effect=delayed_crc)
            clock_values = iter((0.0, 2.0))
            with patch(
                "pymavlink.mavftp.time.time",
                side_effect=lambda: next(clock_values, 2.0),
            ):
                result = ftp.cmd_crccmp([os.path.join(tempdir, "*.bin"), "/remote"])

            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertEqual(ftp.crccmp_results, ["ERROR", "SKIPPED"])
            ftp.cmd_crc.assert_not_called()

    def test_crccmp_shares_its_deadline_between_remaining_files(self):
        """One silent CRC cannot consume the whole comparison budget."""
        with tempfile.TemporaryDirectory() as tempdir:
            for filename in ("a.bin", "b.bin", "c.bin"):
                with open(os.path.join(tempdir, filename), "wb") as output:
                    output.write(b"data")
            ftp, _master = self.make_ftp([])
            ftp.ftp_settings.crccmp_timeout = 30.0
            ftp.local_file_crc = MagicMock(return_value=1)
            now = [0.0]
            attempted = []

            def silent_crc(args, timeout=None):
                attempted.append(args[0])
                now[0] += timeout
                return MAVFTPReturn("CalcFileCRC32", FtpError.RemoteReplyTimeout)

            ftp.cmd_crc = MagicMock(side_effect=silent_crc)
            with patch("pymavlink.mavftp.time.time", side_effect=lambda: now[0]):
                result = ftp.cmd_crccmp([os.path.join(tempdir, "*.bin"), "/remote"])

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(attempted, ["/remote/a.bin", "/remote/b.bin", "/remote/c.bin"])
        self.assertEqual(
            [call.kwargs["timeout"] for call in ftp.cmd_crc.call_args_list],
            [20.0, 5.0, 5.0],
        )
        self.assertEqual(ftp.crccmp_results, ["ERROR", "ERROR", "ERROR"])

    def test_crccmp_keeps_slow_responsive_crc_results_before_deadline(self):
        """A responsive CRC may use batch time beyond its five-second default."""
        local_files = [f"local/{index}.bin" for index in range(10)]
        ftp, _master = self.make_ftp([])
        ftp.ftp_settings.crccmp_timeout = 120.0
        ftp.local_file_crc = MagicMock(return_value=1)
        now = [0.0]

        def slow_crc(_args, timeout=None):
            if timeout < 15.0:
                now[0] += timeout
                return MAVFTPReturn("CalcFileCRC32", FtpError.RemoteReplyTimeout)
            now[0] += 15.0
            ftp.last_crc = 1
            return MAVFTPReturn("CalcFileCRC32", FtpError.Success)

        ftp.cmd_crc = MagicMock(side_effect=slow_crc)
        with (
            patch("pymavlink.mavftp.glob.glob", return_value=local_files),
            patch("pymavlink.mavftp.os.path.isfile", return_value=True),
            patch("pymavlink.mavftp.time.time", side_effect=lambda: now[0]),
        ):
            result = ftp.cmd_crccmp(["local/*.bin", "/remote"])

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(ftp.crccmp_results[:8], ["MATCH"] * 8)
        self.assertEqual(ftp.crccmp_results[8:], ["ERROR", "SKIPPED"])

    def test_crccmp_records_files_skipped_after_deadline(self):
        """A deadline-expired batch includes an explicit result for every file."""
        with tempfile.TemporaryDirectory() as tempdir:
            for filename in ("a.bin", "b.bin", "c.bin"):
                with open(os.path.join(tempdir, filename), "wb") as output:
                    output.write(b"data")
            ftp, _master = self.make_ftp([])
            ftp.ftp_settings.crccmp_timeout = 1.0
            ftp.local_file_crc = MagicMock(return_value=1)
            ftp.cmd_crc = MagicMock(
                return_value=MAVFTPReturn("CalcFileCRC32", FtpError.Success)
            )
            ftp.last_crc = 1

            clock_values = iter((0.0, 0.0, 2.0))
            with patch(
                "pymavlink.mavftp.time.time",
                side_effect=lambda: next(clock_values, 2.0),
            ):
                result = ftp.cmd_crccmp([os.path.join(tempdir, "*.bin"), "/remote"])

            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertEqual(ftp.crccmp_results, ["MATCH", "ERROR", "SKIPPED"])

    def test_batch_tcp_backpressure_does_not_disconnect_link(self):
        """A transient non-blocking TCP send must not be treated as a link loss."""
        ftp, _master = self.make_ftp([])
        link = mavtcp()

        ftp._MAVFTP__write_link_data(link, b"payload", True)  # pylint: disable=protected-access

        link.handle_disconnect.assert_not_called()
    def test_staging_creation_does_not_read_process_umask(self):
        """Staging allocation must not mutate the process-global umask."""
        ftp, _master = self.make_ftp([])
        with tempfile.TemporaryDirectory() as tempdir:
            with patch("pymavlink.mavftp.os.umask", wraps=os.umask) as umask:
                fd, filename = ftp._MAVFTP__create_staging_file(tempdir)  # pylint: disable=protected-access
            os.close(fd)
            os.unlink(filename)
            umask.assert_not_called()

    def test_download_fsyncs_staging_file_and_destination_directory(self):
        """Successful publication flushes file data and the rename to storage."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "download.bin")
            ftp, _master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=struct.pack("<I", 4), session=8),
                    ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"data", burst_complete=1, session=8),
                    ftp_reply(4, OP_Ack, OP_TerminateSession, session=8),
                ]
            )
            ftp.cmd_get(["remote.bin", destination])
            with patch("pymavlink.mavftp.os.fsync") as fsync:
                result = ftp.process_ftp_reply("get", timeout=1)

            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(fsync.call_count, 2)

    def test_download_directory_fsync_failure_does_not_fail_published_file(self):
        """A directory fsync failure is only a durability warning after publication."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = os.path.join(tempdir, "download.bin")
            ftp, _master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=struct.pack("<I", 4), session=8),
                    ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"data", burst_complete=1, session=8),
                    ftp_reply(4, OP_Ack, OP_TerminateSession, session=8),
                ]
            )
            ftp.cmd_get(["remote.bin", destination])
            with patch(
                "pymavlink.mavftp.os.fsync",
                side_effect=[None, OSError("directory sync unavailable")],
            ):
                result = ftp.process_ftp_reply("get", timeout=1)

            self.assertEqual(result.error_code, FtpError.Success)
            with open(destination, "rb") as downloaded:
                self.assertEqual(downloaded.read(), b"data")
            self.assertIsNone(ftp.temp_filename)

    def test_timestamp_probe_remains_pending_until_configured_timeout(self):
        """A silent timestamp server cannot end the list before its probe timeout."""
        ftp, _master = self.make_ftp([])
        ftp.list_with_time = True
        ftp.list_time_supported = None
        ftp.dir_offset = 0
        ftp.last_op = FTP_OP(1, 0, OP_ListDirectoryWithTime, 1, 0, 0, 0, bytearray(b"/"))
        ftp.last_op_time = 0
        ftp.last_send_time = 0
        ftp.ftp_settings.list_time_timeout = 10.0

        with patch("pymavlink.mavftp.time.time", return_value=3.8):
            self.assertFalse(ftp.idle_task())

    def test_retried_generic_open_failure_is_verified_by_burst_read(self):
        """A retried generic open failure must prove the session before success."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])
        ftp.request_retries = 1
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        open_result = ftp._MAVFTP__handle_open_ro_reply(  # pylint: disable=protected-access
            FTP_OP(
                2,
                0,
                OP_Nack,
                1,
                OP_OpenFileRO,
                0,
                0,
                bytearray([FtpError.Fail]),
            ),
            None,
        )

        self.assertEqual(open_result.error_code, FtpError.Success)
        burst_result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(
                3,
                0,
                OP_Nack,
                1,
                OP_BurstReadFile,
                0,
                0,
                bytearray([FtpError.InvalidSession]),
            ),
            None,
        )

        self.assertEqual(burst_result.error_code, FtpError.InvalidSession)
        self.assertEqual(terminated, [True])

    def test_retried_generic_empty_create_failure_is_not_recovered(self):
        """An empty upload has no write exchange to verify a generic failure."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_put(["local", "remote"], fh=BytesIO())
        ftp.request_retries = 1
        terminated = []
        setattr(
            ftp,
            "_MAVFTP__terminate_session",
            lambda: terminated.append(True),
        )

        result = ftp._MAVFTP__handle_create_file_reply(  # pylint: disable=protected-access
            FTP_OP(
                2,
                0,
                OP_Nack,
                1,
                OP_CreateFile,
                0,
                0,
                bytearray([FtpError.Fail]),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertEqual(terminated, [True])

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

    def test_read_wrong_target_traffic_does_not_extend_inactivity_deadline(self):
        """Replies for another GCS cannot keep a synchronous read alive."""
        class WrongTargetFloodMaster:  # pylint: disable=too-few-public-methods
            """Return wrong-target replies while advancing the test clock."""

            source_system = 1
            source_component = 1

            def __init__(self, clock):
                self.mav = FakeMAV()
                self.clock = clock
                self.recv_calls = 0
                self.reply = ftp_reply(2, OP_Ack, OP_OpenFileRO)
                self.reply.target_system = 99

            def recv_match(self, **_kwargs):
                self.recv_calls += 1
                if self.recv_calls > 20:
                    raise AssertionError("wrong-target traffic kept read alive")
                self.clock[0] += 0.5
                return self.reply

        clock = [0.0]
        master = WrongTargetFloodMaster(clock)
        with patch.object(mavftp_module.time, "time", side_effect=lambda: clock[0]), patch.object(
            mavftp_module.time, "sleep"
        ):
            ftp = MAVFTP(master, target_system=1, target_component=1)
            ftp.idle_task = lambda: False
            setattr(ftp, "_MAVFTP__terminate_session", lambda: None)
            self.assertIsNone(ftp.read("remote", 1))

        self.assertLessEqual(master.recv_calls, 20)

    def test_read_renews_deadline_for_delayed_accepted_replies(self):
        """RX-lagged transfer progress renews the synchronous read deadline."""
        class EmptyMaster:  # pylint: disable=too-few-public-methods
            """Return empty polls while delayed RX traffic makes progress."""

            source_system = 1
            source_component = 1

            def __init__(self, clock):
                self.mav = FakeMAV()
                self.clock = clock

            def recv_match(self, **_kwargs):
                self.clock[0] += 6 if self.clock[0] == 0 else 1

        clock = [0]
        master = EmptyMaster(clock)
        with patch.object(mavftp_module.time, "time", side_effect=lambda: clock[0]), patch.object(
            mavftp_module.time, "sleep"
        ):
            bootstrap_master = FakeMaster([ftp_reply(1, OP_Ack, OP_ResetSessions)])
            ftp = MAVFTP(bootstrap_master, target_system=1, target_component=1)
            ftp.master = master
            accepted_before_read = ftp.accepted_reply_generation
            ftp.ftp_settings.pkt_lag_rx = 1
            flush_count = 0
            original_idle_task = ftp.idle_task

            def queue_delayed_progress():
                nonlocal flush_count
                if flush_count == 0:
                    reply = ftp_reply(
                        ftp.last_op.seq + 1,
                        OP_Ack,
                        OP_OpenFileRO,
                        payload=struct.pack("<I", 1),
                        session=7,
                    )
                elif flush_count == 1:
                    reply = ftp_reply(
                        ftp.last_op.seq + 1,
                        OP_Ack,
                        OP_BurstReadFile,
                        payload=b"x",
                        burst_complete=1,
                        session=7,
                    )
                else:
                    return original_idle_task()
                flush_count += 1
                ftp._MAVFTP__receive_packet(reply)
                _deadline, sequence, message = ftp.rx_delay_queue[0]
                ftp.rx_delay_queue[0] = (time.monotonic() - 1, sequence, message)
                return original_idle_task()

            ftp.idle_task = queue_delayed_progress
            setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

            self.assertEqual(ftp.read("remote", 1), b"x")
            self.assertEqual(ftp.accepted_reply_generation, accepted_before_read + 2)

    def test_read_renews_deadline_after_nonempty_delayed_receive(self):
        """Progress flushed after a received packet renews the read deadline."""
        class DelayedMaster:  # pylint: disable=too-few-public-methods
            """Return an open reply, then its delayed burst reply."""

            source_system = 1
            source_component = 1

            def __init__(self, clock):
                self.clock = clock
                self.mav = FakeMAV()
                self.ftp = None
                self.calls = 0

            def recv_match(self, **_kwargs):
                self.calls += 1
                if self.calls == 1:
                    self.clock[0] = 4
                    return ftp_reply(
                        self.ftp.last_op.seq + 1,
                        OP_Ack,
                        OP_OpenFileRO,
                        payload=struct.pack("<I", 1),
                        session=7,
                    )
                self.clock[0] = 6
                return ftp_reply(
                    self.ftp.last_op.seq + 1,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x",
                    burst_complete=1,
                    session=7,
                )

        clock = [0]
        master = DelayedMaster(clock)
        with patch.object(mavftp_module.time, "time", side_effect=lambda: clock[0]), patch.object(
            mavftp_module.time, "sleep"
        ):
            bootstrap_master = FakeMaster([ftp_reply(1, OP_Ack, OP_ResetSessions)])
            ftp = MAVFTP(bootstrap_master, target_system=1, target_component=1)
            ftp.master = master
            master.ftp = ftp
            ftp.ftp_settings.pkt_lag_rx = 1
            original_idle_task = ftp.idle_task

            def flush_and_advance_clock():
                if ftp.rx_delay_queue:
                    _deadline, sequence, message = ftp.rx_delay_queue[0]
                    ftp.rx_delay_queue[0] = (time.monotonic() - 1, sequence, message)
                result = original_idle_task()
                if master.calls == 1:
                    clock[0] = 6
                return result

            ftp.idle_task = flush_and_advance_clock
            setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

            self.assertEqual(ftp.read("remote", 1), b"x")

    def test_read_flushes_delayed_tx_packets(self):
        """A synchronous read services the simulated TX queue while waiting."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[4, 0, 0, 0], session=7),
                ftp_reply(3, OP_Ack, OP_BurstReadFile, payload=b"data", burst_complete=1, session=7),
                ftp_reply(4, OP_Ack, OP_TerminateSession, session=7),
            ]
        )
        ftp.ftp_settings.pkt_lag_tx = 50

        self.assertEqual(ftp.read("remote", 4), b"data")
        self.assertTrue(
            any(request.opcode == OP_OpenFileRO for request in self.sent_requests(master))
        )

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

        callback_data = []
        for expected in (b"A" * 319, b"B" * 319):
            ftp.cmd_get(
                ["remote", "-"],
                callback=lambda fh: (
                    callback_data.append(fh.read())
                    or MAVFTPReturn("Get", FtpError.Success)
                ),
            )
            result = ftp.process_ftp_reply("get", timeout=1)
            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(callback_data[-1], expected)
            self.assertIsNone(ftp.get_result)

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
            ("pkt_lag_tx", "-1"),
            ("pkt_lag_jitter_rx", "-1"),
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

    def test_cmd_set_rejects_invalid_crc_compare_timeout(self):
        """The command path applies the same bounds as library settings."""
        ftp, _master = self.make_ftp([])

        result = ftp.cmd_set(["crccmp_timeout", "0"])

        self.assertEqual(result.error_code, FtpError.InvalidArguments)

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

    def test_settings_constructor_normalizes_float_overflow(self):
        """Invalid float defaults use the settings ValueError contract."""
        with self.assertRaises(ValueError):
            MAVFTPSettings([("retry_time", float, 10**1000)])

        settings = MAVFTPSettings([("retry_time", float, 1.0)])
        with self.assertRaises(ValueError):
            settings.retry_time = 10**1000

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

    def test_malformed_file_entry_does_not_discard_the_listing(self):
        """A malformed file entry is skipped while valid entries still complete."""
        ftp, _master = self.make_ftp(
            [
                ftp_reply(
                    2,
                    OP_Ack,
                    OP_ListDirectory,
                    payload=b"Fmissing-size\x00Fvalid.bin\t3\x00",
                ),
                ftp_reply(
                    3,
                    OP_Nack,
                    OP_ListDirectory,
                    payload=[FtpError.EndOfFile],
                ),
            ]
        )
        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            result.directory_listing,
            [DirectoryEntry("valid.bin", False, 3)],
        )

    def test_directory_listing_with_time_preserves_metadata(self):
        """The optional listing extension returns file modification times."""
        ftp, _master = self.make_ftp(
            [
                ftp_reply(
                    2,
                    OP_Ack,
                    OP_ListDirectoryWithTime,
                    payload=b"Ffile.bin\t42\t1700000000\x00Dlogs\t0\t1700000001\x00",
                ),
                ftp_reply(3, OP_Nack, OP_ListDirectoryWithTime, payload=[FtpError.EndOfFile]),
            ],
            list_time=1,
        )
        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            result.directory_listing,
            [
                DirectoryEntry("logs", True, 0, 1700000001),
                DirectoryEntry("file.bin", False, 42, 1700000000),
            ],
        )

    def test_directory_listing_with_time_handles_malformed_mtime(self):
        """A malformed timestamp is retained as an unknown modification time."""
        ftp, _master = self.make_ftp(
            [
                ftp_reply(
                    2,
                    OP_Ack,
                    OP_ListDirectoryWithTime,
                    payload=b"Ffile.bin\t42\tnot-a-timestamp\x00",
                ),
                ftp_reply(3, OP_Nack, OP_ListDirectoryWithTime, payload=[FtpError.EndOfFile]),
            ],
            list_time=1,
        )
        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            result.directory_listing,
            [DirectoryEntry("file.bin", False, 42, 0)],
        )

    def test_directory_listing_with_time_handles_malformed_directory_mtime(self):
        """A malformed directory timestamp is retained as unknown metadata."""
        ftp, _master = self.make_ftp(
            [
                ftp_reply(
                    2,
                    OP_Ack,
                    OP_ListDirectoryWithTime,
                    payload=b"Dlogs\t0\tnot-a-timestamp\x00",
                ),
                ftp_reply(3, OP_Nack, OP_ListDirectoryWithTime, payload=[FtpError.EndOfFile]),
            ],
            list_time=1,
        )
        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            result.directory_listing,
            [DirectoryEntry("logs", True, 0, 0)],
        )

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
        self.assertFalse(ftp.list_time_supported)
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

    def test_silent_timestamp_listing_falls_back_during_cmd_list(self):
        """A silent optional-opcode server completes a whole list operation."""
        master = SilentTimestampListingMaster()
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            ftp = MAVFTP(master, target_system=1, target_component=1)
            ftp.ftp_settings.retry_time = 0.2
            ftp.ftp_settings.list_time_timeout = 0.4
            ftp.ftp_settings.list_retries = 1
            ftp.rtt = 0.0
            ftp.rttvar = 0.0
            ftp.rtt_valid = True

            result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            result.directory_listing,
            [
                DirectoryEntry("logs", True, 0),
                DirectoryEntry("one.bin", False, 1),
            ],
        )
        requests = self.sent_requests(master)
        self.assertEqual(
            [
                (request.opcode, request.seq, request.offset, bytes(request.payload or b""))
                for request in requests
            ],
            [
                (OP_ResetSessions, 0, 0, b""),
                (OP_ListDirectoryWithTime, 1, 0, b"/"),
                (OP_ListDirectoryWithTime, 1, 0, b"/"),
                (OP_ListDirectory, 2, 0, b"/"),
                (OP_ListDirectory, 3, 2, b"/"),
            ],
        )
        # A timeout is transport loss, not proof that the server lacks the
        # extension. Do not permanently downgrade future listings.
        self.assertIsNone(ftp.list_time_supported)

    def test_slow_timestamp_probes_reach_baseline_before_cmd_list_timeout(self):
        """The list deadline covers RTT-scaled timestamp probes and fallback."""
        ftp, _master = self.make_ftp([], list_time=1)
        ftp.ftp_settings.list_time_timeout = 3.0
        ftp.ftp_settings.list_retries = 3
        ftp.ftp_settings.retry_time = 0.5
        ftp.rtt = 2.0
        ftp.rttvar = 0.5
        ftp.rtt_valid = True
        process_reply = MagicMock(
            return_value=MAVFTPReturn("ListDirectory", FtpError.Success)
        )
        ftp.process_ftp_reply = process_reply

        result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        process_reply.assert_called_once()
        self.assertEqual(process_reply.call_args.args, ("ListDirectory",))
        self.assertAlmostEqual(process_reply.call_args.kwargs["timeout"], 60.02)

    def test_baseline_listing_uses_the_full_initial_retry_budget(self):
        """A non-timestamp listing has enough time for every initial retry."""
        ftp, _master = self.make_ftp([], list_time=0)
        ftp.process_ftp_reply = MagicMock(
            return_value=MAVFTPReturn("ListDirectory", FtpError.Success)
        )

        with patch.object(
            ftp, "_MAVFTP__initial_request_retry_budget", return_value=11.0
        ):
            result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        ftp.process_ftp_reply.assert_called_once_with(
            "ListDirectory", timeout=11.0
        )

    def test_timestamp_nack_fail_does_not_latch_extension_as_unsupported(self):
        """A generic timestamp NACK falls back once but keeps future probes enabled."""
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Nack, OP_ListDirectoryWithTime, payload=[FtpError.Fail]),
                ftp_reply(3, OP_Nack, OP_ListDirectory, payload=[FtpError.EndOfFile]),
                ftp_reply(
                    4,
                    OP_Nack,
                    OP_ListDirectoryWithTime,
                    payload=[FtpError.EndOfFile],
                ),
            ],
            list_time=1,
        )

        self.assertEqual(ftp.cmd_list([]).error_code, FtpError.Success)
        self.assertIsNone(ftp.list_time_supported)
        self.assertEqual(ftp.cmd_list([]).error_code, FtpError.Success)
        self.assertEqual(
            [request.opcode for request in self.sent_requests(master)],
            [OP_ResetSessions, OP_ListDirectoryWithTime, OP_ListDirectory, OP_ListDirectoryWithTime],
        )

    def test_open_retry_exhaustion_sets_terminal_timeout_for_reply_loop(self):
        """The final failed OpenFileRO retry sets the reply-loop timeout latch."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])
        # Keep this focused on the terminal result from the OpenFileRO cap;
        # a real download would subsequently attempt session cleanup too.
        ftp.filename = None
        ftp.op_start = 0.0
        ftp.open_retries = MAX_READ_RETRIES
        terminate = MagicMock()

        with (
            patch.object(ftp, "_MAVFTP__terminate_session", terminate),
            patch("pymavlink.mavftp.time.time", return_value=1.0),
        ):
            self.assertFalse(ftp.idle_task())

        self.assertTrue(ftp.terminal_timeout)
        terminate.assert_called_once()

    def test_process_reply_does_not_expose_an_unused_initial_request_deadline(self):
        """Retry-deadline implementation detail is not retained as dead state."""
        ftp, _master = self.make_ftp([])

        self.assertFalse(hasattr(ftp, "initial_request_deadline"))

    def test_process_reply_honors_an_explicit_initial_request_timeout(self):
        """An explicit reply-loop timeout is never replaced with a retry budget."""
        ftp, _master = self.make_ftp([])
        ftp.last_op = FTP_OP(1, 0, OP_RemoveFile, 1, 0, 0, 0, bytearray(b"x"))
        ftp.idle_task = MagicMock(return_value=True)

        with patch.object(
            ftp,
            "_MAVFTP__initial_request_retry_budget",
            side_effect=AssertionError("explicit timeout was overridden"),
        ):
            result = ftp.process_ftp_reply("RemoveFile", timeout=1.0)

        self.assertEqual(result.error_code, FtpError.Fail)

    def test_default_put_reply_loop_uses_the_create_file_retry_budget(self):
        """An omitted put timeout leaves time for the CreateFile retry ladder."""
        ftp, _master = self.make_ftp([])
        self.assertEqual(
            ftp.cmd_put(["local", "remote"], fh=BytesIO(b"data")).error_code,
            FtpError.Success,
        )
        ftp.idle_task = MagicMock(return_value=True)

        with patch.object(
            ftp, "_MAVFTP__initial_request_retry_budget", return_value=11.0
        ) as retry_budget:
            result = ftp.process_ftp_reply("put")

        self.assertEqual(result.error_code, FtpError.Fail)
        retry_budget.assert_called_once_with()

    def test_mutating_commands_request_the_full_initial_retry_budget(self):
        """Default mutation commands opt into the complete retry ladder."""
        commands = (
            ("cmd_rm", ["remote"], "RemoveFile"),
            ("cmd_rmdir", ["remote"], "RemoveDirectory"),
            ("cmd_rename", ["old", "new"], "Rename"),
            ("cmd_mkdir", ["remote"], "CreateDirectory"),
        )
        for method_name, args, operation_name in commands:
            with self.subTest(command=method_name):
                ftp, _master = self.make_ftp([])
                ftp.process_ftp_reply = MagicMock(
                    return_value=MAVFTPReturn(operation_name, FtpError.Success)
                )
                with patch.object(
                    ftp, "_MAVFTP__initial_request_retry_budget", return_value=11.0
                ):
                    result = getattr(ftp, method_name)(args)

                self.assertEqual(result.error_code, FtpError.Success)
                ftp.process_ftp_reply.assert_called_once_with(
                    operation_name, timeout=11.0
                )

    def test_command_callers_can_override_the_extended_default_timeout(self):
        """Public command helpers preserve a caller-supplied reply timeout."""
        commands = (
            ("cmd_list", [], "ListDirectory"),
            ("cmd_rm", ["remote"], "RemoveFile"),
            ("cmd_rmdir", ["remote"], "RemoveDirectory"),
            ("cmd_rename", ["old", "new"], "Rename"),
            ("cmd_mkdir", ["remote"], "CreateDirectory"),
        )
        for method_name, args, operation_name in commands:
            with self.subTest(command=method_name):
                ftp, _master = self.make_ftp([])
                ftp.process_ftp_reply = MagicMock(
                    return_value=MAVFTPReturn(operation_name, FtpError.Success)
                )

                result = getattr(ftp, method_name)(args, timeout=1.0)

                self.assertEqual(result.error_code, FtpError.Success)
                ftp.process_ftp_reply.assert_called_once_with(
                    operation_name, timeout=1.0
                )

    def test_timestamp_listing_retries_lost_followup_page(self):
        """A lost page request is retried after timestamp support is confirmed."""
        master = LostTimestampPageMaster()
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            ftp = MAVFTP(master, target_system=1, target_component=1)
            ftp.ftp_settings.list_time = 1
            ftp.ftp_settings.retry_time = 0.2
            ftp.ftp_settings.list_time_timeout = 0.4
            ftp.rtt = 0.0
            ftp.rttvar = 0.0
            ftp.rtt_valid = True

            result = ftp.cmd_list([])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            result.directory_listing,
            [
                DirectoryEntry("one.bin", False, 1, 1700000000),
                DirectoryEntry("logs", True, 0, 1700000001),
            ],
        )
        self.assertTrue(master.dropped_page)
        self.assertTrue(ftp.list_time_supported)
        requests = self.sent_requests(master)
        timestamp_requests = [
            request for request in requests if request.opcode == OP_ListDirectoryWithTime
        ]
        self.assertEqual(
            [(request.seq, request.offset) for request in timestamp_requests],
            [(1, 0), (2, 1), (2, 1), (3, 2)],
        )

    def test_confirmed_timestamp_capability_survives_lost_new_listing_page(self):
        """A later lost page is retried without disabling known capability."""
        master = FlakyTimestampListingMaster()
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            ftp = MAVFTP(master, target_system=1, target_component=1)
            ftp.ftp_settings.list_time = 1
            ftp.ftp_settings.retry_time = 0.2
            ftp.ftp_settings.list_time_timeout = 0.4
            ftp.ftp_settings.list_retries = 0
            ftp.rtt = 0.0
            ftp.rttvar = 0.0
            ftp.rtt_valid = True

            first_result = ftp.cmd_list([])
            second_result = ftp.cmd_list([])

        self.assertEqual(first_result.error_code, FtpError.Success)
        self.assertEqual(
            first_result.directory_listing,
            [DirectoryEntry("one.bin", False, 1, 1700000000)],
        )
        self.assertEqual(second_result.error_code, FtpError.Success)
        self.assertEqual(
            second_result.directory_listing,
            [DirectoryEntry("one.bin", False, 1, 1700000002)],
        )
        self.assertTrue(master.dropped_page)
        self.assertTrue(ftp.list_time_supported)
        requests = self.sent_requests(master)
        self.assertNotIn(OP_ListDirectory, [request.opcode for request in requests])
        timestamp_requests = [
            request for request in requests if request.opcode == OP_ListDirectoryWithTime
        ]
        self.assertEqual(
            [(request.seq, request.offset) for request in timestamp_requests],
            [(1, 0), (2, 1), (3, 0), (3, 0), (4, 1)],
        )

    def test_timestamp_listing_retries_after_no_sessions_once_capable(self):
        """NoSessionsAvailable does not force a capable server to baseline mode."""
        master = NoSessionsTimestampListingMaster()
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            ftp = MAVFTP(master, target_system=1, target_component=1)
            ftp.ftp_settings.list_time = 1
            ftp.ftp_settings.retry_time = 0.2
            ftp.ftp_settings.list_time_timeout = 0.4
            ftp.ftp_settings.list_retries = 0
            ftp.rtt = 0.0
            ftp.rttvar = 0.0
            ftp.rtt_valid = True

            first_result = ftp.cmd_list([])
            second_result = ftp.cmd_list([])

        self.assertEqual(first_result.error_code, FtpError.Success)
        self.assertEqual(
            first_result.directory_listing,
            [DirectoryEntry("one.bin", False, 1, 1700000000)],
        )
        self.assertEqual(second_result.error_code, FtpError.Success)
        self.assertEqual(
            second_result.directory_listing,
            [DirectoryEntry("one.bin", False, 1, 1700000002)],
        )
        self.assertTrue(master.no_sessions_sent)
        self.assertTrue(ftp.list_time_supported)
        requests = self.sent_requests(master)
        self.assertNotIn(OP_ListDirectory, [request.opcode for request in requests])
        timestamp_requests = [
            request for request in requests if request.opcode == OP_ListDirectoryWithTime
        ]
        self.assertEqual(
            [(request.seq, request.offset) for request in timestamp_requests],
            [(1, 0), (2, 1), (3, 0), (3, 0), (4, 1)],
        )

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

    def test_local_crc_accumulates_across_read_blocks(self):
        """Local CRCs continue across the 64 KiB file-read boundary."""
        with tempfile.NamedTemporaryFile() as local_file:
            local_file.write(b"a" * 65536 + b"b")
            local_file.flush()

            self.assertEqual(local_file_crc(local_file.name), 0xB9726316)

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

    def test_crccmp_clears_results_before_preflight_failure(self):
        """A failed comparison does not expose results from an earlier call."""
        ftp, _master = self.make_ftp([])
        ftp.crccmp_results = ["MATCH", "MATCH"]

        result = ftp.cmd_crccmp(["no-such-file-*.bin", "/remote"])

        self.assertEqual(result.error_code, FtpError.FileNotFound)
        self.assertEqual(ftp.crccmp_results, [])

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

    def test_crc_waits_for_a_long_remote_calculation_without_retrying(self):
        """A CRC request may outlive idle detection but is sent only once."""
        ftp, master = self.make_ftp([])
        master.replies.append(
            ftp_reply(
                2,
                OP_Ack,
                OP_CalcFileCRC32,
                payload=struct.pack("<I", 0x12345678),
            )
        )
        master.empty_polls = 5
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            result = ftp.cmd_crc(["remote"], timeout=1.0)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(
            len(
                [
                    sent
                    for sent in master.mav.sent
                    if sent[-1][3] == OP_CalcFileCRC32
                ]
            ),
            1,
        )

    def test_crc_uses_caller_timeout_when_remote_calculation_never_replies(self):
        """A silent CRC request times out at its caller deadline, not idle time."""
        ftp, master = self.make_ftp([])
        clock = [0.0]

        def fake_time():
            clock[0] += 0.05
            return clock[0]

        with patch("pymavlink.mavftp.time.time", side_effect=fake_time):
            result = ftp.cmd_crc(["remote"], timeout=1.0)

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)
        self.assertEqual(
            len(
                [
                    sent
                    for sent in master.mav.sent
                    if sent[-1][3] == OP_CalcFileCRC32
                ]
            ),
            1,
        )

    def test_terminate_uses_rtt_timeout_with_one_second_cap(self):
        """Session termination uses the adaptive timeout but remains bounded."""
        for rtt, rttvar, expected_timeout in (
            (0.4, 0.1, 0.8),
            (5.0, 2.0, 1.0),
        ):
            with self.subTest(rtt=rtt, rttvar=rttvar):
                ftp, _master = self.make_ftp([])
                ftp.rtt_valid = True
                ftp.rtt = rtt
                ftp.rttvar = rttvar

                with patch.object(
                    ftp,
                    "process_ftp_reply",
                    return_value=MAVFTPReturn("TerminateSession", FtpError.Success),
                ) as process_reply:
                    result = ftp._MAVFTP__terminate_session()  # pylint: disable=protected-access

                self.assertEqual(result.error_code, FtpError.Success)
                process_reply.assert_called_once_with(
                    "TerminateSession", timeout=expected_timeout
                )

    def test_packet_lag_is_nonblocking_and_bidirectional(self):
        """Configured link lag queues packets until idle processing delivers them."""
        ftp, master = self.make_ftp([])
        ftp.ftp_settings.pkt_lag_tx = 100
        ftp._MAVFTP__send(  # pylint: disable=protected-access
            FTP_OP(ftp.seq, ftp.session, OP_RemoveFile, 1, 0, 0, 0, bytearray(b"x"))
        )
        self.assertEqual(len(master.mav.sent), 1)  # ResetSessions
        self.assertEqual(len(ftp.tx_delay_queue), 1)

        _deadline, sequence, payload = ftp.tx_delay_queue[0]
        ftp.tx_delay_queue[0] = (time.monotonic() - 1, sequence, payload)
        ftp.idle_task()
        self.assertEqual(len(master.mav.sent), 2)  # reset plus the delayed request

        # A download starter does not enter the blocking reply loop, so its
        # reply can be exercised through the public event-driven hook.
        ftp.ftp_settings.pkt_lag_tx = 0
        ftp.ftp_settings.pkt_lag_rx = 100
        ftp.cmd_get(["remote.bin", "-"])
        reply = ftp_reply(ftp.last_op.seq + 1, OP_Ack, OP_OpenFileRO,
                          payload=struct.pack("<I", 1), session=7)
        self.assertIsNone(ftp.mavlink_packet(reply))
        self.assertEqual(len(ftp.rx_delay_queue), 1)

        _deadline, sequence, message = ftp.rx_delay_queue[0]
        ftp.rx_delay_queue[0] = (time.monotonic() - 1, sequence, message)
        ftp.idle_task()
        self.assertIsNotNone(ftp.fh)
        self.assertEqual(
            self.sent_requests(master)[-1].opcode,
            OP_BurstReadFile,
        )

    def test_timed_out_lagged_remove_is_not_sent_after_return(self):
        """A timed-out remove must cancel its delayed simulated TX packet."""
        ftp, master = self.make_ftp([])
        ftp.ftp_settings.pkt_lag_tx = 1000

        result = ftp.cmd_rm(["late-delete"], timeout=0.01)

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)
        self.assertEqual(ftp.tx_delay_queue, [])
        self.assertTrue(ftp.request_cancelled)
        ftp.idle_task()
        self.assertEqual(
            [request for request in self.sent_requests(master) if request.opcode == OP_RemoveFile],
            [],
        )

    def test_tx_loss_does_not_discard_received_burst_reply(self):
        """Transmit loss applies only to outgoing requests, never incoming data."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_to_memory = True
        ftp.requested_size = 1
        ftp.op_start = 1
        ftp.burst_size = 80
        ftp.session = 7
        ftp.ftp_settings.pkt_loss_tx = 100

        result = ftp._MAVFTP__handle_burst_read(
            FTP_OP(2, 7, OP_Ack, 1, OP_BurstReadFile, 1, 0, bytearray(b"x")),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.get_result, b"x")

    def test_synchronous_operation_preserves_delayed_reply_result(self):
        """A delayed ACK still completes the operation that is waiting for it."""
        ftp, master = self.make_ftp([])
        ftp.ftp_settings.pkt_lag_rx = 1
        master.replies.append(ftp_reply(ftp.seq + 1, OP_Ack, OP_RemoveFile))

        result = ftp.cmd_rm(["remote.bin"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(master.replies, [])
        self.assertEqual(self.sent_requests(master)[-1].opcode, OP_RemoveFile)

    def test_delayed_reply_wins_over_a_simultaneous_terminal_timeout(self):
        """A matching delayed reply cannot be overwritten by a timeout."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_rm(["remote"])

        idle_calls = [0]

        def queue_delayed_reply():
            idle_calls[0] += 1
            if idle_calls[0] == 1:
                ftp.terminal_timeout = True
                ftp.delayed_rx_results.append(
                    (MAVFTPReturn("RemoveFile", FtpError.Success), True, True, False, True)
                )
                return False
            return True

        ftp.idle_task = queue_delayed_reply
        result = ftp.process_ftp_reply("RemoveFile", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)

    def test_loss_seed_replays_jitter_sequence(self):
        """The loss seed makes latency jitter reproducible between clients."""
        first, _master = self.make_ftp([])
        second, _master = self.make_ftp([])
        for ftp in (first, second):
            ftp.ftp_settings.loss_seed = 1234
            ftp.ftp_settings.pkt_lag_tx = 100
            ftp.ftp_settings.pkt_lag_jitter_tx = 500
            ftp._MAVFTP__packet_lost("TX")  # pylint: disable=protected-access

        first_delays = [
            first._MAVFTP__packet_delay("TX")  # pylint: disable=protected-access
            for _ in range(10)
        ]
        second_delays = [
            second._MAVFTP__packet_delay("TX")  # pylint: disable=protected-access
            for _ in range(10)
        ]
        self.assertEqual(first_delays, second_delays)
        self.assertTrue(all(0.1 <= delay <= 0.6 for delay in first_delays))

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
        self.assertTrue(all(observed is link for observed in ftp.master.mav.observed_files))
        self.assertTrue(all(len(write) <= mavftp_module.MAX_NETWORK_BATCH for write in link.writes))
        encoded = b"".join(link.writes)
        self.assertEqual(len(encoded), 8 * 252)
        decoded = []
        for frame_start in range(0, len(encoded), 252):
            frame = encoded[frame_start : frame_start + 252]
            self.assertEqual(frame[:1], b"F")
            payload = frame[1:]
            fields = struct.unpack_from("<HBBBBBBI", payload)
            size = fields[3]
            offset = fields[7]
            decoded.append((fields[0], fields[2], size, offset, payload[12 : 12 + size]))

        self.assertEqual(
            decoded,
            [
                (index + 1, OP_WriteFile, 1, index, bytes([index]))
                for index in range(8)
            ],
        )

    def test_signed_batch_writes_before_send_callbacks_can_interleave(self):
        """Signed FTP sends are serialized instead of packing packets ahead of callbacks."""
        ftp, _master = self.make_ftp([])
        link = BatchLink()
        mav = SignedBatchMAV(link)
        ftp.master.mav = mav
        operations = [
            FTP_OP(ftp.seq, ftp.session, OP_WriteFile, 1, 0, 0, index, bytearray([index]))
            for index in range(2)
        ]

        ftp._MAVFTP__send_batch(operations)  # pylint: disable=protected-access

        self.assertEqual(mav.encoded, 0)
        self.assertEqual([write[:1] for write in link.writes], [b"F", b"H", b"F", b"H"])

    def test_batch_send_without_mavlink_master_falls_back_cleanly(self):
        """Batching must retain the normal no-master safety guard."""
        ftp, _master = self.make_ftp([])
        ftp.master = object()
        operations = [
            FTP_OP(ftp.seq, ftp.session, OP_WriteFile, 1, 0, 0, index, bytearray([index]))
            for index in range(2)
        ]

        ftp._MAVFTP__send_batch(operations)  # pylint: disable=protected-access

    def test_new_features_are_exposed_by_cli(self):
        """The timestamp, CRC, and comparison options are parser-visible."""
        parser = create_argument_parser()
        timeout_action = next(
            action for action in parser._actions if action.dest == "crccmp_timeout"
        )
        self.assertIn("whole CRC comparison batch", timeout_action.help)

        args = parser.parse_args(
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
        self.assertEqual(list_args.idle_detection_time, 3.7)
        for retry_time in (0.5, 1.0, 1.1, 1.2, 1.5, 2.0, 3.0):
            with self.subTest(read_retry_time=retry_time):
                retry_args = create_argument_parser().parse_args(
                    ["--read_retry_time", str(retry_time), "list"]
                )
                self.assertEqual(retry_args.read_retry_time, retry_time)

        lag_args = create_argument_parser().parse_args(
            ["--pkt_lag_tx", "100", "--pkt_lag_jitter_rx", "25", "list"]
        )
        self.assertEqual(lag_args.pkt_lag_tx, 100.0)
        self.assertEqual(lag_args.pkt_lag_jitter_rx, 25.0)

    def test_example_parser_allows_programmatic_default_namespace(self):
        """The example parser remains usable by callers that provide no argv."""
        args = mavftp_example.argument_parser([])

        self.assertIsNone(args.local_file)
        self.assertIsNone(args.remote_file)

    def test_example_main_reports_missing_paths_as_an_argument_error(self):
        """The command-line entry point keeps argparse's usage diagnostics."""
        stderr = StringIO()

        with patch.object(sys, "stderr", stderr), self.assertRaises(SystemExit) as error:
            mavftp_example.main([])

        self.assertEqual(error.exception.code, 2)
        self.assertIn("usage:", stderr.getvalue())
        self.assertIn("LOCAL_FILE and REMOTE_FILE are required", stderr.getvalue())

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

    def test_gap_read_ack_releases_one_backlog_slot(self):
        """A gap ACK must release exactly the request's backlog slot."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_gaps = [(0, 2), (2, 2)]
        ftp.read_gap_times = {(0, 2): 0, (2, 2): 0}

        ftp._MAVFTP__send_gap_read((0, 2))  # pylint: disable=protected-access
        ftp._MAVFTP__send_gap_read((2, 2))  # pylint: disable=protected-access

        result = ftp._MAVFTP__handle_reply_read(  # pylint: disable=protected-access
            FTP_OP(2, 0, OP_Ack, 2, OP_ReadFile, 0, 0, bytearray(b"ab")),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.backlog, 1)

    def test_open_retry_reuses_request_sequence(self):
        """Retransmitting an unanswered OpenFileRO preserves its sequence."""
        ftp, master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])
        ftp.op_start = 0

        with patch("pymavlink.mavftp.time.time", return_value=1):
            ftp._MAVFTP__idle_task()  # pylint: disable=protected-access

        self.assertEqual(self.sent_request_sequences(master, OP_OpenFileRO), [1, 1])

    def test_no_sessions_available_does_not_leave_unused_session_state(self):
        """NoSessionsAvailable uses the retry timer without a shadow flag."""
        ftp, _master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(
                2,
                OP_Nack,
                OP_OpenFileRO,
                payload=[FtpError.NoSessionsAvailable],
            )
        )

        self.assertEqual(result.error_code, FtpError.NoSessionsAvailable)
        self.assertNotIn("session_waiting", ftp.__dict__)

    def test_open_no_sessions_reply_does_not_mutate_unused_reply_state(self):
        """The open-session retry path does not write an unconsumed flag."""
        ftp, _master = self.make_ftp([])
        ftp.last_op_reply = True

        result = ftp._MAVFTP__handle_open_ro_reply(  # pylint: disable=protected-access
            FTP_OP(
                2,
                0,
                OP_Nack,
                1,
                OP_OpenFileRO,
                0,
                0,
                bytearray([FtpError.NoSessionsAvailable]),
            ),
            None,
        )

        self.assertEqual(result.error_code, FtpError.NoSessionsAvailable)
        self.assertTrue(ftp.last_op_reply)

    def test_open_retry_waits_for_sessions_but_still_has_a_hard_cap(self):
        """NoSessionsAvailable extends, but never removes, the open retry budget."""
        ftp, master = self.make_ftp([])
        ftp.cmd_get(["remote", "-"])
        terminated = MagicMock()
        setattr(ftp, "_MAVFTP__terminate_session", terminated)

        for _ in range(MAX_READ_RETRIES):
            ftp.op_start = 0
            with patch("pymavlink.mavftp.time.time", return_value=1):
                ftp._MAVFTP__idle_task()  # pylint: disable=protected-access

        terminated.assert_not_called()

        ftp.op_start = 0
        with patch("pymavlink.mavftp.time.time", return_value=1):
            ftp._MAVFTP__idle_task()  # pylint: disable=protected-access

        self.assertEqual(
            self.sent_request_sequences(master, OP_OpenFileRO),
            [1] * (MAX_READ_RETRIES + 1),
        )
        terminated.assert_called_once()

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

    def test_write_ack_restarts_retry_window_for_remaining_batch(self):
        """An ACK refreshes the loss timer while other writes remain in flight."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"ab")
        ftp.filename = "remote"
        ftp.write_open = True
        ftp.write_list = {0, 1}
        ftp.write_block_size = 1
        ftp.write_total = 2
        ftp.ftp_settings.write_qsize = 2
        ftp._MAVFTP__send_more_writes()  # pylint: disable=protected-access
        reply_seq = next(iter(ftp.pending_write_replies))

        with patch("pymavlink.mavftp.time.time", return_value=10.0):
            result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
                FTP_OP(
                    reply_seq,
                    0,
                    OP_Ack,
                    0,
                    OP_WriteFile,
                    0,
                    ftp.pending_write_replies[reply_seq],
                    bytearray(),
                ),
                None,
            )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.write_last_send, 10.0)
        self.assertEqual(ftp.write_inflight, {1})

    def test_reordered_write_ack_keeps_the_newer_inflight_window(self):
        """A late ACK cannot release slots after the receive cursor."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"abcd")
        ftp.filename = "remote"
        ftp.write_list = {0, 1, 2, 3}
        ftp.write_block_size = 1
        ftp.write_total = 4
        ftp.write_recv_idx = -1
        ftp.write_inflight = {0, 1, 2, 3}
        ftp.write_pending = 4
        ftp.pending_write_replies = {10: 2, 11: 1}

        with patch.object(ftp, "_MAVFTP__send_more_writes"):
            result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
                FTP_OP(10, 0, OP_Ack, 0, OP_WriteFile, 0, 2, bytearray()),
                None,
            )
            self.assertEqual(result.error_code, FtpError.Success)
            self.assertEqual(ftp.write_inflight, {3})
            self.assertEqual(ftp.write_list, {0, 1, 3})

            result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
                FTP_OP(11, 0, OP_Ack, 0, OP_WriteFile, 0, 1, bytearray()),
                None,
            )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.write_recv_idx, 2)
        self.assertEqual(ftp.write_inflight, {3})
        self.assertEqual(ftp.write_list, {0, 3})

    def test_stale_write_ack_cannot_release_a_half_ring_inflight_block(self):
        """A duplicate ACK must not alter the active write window."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"abcd")
        ftp.filename = "remote"
        ftp.write_list = {3}
        ftp.write_block_size = 1
        ftp.write_total = 4
        ftp.write_recv_idx = 2
        ftp.write_inflight = {3}
        ftp.write_pending = 1

        with patch.object(ftp, "_MAVFTP__send_more_writes") as send_more:
            result = ftp._MAVFTP__handle_write_reply(  # pylint: disable=protected-access
                FTP_OP(10, 0, OP_Ack, 0, OP_WriteFile, 0, 0, bytearray()),
                None,
            )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.write_recv_idx, 2)
        self.assertEqual(ftp.write_inflight, {3})
        self.assertEqual(ftp.write_list, {3})
        send_more.assert_not_called()

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

    def test_read_sector_accepts_eof_before_advertised_size(self):
        """A sector at EOF returns its available bytes when stat size is an estimate."""
        ftp, _master = self.make_ftp(
            [
                ftp_reply(
                    2,
                    OP_Ack,
                    OP_OpenFileRO,
                    payload=struct.pack("<I", 1024),
                    session=7,
                ),
                ftp_reply(
                    3,
                    OP_Ack,
                    OP_BurstReadFile,
                    payload=b"x" * 224,
                    offset=800,
                    burst_complete=1,
                    session=7,
                ),
                ftp_reply(4, OP_Ack, OP_TerminateSession, session=7),
            ]
        )

        self.assertEqual(ftp.read_sector("remote", 800, 500), b"x" * 224)

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

    def test_download_rejects_payload_before_requested_memory_range(self):
        """A reply before a range-read start must fail cleanly instead of raising."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote"
        ftp.read_to_memory = True
        ftp.requested_offset = 100
        terminated = MagicMock()
        setattr(ftp, "_MAVFTP__terminate_session", terminated)

        result = ftp._MAVFTP__write_payload(  # pylint: disable=protected-access
            FTP_OP(1, 0, OP_Ack, 1, OP_BurstReadFile, 0, 99, bytearray(b"x"))
        )

        self.assertFalse(result)
        self.assertEqual(ftp.callback_failure.error_code, FtpError.InvalidDataSize)
        terminated.assert_called_once()

    def test_download_rejects_implausibly_far_payload_offset(self):
        """A peer cannot make the staging file sparse with a far-ahead offset."""
        ftp, _master = self.make_ftp([])
        ftp.fh = MagicMock()
        ftp.fh.tell.return_value = 0
        ftp.filename = "remote"
        ftp.remote_size_known = True
        ftp.remote_file_size = 4
        ftp.requested_size = 4
        terminated = MagicMock()
        setattr(ftp, "_MAVFTP__terminate_session", terminated)

        result = ftp._MAVFTP__write_payload(  # pylint: disable=protected-access
            FTP_OP(1, 0, OP_Ack, 1, OP_BurstReadFile, 0, 0xFFFFFF00, bytearray(b"x"))
        )

        self.assertFalse(result)
        self.assertEqual(ftp.callback_failure.error_code, FtpError.InvalidDataSize)
        ftp.fh.seek.assert_not_called()
        terminated.assert_called_once()

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

        stdout = BinaryStdout()
        with patch.object(sys, "stdout", stdout):
            self.assertTrue(ftp._MAVFTP__check_read_finished())
        self.assertEqual(stdout.buffer.getvalue(), b"x" * 200)
        self.assertEqual(ftp.get_result, b"x" * 200)

    def test_stdout_download_falls_back_to_text_stdout(self):
        """A StringIO stdout accepts a completed '-' download."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"text output")
        ftp.filename = "-"
        ftp.requested_size = 11
        ftp.read_total = 11
        ftp.reached_eof = True
        ftp.op_start = 1
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        stdout = StringIO()
        with patch.object(sys, "stdout", stdout):
            self.assertTrue(ftp._MAVFTP__check_read_finished())
        self.assertEqual(stdout.getvalue(), "text output")
        self.assertEqual(ftp.get_result, b"text output")

    def test_stdout_download_callback_owns_data(self):
        """A callback suppresses stdout publication for a '-' download."""
        ftp, _master = self.make_ftp([])
        callback_data = []
        ftp.fh = BytesIO(b"callback output")
        ftp.filename = "-"
        ftp.callback = lambda fh: callback_data.append(fh.read())
        ftp.requested_size = 15
        ftp.read_total = 15
        ftp.reached_eof = True
        ftp.op_start = 1
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        stdout = StringIO()
        with patch.object(sys, "stdout", stdout):
            self.assertTrue(ftp._MAVFTP__check_read_finished())
        self.assertEqual(callback_data, [b"callback output"])
        self.assertEqual(stdout.getvalue(), "")
        self.assertIsNone(ftp.callback_failure)

    def test_known_size_download_accepts_eof_before_advertised_size(self):
        """EOF is authoritative when the server advertises an estimated size."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"x" * 2560)
        ftp.filename = "-"
        ftp.remote_size_known = True
        ftp.remote_file_size = 5120
        ftp.requested_size = 5120
        ftp.read_total = 2560
        ftp.reached_eof = True
        ftp.op_start = 1
        ftp.session = 7
        ftp.fh.seek(2560)
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        stdout = BinaryStdout()
        with patch.object(sys, "stdout", stdout):
            self.assertTrue(ftp._MAVFTP__check_read_finished())
        self.assertIsNone(ftp.callback_failure)
        self.assertEqual(stdout.buffer.getvalue(), b"x" * 2560)

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

    def test_duplicate_burst_reply_does_not_count_as_progress(self):
        """A duplicate burst packet must not renew read progress or stall time."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote.bin"
        ftp.read_to_memory = True
        ftp.requested_offset = 0
        ftp.requested_size = 2
        ftp.burst_size = 239
        ftp.session = 1
        ftp.pending_burst_seq = 1
        ftp.pending_burst_offset = 0
        reply = FTP_OP(
            seq=1,
            session=1,
            opcode=OP_Ack,
            size=1,
            req_opcode=OP_BurstReadFile,
            burst_complete=0,
            offset=0,
            payload=b"x",
        )
        packet = FakeFTPMessage(reply)

        first_result = ftp.mavlink_packet(packet)
        generation_after_first = ftp.accepted_reply_generation
        last_burst_after_first = ftp.last_burst_read
        second_result = ftp.mavlink_packet(packet)

        self.assertEqual(first_result.error_code, FtpError.Success)
        self.assertEqual(second_result.error_code, FtpError.Fail)
        self.assertEqual(ftp.accepted_reply_generation, generation_after_first)
        self.assertEqual(ftp.last_burst_read, last_burst_after_first)
        self.assertEqual(ftp.duplicates, 1)

    def test_zero_byte_advancing_burst_does_not_count_gap_creation_as_progress(self):
        """An empty out-of-order burst reply must not renew the read deadline."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote.bin"
        ftp.read_to_memory = True
        ftp.requested_offset = 0
        ftp.requested_size = 2
        ftp.burst_size = 239
        ftp.session = 1
        ftp.pending_burst_seq = 1
        ftp.pending_burst_offset = 0
        ftp.last_burst_read = 10.0
        accepted_before = ftp.accepted_reply_generation
        packet = FakeFTPMessage(
            FTP_OP(
                seq=1,
                session=1,
                opcode=OP_Ack,
                size=0,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=1,
                payload=b"",
            )
        )

        result = ftp.mavlink_packet(packet)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.read_total, 0)
        self.assertFalse(ftp.reached_eof)
        self.assertEqual(len(ftp.read_gaps), 1)
        self.assertEqual(ftp.accepted_reply_generation, accepted_before)
        self.assertEqual(ftp.last_burst_read, 10.0)

    def test_termination_cleanup_does_not_count_as_read_progress(self):
        """Resetting EOF during cleanup must not advance burst progress."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote.bin"
        ftp.session = 1
        ftp.pending_burst_seq = 1
        ftp.pending_burst_offset = 0
        ftp.reached_eof = True
        accepted_before = ftp.accepted_reply_generation
        packet = FakeFTPMessage(
            FTP_OP(
                seq=1,
                session=1,
                opcode=OP_Ack,
                size=0,
                req_opcode=OP_BurstReadFile,
                burst_complete=0,
                offset=0,
                payload=b"",
            )
        )

        def cleanup(_op, _message):
            ftp.reached_eof = False
            return MAVFTPReturn("BurstReadFile", FtpError.Success)

        with patch.object(ftp, "_MAVFTP__handle_burst_read", side_effect=cleanup):
            result = ftp.mavlink_packet(packet)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.accepted_reply_generation, accepted_before)

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

    def test_burst_reply_advances_the_next_request_sequence(self):
        """A new transfer cannot reuse the accepted burst reply's sequence."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.seq = 40
        ftp.pending_burst_seq = 40
        ftp.pending_burst_offset = 0

        result = ftp._MAVFTP__handle_burst_read(  # pylint: disable=protected-access
            FTP_OP(42, 0, OP_Ack, 1, OP_BurstReadFile, 0, 2, bytearray(b"x")),
            None,
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.seq, 43)

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

    def test_burst_stall_retry_is_capped(self):
        """A stalled burst must eventually terminate instead of retrying forever."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote.bin"
        ftp.last_burst_read = 0.0
        ftp.pending_burst_request = FTP_OP(
            seq=0,
            session=ftp.session,
            opcode=OP_BurstReadFile,
            size=0,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )
        terminate = MagicMock(side_effect=lambda: setattr(ftp, "fh", None))

        clock = [0.0]

        def advance_time():
            clock[0] += 2.0
            return clock[0]

        with (
            patch.object(ftp, "_MAVFTP__send") as send,
            patch.object(ftp, "_MAVFTP__terminate_session", terminate),
            patch("pymavlink.mavftp.time.time", side_effect=advance_time),
        ):
            for _ in range(MAX_READ_RETRIES + 5):
                ftp._MAVFTP__idle_task()  # pylint: disable=protected-access

        self.assertEqual(send.call_count, MAX_READ_RETRIES)
        terminate.assert_called_once()
        self.assertEqual(ftp.read_retries, MAX_READ_RETRIES)

    def test_burst_retry_exhaustion_reports_timeout_after_termination_cleanup(self):
        """A capped burst download must not return the earlier open success."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote.bin"
        ftp.last_op = FTP_OP(
            0,
            ftp.session,
            OP_BurstReadFile,
            0,
            0,
            0,
            0,
            None,
        )
        ftp.last_burst_read = 0.0
        ftp.read_retries = MAX_READ_RETRIES
        ftp.read_complete = False
        ftp.last_send_time = 0.0
        ftp.op_start = None

        def terminate_session():
            ftp.last_burst_read = None
            ftp.fh = None

        with (
            patch.object(ftp, "_MAVFTP__terminate_session", side_effect=terminate_session),
            patch("pymavlink.mavftp.time.time", return_value=1.0),
        ):
            result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)

    def test_burst_progress_resets_the_consecutive_stall_retry_cap(self):
        """A payload accepted between stalls starts a fresh retry window."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "remote.bin"
        ftp.read_to_memory = True
        ftp.requested_size = 2
        ftp.burst_size = 239
        ftp.session = 1
        ftp.pending_burst_seq = 1
        ftp.pending_burst_offset = 0
        ftp.read_retries = MAX_READ_RETRIES

        result = ftp.mavlink_packet(
            FakeFTPMessage(
                FTP_OP(1, 1, OP_Ack, 1, OP_BurstReadFile, 0, 0, bytearray(b"x"))
            )
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.read_retries, 0)

    def test_out_of_order_gap_reply_is_dispatched(self):
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO()
        ftp.filename = "-"
        ftp.read_gaps = [(0, 2), (2, 2)]
        ftp.read_gap_times = {(0, 2): 0, (2, 2): 0}
        ftp.read_retries = MAX_READ_RETRIES

        ftp._MAVFTP__send_gap_read((0, 2))  # pylint: disable=protected-access
        ftp._MAVFTP__send_gap_read((2, 2))  # pylint: disable=protected-access

        result = ftp._MAVFTP__mavlink_packet(  # pylint: disable=protected-access
            ftp_reply(3, OP_Ack, OP_ReadFile, payload=b"cd", offset=2)
        )

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.read_gaps, [(0, 2)])
        self.assertEqual(ftp.fh.getvalue(), b"\x00\x00cd")
        self.assertEqual(ftp.read_retries, 0)

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

        stdout = BinaryStdout()
        with patch.object(sys, "stdout", stdout):
            result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertTrue(ftp.read_complete)
        self.assertEqual(ftp.read_gaps, [])
        self.assertEqual(stdout.buffer.getvalue(), b"a" * 120 + b"b" * 120)

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
        self.assertEqual(ftp.seq, 0)
        self.assertEqual(self.sent_requests(master)[-1].seq, FTP_SEQ_MODULUS - 1)

    def test_remove_sequence_255_advances_to_256(self):
        """A 16-bit sequence continues from 255 to 256 without wrapping."""
        ftp, master = self.make_ftp([])
        ftp.seq = 255
        master.replies.append(ftp_reply(256, OP_Ack, OP_RemoveFile))

        result = ftp.cmd_rm(["remote"])

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.seq, 256)
        self.assertEqual(self.sent_requests(master)[-1].seq, 255)

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

        self.assertEqual(result.error_code, FtpError.RemoteReplyTimeout)

    def test_completed_put_skips_late_reply_after_termination_timeout(self):
        ftp, master = self.make_ftp(
            [
                ftp_reply(2, OP_Ack, OP_WriteFile),
                ftp_reply(3, OP_Ack, OP_WriteFile),
            ],
            validate_replies=False,
        )
        ftp.pending_terminate_seq = 7

        def complete_operation(_message):
            ftp.completed_reply = (OP_WriteFile, 6)
            return MAVFTPReturn("WriteFile", FtpError.Success)

        setattr(ftp, "_MAVFTP__mavlink_packet", complete_operation)
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(len(master.replies), 1)

    def test_delayed_foreign_reply_cannot_complete_a_put(self):
        """A delayed reply for another GCS cannot satisfy an upload completion."""
        ftp, _master = self.make_ftp([])

        def complete_with_foreign_reply():
            ftp.completed_reply = (OP_WriteFile, 6)
            return True

        ftp.idle_task = complete_with_foreign_reply
        ftp.delayed_rx_results.append(
            (MAVFTPReturn("WriteFile", FtpError.Success), False, False, False, False)
        )

        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Fail)

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
        ftp.ftp_settings.pkt_lag_rx = 1
        result = ftp.process_ftp_reply("put", timeout=1)

        self.assertEqual(result.error_code, FtpError.Fail)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.write_list)
        self.assertEqual(ftp._ftp_reply_processing_depth, 0)

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
        mav_ftp.cmd_cancel.side_effect = ValueError("cancel failed")

        with patch.object(mavftp_module, "create_argument_parser") as create_parser, \
                patch.object(
                    mavftp_module,
                    "auto_connect",
                    return_value=Namespace(device="/dev/test"),
                ), \
                patch.object(
                    mavftp_module.mavutil,
                    "mavlink_connection",
                    return_value=master,
                ), \
                patch.object(mavftp_module, "wait_heartbeat"), \
                patch.object(mavftp_module, "MAVFTP", return_value=mav_ftp):
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

        with patch.object(mavftp_module, "create_argument_parser") as create_parser, \
                patch.object(
                    mavftp_module,
                    "auto_connect",
                    return_value=Namespace(device="/dev/test"),
                ), \
                patch.object(
                    mavftp_module.mavutil,
                    "mavlink_connection",
                    return_value=master,
                ), \
                patch.object(mavftp_module, "wait_heartbeat"), \
                patch.object(mavftp_module, "MAVFTP", return_value=mav_ftp):
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

        with patch.object(mavftp_module, "create_argument_parser", return_value=parser), \
                patch.object(
                    mavftp_module,
                    "auto_connect",
                    return_value=Namespace(device="/dev/test"),
                ), \
                patch.object(
                    mavftp_module.mavutil,
                    "mavlink_connection",
                    return_value=MagicMock(target_system=1, target_component=1),
                ), \
                patch.object(mavftp_module, "wait_heartbeat"):
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
            ftp.ftp_settings.pkt_lag_rx = 1
            result = ftp.process_ftp_reply("getparams", timeout=1)

            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertFalse(os.path.exists(destination))
            self.assertEqual(ftp._ftp_reply_processing_depth, 0)

    def test_lagged_download_callback_exception_returns_failure(self):
        """A callback exception remains a failure when RX delivery is delayed."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = f"{tempdir}/download.bin"
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

            callback_calls = []

            def failing_callback(_fh):
                callback_calls.append(True)
                raise RuntimeError("decode failed")

            ftp.cmd_get(["remote.bin", destination], callback=failing_callback)
            ftp.ftp_settings.pkt_lag_rx = 1
            result = ftp.process_ftp_reply("get", timeout=1)

            self.assertEqual(result.error_code, FtpError.Fail)
            self.assertEqual(callback_calls, [True])
            self.assertFalse(os.path.exists(destination))
            self.assertEqual(ftp._ftp_reply_processing_depth, 0)

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

    def test_callback_closing_download_buffer_succeeds(self):
        """A callback owns its buffer, including the option to close it."""
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

        def closing_callback(fh):
            fh.close()

        ftp.cmd_get(["remote.bin", "ignored.bin"], callback=closing_callback)
        result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertTrue(ftp.read_complete)
        self.assertIsNone(ftp.fh)
        self.assertIsNone(ftp.callback_failure)

    def test_release_staging_ignores_closed_owned_handle(self):
        """Staging cleanup must not leak ValueError from an owned closed handle."""
        ftp, _master = self.make_ftp([])
        handle = MagicMock()
        handle.close.side_effect = ValueError("I/O operation on closed file")
        ftp.fh = handle
        ftp.fh_owned = True

        ftp._MAVFTP__release_staging()

        handle.close.assert_called_once_with()
        self.assertFalse(ftp.fh_owned)

    def test_finalization_failure_preserves_callback_failure(self):
        """A later local-I/O failure must retain the callback's diagnostic result."""
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

        def failing_callback(fh):
            failing_handle = MagicMock(wraps=fh)
            failing_handle.flush.side_effect = OSError("disk full")
            ftp.fh = failing_handle
            return MAVFTPReturn("GetParams", FtpError.InvalidDataSize)

        ftp.cmd_get(["remote.bin", "ignored.bin"], callback=failing_callback)
        result = ftp.process_ftp_reply("get", timeout=1)

        self.assertEqual(result.operation_name, "GetParams")
        self.assertEqual(result.error_code, FtpError.InvalidDataSize)
        self.assertTrue(ftp.read_complete)
        self.assertIsNone(ftp.fh)

    def test_callback_success_does_not_publish_download(self):
        """Callbacks accept virtual files shorter than their advertised estimate."""
        with tempfile.TemporaryDirectory() as tempdir:
            destination = f"{tempdir}/param.pck"
            callback_data = []
            ftp, _master = self.make_ftp(
                [
                    ftp_reply(2, OP_Ack, OP_OpenFileRO, payload=[8, 0, 0, 0]),
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

    def test_callback_short_read_warns_when_remote_size_is_known(self):
        """A callback download keeps the short-read diagnostic."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"data")
        ftp.fh.seek(4)
        ftp.filename = "virtual-file"
        ftp.op_start = 1
        ftp.requested_size = 1000
        ftp.read_total = 4
        ftp.reached_eof = True
        ftp.remote_size_known = True
        ftp.callback = lambda _fh: None
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        with self.assertLogs(level="WARNING") as logs:
            self.assertTrue(ftp._MAVFTP__check_read_finished())

        self.assertTrue(any("expected 1000, got 4" in line for line in logs.output))

    def test_callback_short_read_updates_unknown_remote_size(self):
        """A callback download retains the normal estimated-size fixup."""
        ftp, _master = self.make_ftp([])
        ftp.fh = BytesIO(b"data")
        ftp.fh.seek(4)
        ftp.filename = "virtual-file"
        ftp.op_start = 1
        ftp.requested_size = 1000
        ftp.read_total = 4
        ftp.reached_eof = True
        ftp.callback = lambda _fh: None
        setattr(ftp, "_MAVFTP__terminate_session", lambda: None)

        self.assertTrue(ftp._MAVFTP__check_read_finished())

        self.assertEqual(ftp.requested_size, 4)

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
        self.handler = logging.StreamHandler(self.log_stream)
        formatter = logging.Formatter('%(levelname)s: %(message)s')
        self.handler.setFormatter(formatter)
        self.logger = logging.getLogger()
        self.log_level = self.logger.level
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.DEBUG)

        # Mock mavutil.mavlink_connection to simulate a connection
        self.mock_master = mavutil.mavlink_connection(device="udp:localhost:14550", source_system=1)

        # Initialize MAVFTP instance for testing
        self.mav_ftp = MAVFTP(self.mock_master, target_system=1, target_component=1)

    def tearDown(self):
        # Release the UDP socket so the next test can re-bind the port.
        self.mock_master.close()
        self.logger.removeHandler(self.handler)
        self.logger.setLevel(self.log_level)
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
