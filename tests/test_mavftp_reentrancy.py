"""Adversarial synchronous transport and callback reentry regressions."""

import struct
from io import BytesIO
from unittest.mock import MagicMock, patch

import pytest

from pymavlink.mavftp import (
    FtpError,
    OP_Ack,
    OP_BurstReadFile,
    OP_CalcFileCRC32,
    OP_CreateDirectory,
    OP_CreateFile,
    OP_Nack,
    OP_OpenFileRO,
    OP_TerminateSession,
    OP_WriteFile,
)
from pymavlink.tests import test_mavftp as helpers
from pymavlink.tests.test_mavftp import ftp_reply


def test_retry_ack_does_not_resurrect_upload_inflight():
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.write_qsize = 1
    ftp.cmd_put(["old"], fh=BytesIO(b"x" * 160), callback=lambda _size: None)
    create = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(create.seq + 1, OP_Ack, OP_CreateFile, session=37))

    def send(packets):
        sent.extend(packets)
        request = master._decode_payload(packets[0])
        if request.opcode == OP_WriteFile and request.offset == 0:
            ftp.mavlink_packet(ftp_reply(
                request.seq + 1, OP_Ack, OP_WriteFile,
                offset=request.offset, session=37,
            ))

    ftp._send_payloads = send
    ftp.write_last_send = 0
    ftp.write_idx = 0
    ftp.idle_task()
    assert ftp.write_acks == 1
    assert ftp.write_inflight == {1}
    assert ftp.write_pending == 1
    assert ftp.pending_write_replies == {
        master._decode_payload(sent[-1]).seq + 1: 80,
    }
    assert completed == []


@pytest.mark.parametrize("cancel_only", [False, True])
def test_burst_retry_reentry_does_not_change_new_retry_counter(cancel_only):
    ftp, master, sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    request = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        request.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", 160), session=37,
    ))

    def send(packets):
        sent.extend(packets)
        if master._decode_payload(packets[0]).opcode == OP_BurstReadFile:
            ftp.cmd_cancel()
            if not cancel_only:
                ftp.cmd_mkdir(["next"], wait=False)
            ftp.read_retries = 7

    ftp._send_payloads = send
    ftp.last_burst_read = 0
    ftp.idle_task()
    assert ftp.read_retries == 7
    if not cancel_only:
        assert ftp.last_op.opcode == OP_CreateDirectory
        assert not ftp.event_complete


@pytest.mark.parametrize("raises", [False, True])
def test_standalone_download_callback_preserves_replacement(raises):
    ftp, _master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    new_callback = MagicMock()
    stream = BytesIO(b"data")
    ftp.fh = stream
    ftp.filename = "old"
    ftp.op_start = 1
    ftp.reached_eof = True
    ftp.remote_size_known = True
    ftp.requested_size = 4

    def callback(_stream):
        # Model releasing the old operation before starting another download.
        ftp.cmd_cancel()
        ftp.cmd_get(["next", "-"], callback=new_callback)
        if raises:
            raise RuntimeError("old callback failed after replacement")

    ftp.callback = callback
    with patch.object(ftp, "process_ftp_reply"):
        ftp._MAVFTP__check_read_finished()
    assert ftp.callback is new_callback
    assert ftp.last_op.opcode == OP_OpenFileRO
    assert ftp.filename == "-"
    assert ftp.callback_failure is None
    assert not ftp.read_complete
    new_callback.assert_not_called()


def test_standalone_final_progress_skips_old_completion_after_replacement():
    ftp, _master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    old_completion = MagicMock(side_effect=lambda _size: ftp.cmd_cancel())
    ftp.cmd_put(["old"], fh=BytesIO(b"data"), callback=old_completion)

    def progress(value):
        if value == 1:
            ftp.cmd_cancel()
            ftp.cmd_mkdir(["next"], wait=False)

    ftp.put_callback_progress = progress
    with patch.object(ftp, "process_ftp_reply"):
        ftp._MAVFTP__put_finished(4)
    old_completion.assert_not_called()
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert not ftp.request_cancelled


def test_nested_dispatch_restores_outer_rx_loss_scope():
    ftp, _master, _sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    observed = []

    def dispatch(message):
        if message == "outer":
            ftp._MAVFTP__dispatch_received_packet("inner")
            observed.append(ftp._rx_loss_applied)

    with patch.object(ftp, "_MAVFTP__mavlink_packet", side_effect=dispatch):
        ftp._MAVFTP__dispatch_received_packet("outer")
    assert observed == [True]
    assert not ftp._rx_loss_applied


@pytest.mark.parametrize("send_callback", [False, True])
def test_standalone_batch_stops_after_transport_replaces_upload(send_callback):
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    ftp.ftp_settings.write_qsize = 2
    original_send = master.mav.file_transfer_protocol_send
    replaced = []
    if send_callback:
        master.mav.file = helpers.BatchLink()
        master.mav.file_transfer_protocol_encode = MagicMock()
        master.mav.send_callback = MagicMock()

    def send(*args):
        original_send(*args)
        request = master._decode_payload(args[-1])
        if request.opcode == OP_WriteFile and not replaced:
            replaced.append(True)
            ftp.cmd_cancel()
            ftp.cmd_mkdir(["next"], wait=False)

    master.mav.file_transfer_protocol_send = send
    ftp.cmd_put(["old"], fh=BytesIO(b"x" * 160), callback=lambda _size: None)
    create = ftp.last_op
    with patch.object(ftp, "process_ftp_reply"):
        ftp.mavlink_packet(ftp_reply(
            create.seq + 1, OP_Ack, OP_CreateFile, session=ftp.session,
        ))
    assert replaced == [True]
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert [master._decode_payload(args[-1]).opcode
            for args in master.mav.sent].count(OP_WriteFile) == 1


def test_burst_retry_inline_progress_resets_retry_counter():
    ftp, master, sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    request = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        request.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", 160), session=37,
    ))

    def send(packets):
        sent.extend(packets)
        request = master._decode_payload(packets[0])
        if request.opcode == OP_BurstReadFile:
            ftp.mavlink_packet(ftp_reply(
                request.seq + 1, OP_Ack, OP_BurstReadFile,
                b"x" * 80, session=37,
            ))

    ftp._send_payloads = send
    ftp.last_burst_read = 0
    ftp.read_retries = 4
    ftp.idle_task()
    assert ftp.read_total == 80
    assert ftp.read_retries == 0


def start_fake_crccmp(ftp, names):
    """Exercise the public CRC batch starter without filesystem dependencies."""
    with (
        patch("pymavlink.mavftp.glob.glob", return_value=names),
        patch("pymavlink.mavftp.os.path.isfile", return_value=True),
        patch.object(ftp, "local_file_crc", return_value=123),
    ):
        return ftp.cmd_crccmp(["*.bin", "/remote"], wait=False)


@pytest.mark.parametrize("files", [1, 3])
def test_crccmp_accepts_inline_crc_replies(files):
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()

    def send(packets):
        sent.extend(packets)
        for packet in packets:
            request = master._decode_payload(packet)
            if request.opcode == OP_CalcFileCRC32:
                ftp.mavlink_packet(ftp_reply(
                    request.seq + 1, OP_Ack, OP_CalcFileCRC32,
                    struct.pack("<I", 123), session=37,
                ))

    ftp._send_payloads = send
    start_fake_crccmp(ftp, [f"{i}.bin" for i in range(files)])
    assert ftp.crccmp_results == ["MATCH"] * files
    assert [r.error_code for r in completed] == [FtpError.Success]
    assert ftp.crccmp_sent is None
    assert ftp.crccmp_expect is None
    assert ftp.crccmp_file_deadline is None


def test_crccmp_inline_backpressure_retry_reply_matches_fresh_sequence():
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    start_fake_crccmp(ftp, ["old.bin"])
    first = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        first.seq + 1, OP_Nack, OP_CalcFileCRC32,
        [FtpError.NoSessionsAvailable], session=37,
    ))

    def send(packets):
        sent.extend(packets)
        request = master._decode_payload(packets[0])
        if request.opcode == OP_CalcFileCRC32:
            ftp.mavlink_packet(ftp_reply(
                request.seq + 1, OP_Ack, OP_CalcFileCRC32,
                struct.pack("<I", 123), session=37,
            ))

    ftp._send_payloads = send
    ftp.last_op_time = 0
    ftp.idle_task()
    assert ftp.crccmp_results == ["MATCH"]
    assert [r.error_code for r in completed] == [FtpError.Success]
    assert ftp.crccmp_expect is None


def test_crccmp_replacement_batch_retains_its_published_timers():
    ftp, master, sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    replaced = []
    published = []

    def send(packets):
        sent.extend(packets)
        request = master._decode_payload(packets[0])
        if request.opcode == OP_CalcFileCRC32 and not replaced:
            replaced.append(True)
            ftp.cmd_cancel()
            start_fake_crccmp(ftp, ["new.bin"])
            ftp.crccmp_file_deadline = 987654321
            published.append((ftp.crccmp_sent, ftp.crccmp_expect,
                              ftp.crccmp_file_deadline))

    ftp._send_payloads = send
    start_fake_crccmp(ftp, ["old.bin"])
    assert ftp.crccmp_local == "new.bin"
    assert (ftp.crccmp_sent, ftp.crccmp_expect,
            ftp.crccmp_file_deadline) == published[0]


@pytest.mark.parametrize("replacement", ["get", "mkdir"])
def test_synchronous_reply_loop_stops_when_download_callback_replaces_operation(replacement):
    ftp, _master = helpers.TestMAVFTPReplyCompletion.make_ftp([
        ftp_reply(2, OP_Ack, OP_OpenFileRO, struct.pack("<I", 4)),
        ftp_reply(3, OP_Ack, OP_BurstReadFile, b"data", burst_complete=1),
        ftp_reply(4, OP_Ack, OP_TerminateSession),
    ])
    new_callback = MagicMock()

    def callback(_stream):
        ftp.cmd_cancel()
        if replacement == "get":
            ftp.cmd_get(["next", "-"], callback=new_callback)
        else:
            ftp.cmd_mkdir(["next"], wait=False)

    ftp.cmd_get(["old", "-"], callback=callback)
    ftp.process_ftp_reply("get", timeout=0.02)
    expected = OP_OpenFileRO if replacement == "get" else OP_CreateDirectory
    assert ftp.last_op.opcode == expected
    assert not ftp.request_cancelled
    assert not ftp.read_complete
    if replacement == "get":
        assert ftp.callback is new_callback
        new_callback.assert_not_called()


def test_synchronous_read_stops_when_transport_starts_replacement():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    new_callback = MagicMock()
    reentered = []

    def receive(**_kwargs):
        reentered.append(True)
        ftp.cmd_get(["next", "-"], callback=new_callback)

    master.recv_match = receive
    with patch("pymavlink.mavftp.READ_DEADLINE_SECONDS", 0.01):
        assert ftp.read("old", 4) is None
    assert len(reentered) == 1
    assert ftp.last_op.opcode == OP_OpenFileRO
    assert ftp.callback is new_callback
    assert not ftp.request_cancelled
    new_callback.assert_not_called()


def test_termination_reply_wait_cannot_retry_or_rotate_replacement_session():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    ftp.cmd_get(["old", "-"])
    new_callback = MagicMock()
    entered = []
    new_session = []

    def receive(**_kwargs):
        if not entered:
            entered.append(True)
            ftp.cmd_get(["next", "-"], callback=new_callback)
            new_session.append(ftp.session)

    master.recv_match = receive
    ftp.cmd_cancel()
    assert ftp.last_op.opcode == OP_OpenFileRO
    assert ftp.session == new_session[0]
    assert not ftp.request_cancelled
    assert ftp.callback is new_callback
    new_callback.assert_not_called()


def test_crccmp_inline_first_reply_preserves_next_pending_request():
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    answered = []

    def send(packets):
        sent.extend(packets)
        request = master._decode_payload(packets[0])
        if request.opcode == OP_CalcFileCRC32 and not answered:
            answered.append(True)
            ftp.mavlink_packet(ftp_reply(
                request.seq + 1, OP_Ack, OP_CalcFileCRC32,
                struct.pack("<I", 123), session=37,
            ))

    ftp._send_payloads = send
    start_fake_crccmp(ftp, ["first.bin", "second.bin"])
    pending = master._decode_payload(sent[-1])
    assert ftp.crccmp_local == "second.bin"
    assert ftp.crccmp_results == ["MATCH"]
    assert ftp.crccmp_expect == (37, pending.seq + 1)
    ftp.mavlink_packet(ftp_reply(
        pending.seq + 1, OP_Ack, OP_CalcFileCRC32,
        struct.pack("<I", 123), session=37,
    ))
    assert ftp.crccmp_results == ["MATCH", "MATCH"]
    assert [r.error_code for r in completed] == [FtpError.Success]
