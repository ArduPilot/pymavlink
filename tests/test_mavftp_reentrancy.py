"""Adversarial synchronous transport and callback reentry regressions."""

# These tests intentionally inspect internal protocol state and define small
# fixture classes without public APIs; both are part of the regression setup.
# pylint: disable=protected-access,missing-class-docstring,too-few-public-methods,too-many-lines

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
    OP_ListDirectory,
    OP_Nack,
    OP_OpenFileRO,
    OP_ReadFile,
    OP_TerminateSession,
    OP_WriteFile,
)
from pymavlink.tests import test_mavftp as helpers
from pymavlink.tests.test_mavftp import ftp_reply


@pytest.mark.parametrize("inline", [False, True])
def test_large_download_drains_inline_burst_continuations(inline):
    """Two hundred burst replies complete with inline and queued transports."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.burst_read_size = 80
    data = bytes(range(80)) * 199 + bytes(range(79))
    downloaded = []
    pending = []
    ftp.cmd_get(["large.bin", "-"], callback=lambda stream: downloaded.append(stream.read()))
    opened = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        opened.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", len(data)), session=37,
    ))
    first = master._decode_payload(sent[-1])

    def send(packets):
        sent.extend(packets)
        for payload in packets:
            request = master._decode_payload(payload)
            if request.opcode != OP_BurstReadFile:
                continue
            chunk = data[request.offset:request.offset + 80]
            reply = ftp_reply(
                request.seq + 1, OP_Ack, OP_BurstReadFile,
                chunk, offset=request.offset, burst_complete=1, session=37,
            )
            if inline:
                ftp.mavlink_packet(reply)
            else:
                pending.append(reply)

    ftp._send_payloads = send
    ftp.mavlink_packet(ftp_reply(
        first.seq + 1, OP_Ack, OP_BurstReadFile,
        data[:80], offset=0, burst_complete=1, session=37,
    ))
    while pending:
        ftp.mavlink_packet(pending.pop(0))
    assert downloaded == [data]
    assert [result.error_code for result in completed] == [FtpError.Success]
    assert sum(master._decode_payload(p).opcode == OP_BurstReadFile for p in sent) == 200


@pytest.mark.parametrize("inline", [False, True])
@pytest.mark.parametrize("max_backlog", [1, 5])
@pytest.mark.parametrize("gap_count", [8, 198])
def test_download_drains_inline_gap_refills(inline, max_backlog, gap_count):
    """Gap repairs complete without recursion or an idle-task recovery loop."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.burst_read_size = 80
    ftp.ftp_settings.max_backlog = max_backlog
    data = bytes(i % 251 for i in range(gap_count * 80 + 79))
    downloaded = []
    pending = []
    requests = []
    ftp.cmd_get(["gapped.bin", "-"], callback=lambda stream: downloaded.append(stream.read()))
    opened = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        opened.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", len(data)), session=37,
    ))
    burst = master._decode_payload(sent[-1])

    def send(packets):
        sent.extend(packets)
        for payload in packets:
            request = master._decode_payload(payload)
            if request.opcode != OP_ReadFile:
                continue
            requests.append((request.offset, request.size))
            assert 0 < ftp.backlog <= max_backlog
            reply = ftp_reply(
                request.seq + 1, OP_Ack, OP_ReadFile,
                data[request.offset:request.offset + request.size],
                offset=request.offset, session=37,
            )
            if inline:
                ftp.mavlink_packet(reply)
            else:
                pending.append(reply)

    ftp._send_payloads = send
    # Only the short final burst packet arrives; all preceding blocks are gaps.
    ftp.mavlink_packet(ftp_reply(
        burst.seq + 1, OP_Ack, OP_BurstReadFile,
        data[-79:], offset=gap_count * 80, burst_complete=1, session=37,
    ))
    if not inline:
        assert ftp.backlog == max_backlog
        assert len(pending) == max_backlog
    while pending:
        ftp.mavlink_packet(pending.pop(0))
    assert requests == [(i * 80, 80) for i in range(gap_count)]
    assert downloaded == [data]
    assert [result.error_code for result in completed] == [FtpError.Success]
    assert sum(master._decode_payload(p).opcode == OP_TerminateSession for p in sent) == 1
    assert ftp.backlog == 0
    assert not ftp.read_gaps
    assert not ftp.read_gap_times
    assert not ftp.read_gap_retries
    assert not ftp.pending_read_replies
    assert not ftp.pending_read_requests


@pytest.mark.parametrize("replacement", [False, True])
def test_gap_refill_drain_stops_on_cancel_and_allows_replacement(replacement):
    """An old drain cannot send stale gaps or block a new inline download."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.burst_read_size = 80
    old_data = []
    new_data = []
    old_reads = []
    new_reads = []
    replacing = []
    ftp.cmd_get(["old", "-"], callback=old_data.append)
    opened = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        opened.seq + 1, OP_Ack, OP_OpenFileRO, struct.pack("<I", 239), session=37,
    ))
    burst = master._decode_payload(sent[-1])
    data = bytes(range(239))

    def send(packets):
        sent.extend(packets)
        for payload in packets:
            request = master._decode_payload(payload)
            if request.opcode == OP_ReadFile:
                if not replacing:
                    old_reads.append(request.offset)
                    replacing.append(True)
                    # Queue work in the old drain before invalidating it.
                    ftp.check_read_send()
                    ftp.cmd_cancel()
                    if replacement:
                        ftp.cmd_get(["next", "-"],
                                    callback=lambda stream: new_data.append(stream.read()))
                    continue
                new_reads.append(request.offset)
                reply = ftp_reply(
                    request.seq + 1, OP_Ack, OP_ReadFile,
                    data[request.offset:request.offset + request.size],
                    offset=request.offset, session=37,
                )
            elif request.opcode == OP_OpenFileRO:
                reply = ftp_reply(
                    request.seq + 1, OP_Ack, OP_OpenFileRO,
                    struct.pack("<I", len(data)), session=37,
                )
            elif request.opcode == OP_BurstReadFile:
                reply = ftp_reply(
                    request.seq + 1, OP_Ack, OP_BurstReadFile,
                    data[160:], offset=160, burst_complete=1, session=37,
                )
            else:
                continue
            ftp.mavlink_packet(reply)

    ftp._send_payloads = send
    ftp.mavlink_packet(ftp_reply(
        burst.seq + 1, OP_Ack, OP_BurstReadFile,
        b"x" * 79, offset=160, burst_complete=1, session=37,
    ))
    assert old_reads == [0]
    assert old_data == [None]
    assert new_reads == ([0, 80] if replacement else [])
    assert new_data == ([data] if replacement else [])
    assert [result.error_code for result in completed] == (
        [FtpError.Fail, FtpError.Success] if replacement else [FtpError.Fail]
    )
    assert not ftp._gap_read_drains
    assert not ftp.pending_read_requests
    assert not ftp.pending_read_replies
    assert ftp.backlog == 0


def test_gap_refill_drain_releases_ownership_after_transport_exception():
    """A failed send leaves the gap retryable and does not strand its drain."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.burst_read_size = 80
    ftp.ftp_settings.max_backlog = 1
    downloaded = []
    reads = []
    clock = [1.0]
    data = bytes(range(239))
    ftp.cmd_get(["gapped.bin", "-"], callback=lambda stream: downloaded.append(stream.read()))
    opened = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        opened.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", len(data)), session=37,
    ))
    burst = master._decode_payload(sent[-1])

    def send(packets):
        sent.extend(packets)
        for payload in packets:
            request = master._decode_payload(payload)
            if request.opcode != OP_ReadFile:
                continue
            reads.append((request.seq, request.offset))
            if len(reads) == 1:
                ftp.check_read_send()
                raise RuntimeError("transport failed")
            ftp.mavlink_packet(ftp_reply(
                request.seq + 1, OP_Ack, OP_ReadFile,
                data[request.offset:request.offset + request.size],
                offset=request.offset, session=37,
            ))

    ftp._send_payloads = send
    with patch("pymavlink.mavftp.time.time", side_effect=lambda: clock[0]):
        with pytest.raises(RuntimeError, match="transport failed"):
            ftp.mavlink_packet(ftp_reply(
                burst.seq + 1, OP_Ack, OP_BurstReadFile,
                data[160:], offset=160, burst_complete=1, session=37,
            ))
        assert not ftp._gap_read_drains
        assert ftp.backlog == 1
        clock[0] += ftp.retry_timeout() + 1
        ftp.check_read_send()
    assert reads[0] == reads[1]
    assert [offset for _seq, offset in reads] == [0, 0, 80]
    assert downloaded == [data]
    assert [result.error_code for result in completed] == [FtpError.Success]
    assert not ftp._gap_read_drains
    assert not ftp.pending_read_requests
    assert not ftp.pending_read_replies
    assert ftp.backlog == 0


@pytest.mark.parametrize("inline", [False, True])
def test_large_listing_drains_inline_page_continuations(inline):
    """Two hundred listing pages complete with inline and queued transports."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    pending = []
    ftp.cmd_list([], wait=False)
    first = master._decode_payload(sent[-1])

    def send(packets):
        sent.extend(packets)
        for payload in packets:
            request = master._decode_payload(payload)
            if request.opcode != OP_ListDirectory:
                continue
            if request.offset == 200:
                reply = ftp_reply(
                    request.seq + 1, OP_Nack, OP_ListDirectory,
                    [FtpError.EndOfFile], session=37,
                )
            else:
                entry = f"Ffile{request.offset}\t{request.offset}\0".encode()
                reply = ftp_reply(
                    request.seq + 1, OP_Ack, OP_ListDirectory,
                    entry, session=37,
                )
            if inline:
                ftp.mavlink_packet(reply)
            else:
                pending.append(reply)

    ftp._send_payloads = send
    ftp.mavlink_packet(ftp_reply(
        first.seq + 1, OP_Ack, OP_ListDirectory, b"Ffile0\t0\0", session=37,
    ))
    while pending:
        ftp.mavlink_packet(pending.pop(0))
    assert [entry.name for entry in ftp.list_result] == [f"file{i}" for i in range(200)]
    assert [result.error_code for result in completed] == [FtpError.Success]
    assert sum(master._decode_payload(p).opcode == OP_ListDirectory for p in sent) == 201


@pytest.mark.parametrize("inline", [False, True])
def test_large_upload_drains_inline_acknowledgements_without_recursion(inline):
    """Inline and queued transports both complete the same 200-block upload."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    pending = []
    uploaded = {}
    sizes = []

    def send(packets):
        sent.extend(packets)
        for payload in packets:
            request = master._decode_payload(payload)
            if request.opcode == OP_WriteFile:
                uploaded[request.offset] = bytes(request.payload)
                reply = ftp_reply(
                    request.seq + 1, OP_Ack, OP_WriteFile,
                    offset=request.offset, session=37,
                )
                if inline:
                    ftp.mavlink_packet(reply)
                else:
                    pending.append(reply)

    ftp._send_payloads = send
    data = bytes(range(80)) * 200
    ftp.cmd_put(["large.bin"], fh=BytesIO(data), callback=sizes.append)
    create = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(create.seq + 1, OP_Ack, OP_CreateFile, session=37))
    while pending:
        ftp.mavlink_packet(pending.pop(0))
    assert b"".join(uploaded[offset] for offset in sorted(uploaded)) == data
    assert sizes == [len(data)]
    assert [result.error_code for result in completed] == [FtpError.Success]
    assert sum(master._decode_payload(p).opcode == OP_TerminateSession for p in sent) == 1


def test_cancel_upload_inline_write_ack_sends_only_one_termination():
    """An ACK arriving during cancellation cannot terminate detached state again."""
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    sizes = []
    ftp.cmd_put(["old"], fh=BytesIO(b"x" * 160), callback=sizes.append)
    create = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(create.seq + 1, OP_Ack, OP_CreateFile, session=37))
    write = master._decode_payload(sent[-1])
    answered = []

    def send(packets):
        sent.extend(packets)
        if master._decode_payload(packets[0]).opcode == OP_TerminateSession and not answered:
            answered.append(True)
            ftp.mavlink_packet(ftp_reply(
                write.seq + 1, OP_Ack, OP_WriteFile, offset=write.offset, session=37,
            ))

    ftp._send_payloads = send
    ftp.cmd_cancel()
    assert answered == [True]
    assert sum(master._decode_payload(p).opcode == OP_TerminateSession for p in sent) == 1
    assert sizes == [None]
    assert len(completed) == 1
    assert ftp.write_list is None
    assert not ftp.write_open
    assert not ftp.pending_write_requests
    assert not ftp.pending_write_replies
    assert not ftp.write_inflight
    assert ftp.write_pending == 0


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
    assert not completed


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
    ftp.cmd_put(["old"], fh=BytesIO(b""), callback=old_completion)

    def progress(value):
        if value == 1:
            ftp.cmd_cancel()
            ftp.cmd_mkdir(["next"], wait=False)

    ftp.put_callback_progress = progress
    with patch.object(ftp, "process_ftp_reply"):
        create = ftp.last_op
        ftp.mavlink_packet(ftp_reply(
            create.seq + 1, OP_Ack, OP_CreateFile, session=ftp.session,
        ))
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


@pytest.mark.parametrize("files", [1, 3, 200])
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


@pytest.mark.parametrize("managed", [False, True])
@pytest.mark.parametrize("files", [1, 3])
def test_crccmp_cancel_during_send_terminates_once(managed, files):
    if managed:
        ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    else:
        ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
        sent = []
        completed = []
    initial_session = ftp.session

    def send(packets):
        sent.extend(packets)
        request = master._decode_payload(packets[0])
        if request.opcode == OP_CalcFileCRC32:
            ftp.cmd_cancel()

    if managed:
        ftp._send_payloads = send
    else:
        master.mav.file_transfer_protocol_send = lambda *args: send([args[-1]])

    with patch.object(ftp, "process_ftp_reply", return_value=helpers.MAVFTPReturn(
        "TerminateSession", FtpError.Success,
    )) as reply_loop:
        start_fake_crccmp(ftp, [f"{i}.bin" for i in range(files)])

    requests = [master._decode_payload(packet) for packet in sent]
    assert [request.opcode for request in requests] == [
        OP_CalcFileCRC32, OP_TerminateSession,
    ]
    assert [request.session for request in requests] == [initial_session] * 2
    assert ftp.session == (initial_session if managed else (initial_session + 1) % 256)
    assert reply_loop.call_count == (0 if managed else 1)
    if managed:
        assert [(result.operation_name, result.error_code) for result in completed] == [
            ("CRCCompare", FtpError.Fail),
        ]
    assert ftp.crccmp_destination is None
    assert ftp.crccmp_pending == []
    assert ftp.crccmp_expect is None
    assert ftp.request_cancelled
    assert ftp._crccmp_advancing_generation is None


@pytest.mark.parametrize("command,args", [
    ("cmd_list", ["/old"]),
    ("cmd_rm", ["old"]),
    ("cmd_rmdir", ["old"]),
    ("cmd_rename", ["old", "renamed"]),
    ("cmd_mkdir", ["old"]),
    ("cmd_crc", ["old"]),
])
def test_blocking_command_send_replacement_never_waits_on_new_operation(command, args):
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    original_send = master.mav.file_transfer_protocol_send
    entered = []
    new_callback = MagicMock()

    def send(*send_args):
        original_send(*send_args)
        if not entered:
            entered.append(True)
            ftp.cmd_get(["next", "-"], callback=new_callback)

    master.mav.file_transfer_protocol_send = send
    with patch.object(ftp, "process_ftp_reply") as reply_loop:
        result = getattr(ftp, command)(args, timeout=0.01)
    reply_loop.assert_not_called()
    assert result.error_code == FtpError.Fail
    assert ftp.last_op.opcode == OP_OpenFileRO
    assert ftp.callback is new_callback
    assert not ftp.request_cancelled


def test_nested_blocking_command_invalidates_outer_reply_loop():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    received = []

    def receive(**_kwargs):
        received.append(True)
        # Avoid running a real nested receive loop; the command itself must
        # publish ownership independently of its reply-loop implementation.
        with patch.object(ftp, "process_ftp_reply"):
            ftp.cmd_mkdir(["next"])

    master.recv_match = receive
    ftp.process_ftp_reply("get", timeout=0.01)
    assert received == [True]
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert not ftp.request_cancelled


@pytest.mark.parametrize("managed,consumer", [
    (True, False), (False, False), (False, True),
])
def test_download_cleanup_transport_replacement_does_not_set_read_complete(managed, consumer):
    if managed:
        ftp, master, sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    else:
        ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
        sent = []
    ftp.fh = BytesIO(b"data")
    ftp.fh.seek(4)
    ftp.filename = "-"
    ftp.op_start = 1
    ftp.reached_eof = True
    ftp.requested_size = 4
    if consumer:
        ftp.callback = lambda _stream: None
    replaced = []

    def replace_on_termination(payload):
        request = master._decode_payload(payload)
        if request.opcode == OP_TerminateSession and not replaced:
            replaced.append(True)
            ftp.cmd_mkdir(["next"], wait=False)

    if managed:
        def send(packets):
            sent.extend(packets)
            replace_on_termination(packets[0])
        ftp._send_payloads = send
    else:
        original_send = master.mav.file_transfer_protocol_send

        def send(*args):
            original_send(*args)
            replace_on_termination(args[-1])
        master.mav.file_transfer_protocol_send = send
    with patch("pymavlink.mavftp.sys.stdout", helpers.BinaryStdout()):
        ftp._MAVFTP__check_read_finished()
    assert replaced == [True]
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert not ftp.read_complete
    assert not ftp.request_cancelled


def test_upload_cleanup_transport_replacement_does_not_publish_old_completed_reply():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    ftp.cmd_put(["old"], fh=BytesIO(b""), callback=lambda _size: None)
    original_send = master.mav.file_transfer_protocol_send
    entered = []

    def send(*args):
        original_send(*args)
        request = master._decode_payload(args[-1])
        if request.opcode == OP_TerminateSession and not entered:
            entered.append(True)
            ftp.cmd_mkdir(["next"], wait=False)

    master.mav.file_transfer_protocol_send = send
    create = ftp.last_op
    ftp.mavlink_packet(ftp_reply(
        create.seq + 1, OP_Ack, OP_CreateFile, session=ftp.session,
    ))
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert ftp.completed_reply is None
    assert not ftp.request_cancelled


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


def test_termination_send_replacement_download_does_not_keep_old_staging_stream():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    old_stream = BytesIO(b"old")
    ftp.cmd_get(["old", "-"])
    ftp.fh = old_stream
    ftp.fh_owned = True
    ftp.temp_filename = "old-stage"
    next_callback = MagicMock()
    original_send = master.mav.file_transfer_protocol_send
    replaced = []

    def send(*args):
        original_send(*args)
        request = master._decode_payload(args[-1])
        if request.opcode == OP_TerminateSession and not replaced:
            replaced.append(True)
            ftp.cmd_get(["next", "-"], callback=next_callback)

    master.mav.file_transfer_protocol_send = send
    with patch("pymavlink.mavftp.os.unlink") as unlink:
        ftp.cmd_cancel()

        request = ftp.last_op
        assert request.opcode == OP_OpenFileRO
        ftp.mavlink_packet(ftp_reply(
            request.seq + 1,
            OP_Ack,
            OP_OpenFileRO,
            struct.pack("<I", 4),
            session=request.session,
        ))

    assert replaced == [True]
    assert ftp.last_op.opcode == OP_BurstReadFile
    assert ftp.fh is not old_stream
    assert old_stream.closed
    assert ftp.temp_filename is None
    assert ftp.callback is next_callback
    next_callback.assert_not_called()
    unlink.assert_called_once_with("old-stage")


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


def test_standalone_download_callback_idle_reentry_cannot_finalize_twice():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    ftp.fh = BytesIO(b"data")
    ftp.fh.seek(4)
    ftp.filename = "-"
    ftp.op_start = 1
    ftp.reached_eof = True
    ftp.requested_size = 4
    observations = []

    def consume(stream):
        ftp._MAVFTP__check_read_finished()
        observations.append((ftp.fh is stream, ftp.read_complete))
        assert stream.read() == b"data"

    ftp.callback = consume
    with (
        patch.object(ftp, "process_ftp_reply", return_value=helpers.MAVFTPReturn(
            "TerminateSession", FtpError.Success,
        )),
        patch("pymavlink.mavftp.sys.stdout", helpers.BinaryStdout()) as stdout,
    ):
        ftp._MAVFTP__check_read_finished()
    assert observations == [(True, False)]
    assert stdout.buffer.getvalue() == b""
    requests = [master._decode_payload(args[-1]) for args in master.mav.sent]
    assert sum(r.opcode == OP_TerminateSession for r in requests) == 1


def test_standalone_upload_final_progress_idle_reentry_cannot_finalize_twice():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    completed = []
    progress = []

    def on_progress(value):
        progress.append(value)
        if value == 1:
            ftp.idle_task()

    ftp.cmd_put(["old"], fh=BytesIO(b""), callback=completed.append,
                progress_callback=on_progress)
    create = ftp.last_op
    with patch.object(ftp, "process_ftp_reply", return_value=helpers.MAVFTPReturn(
        "TerminateSession", FtpError.Success,
    )):
        ftp.mavlink_packet(ftp_reply(
            create.seq + 1, OP_Ack, OP_CreateFile, session=ftp.session,
        ))
    assert progress == [1.0]
    assert completed == [0]
    requests = [master._decode_payload(args[-1]) for args in master.mav.sent]
    assert sum(r.opcode == OP_TerminateSession for r in requests) == 1


@pytest.mark.parametrize("raises", [False, True])
def test_upload_source_read_replacement_cannot_send_or_fail_new_operation(raises):
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    entered = []

    class ReentrantSource(BytesIO):
        def read(self, size=-1):
            data = super().read(size)
            if not entered:
                entered.append(True)
                ftp.cmd_cancel()
                ftp.cmd_mkdir(["next"], wait=False)
                if raises:
                    raise OSError("old source failed after replacement")
            return data

    ftp.cmd_put(["old"], fh=ReentrantSource(b"data"), callback=lambda _size: None)
    create = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        create.seq + 1, OP_Ack, OP_CreateFile, session=37,
    ))
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert not ftp.request_cancelled
    assert ftp.callback_failure is None
    assert ftp.write_inflight == set()
    assert [r.operation_name for r in completed] == ["Put"]
    assert all(master._decode_payload(packet).opcode != OP_WriteFile for packet in sent)


def test_upload_source_read_idle_reentry_cannot_reserve_block_twice():
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    entered = []

    class ReentrantSource(BytesIO):
        def read(self, size=-1):
            data = super().read(size)
            if not entered:
                entered.append(True)
                ftp.idle_task()
            return data

    ftp.cmd_put(["old"], fh=ReentrantSource(b"data"), callback=lambda _size: None)
    create = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        create.seq + 1, OP_Ack, OP_CreateFile, session=37,
    ))
    writes = [master._decode_payload(packet) for packet in sent
              if master._decode_payload(packet).opcode == OP_WriteFile]
    assert len(writes) == 1
    assert ftp.write_pending == 1
    assert ftp.write_inflight == {0}
    assert not completed


def test_standalone_upload_termination_idle_reentry_reports_completion_once():
    ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    completed = []
    entered = []
    original_send = master.mav.file_transfer_protocol_send

    def send(*args):
        original_send(*args)
        request = master._decode_payload(args[-1])
        if request.opcode == OP_TerminateSession and not entered:
            entered.append(True)
            ftp.idle_task()

    master.mav.file_transfer_protocol_send = send
    ftp.cmd_put(["old"], fh=BytesIO(b""), callback=completed.append)
    create = ftp.last_op
    with patch.object(ftp, "process_ftp_reply", return_value=helpers.MAVFTPReturn(
        "TerminateSession", FtpError.Success,
    )):
        ftp.mavlink_packet(ftp_reply(
            create.seq + 1, OP_Ack, OP_CreateFile, session=ftp.session,
        ))
    assert completed == [0]
    requests = [master._decode_payload(args[-1]) for args in master.mav.sent]
    assert sum(r.opcode == OP_TerminateSession for r in requests) == 1
    assert not ftp._put_finalizing_generations


def test_upload_source_read_guard_allows_replacement_upload_to_send_inline():
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    entered = []

    class ReentrantSource(BytesIO):
        def read(self, size=-1):
            data = super().read(size)
            if not entered:
                entered.append(True)
                ftp.cmd_cancel()
                ftp.cmd_put(["next"], fh=BytesIO(b"new"), callback=lambda _size: None)
                create = master._decode_payload(sent[-1])
                ftp.mavlink_packet(ftp_reply(
                    create.seq + 1, OP_Ack, OP_CreateFile, session=37,
                ))
            return data

    ftp.cmd_put(["old"], fh=ReentrantSource(b"old"), callback=lambda _size: None)
    create = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        create.seq + 1, OP_Ack, OP_CreateFile, session=37,
    ))
    writes = [master._decode_payload(packet) for packet in sent
              if master._decode_payload(packet).opcode == OP_WriteFile]
    assert [bytes(write.payload) for write in writes] == [b"new"]
    assert ftp.write_pending == 1
    assert [r.operation_name for r in completed] == ["Put"]
    assert not ftp._upload_read_generations


@pytest.mark.parametrize("replacement", [False, True])
def test_partial_link_write_reentry_stops_old_packet_tail(replacement):
    ftp, _master, _sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_put(["old"], fh=BytesIO(b"data"), callback=lambda _size: None)
    writes = []

    class PartialLink:
        def write(self, data):
            writes.append(data)
            if len(writes) == 1:
                ftp.cmd_cancel()
                if replacement:
                    ftp.cmd_mkdir(["next"], wait=False)
            return 2

    ftp._MAVFTP__write_link_data(PartialLink(), b"old-packet", False)
    assert writes == [b"old-packet"]
    if replacement:
        assert ftp.last_op.opcode == OP_CreateDirectory
        assert not ftp.request_cancelled


@pytest.mark.parametrize("phase", ["seek", "write", "write_error"])
def test_download_destination_io_replacement_preserves_new_operation(phase):
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    entered = []
    new_stream = BytesIO(b"new")
    old_writes = []

    def replace():
        if not entered:
            entered.append(True)
            ftp.cmd_cancel()
            ftp.cmd_get(["next", "-"], callback=lambda _stream: None)
            ftp.fh = new_stream

    class Destination(BytesIO):
        def seek(self, offset, whence=0):
            result = super().seek(offset, whence)
            if phase == "seek":
                replace()
            return result

        def write(self, data):
            old_writes.append(bytes(data))
            replace()
            if phase == "write_error":
                raise OSError("old staging write failed")
            return super().write(data)

    ftp.fh = Destination()
    op = helpers.FTP_OP(0, 37, OP_Ack, 4, OP_BurstReadFile, 0, 0, bytearray(b"old!"))
    assert not ftp._MAVFTP__write_payload(op)
    assert ftp.fh is new_stream
    assert new_stream.getvalue() == b"new"
    assert ftp.read_total == 0
    assert ftp.callback_failure is None
    assert not ftp.request_cancelled
    assert [r.operation_name for r in completed] == ["Get"]
    if phase == "seek":
        assert not old_writes


def test_release_staging_close_reentry_preserves_replacement_resources():
    ftp, _master, _sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    replacement = BytesIO(b"new")

    class ClosingStream(BytesIO):
        def close(self):
            ftp.fh = replacement
            ftp.fh_owned = True
            ftp.temp_filename = "replacement-stage"
            super().close()

    ftp.fh = ClosingStream(b"old")
    ftp.fh_owned = True
    ftp.temp_filename = "old-stage"
    with patch("pymavlink.mavftp.os.unlink") as unlink:
        ftp._MAVFTP__release_staging()
    assert ftp.fh is replacement
    assert ftp.fh_owned
    assert ftp.temp_filename == "replacement-stage"
    unlink.assert_called_once_with("old-stage")


def test_termination_close_reentry_preserves_new_command_and_reports_old_result():
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    replacement = BytesIO(b"new")
    next_callback = MagicMock()

    class ClosingStream(BytesIO):
        def close(self):
            ftp.cmd_get(["next", "-"], callback=next_callback)
            ftp.fh = replacement
            ftp.fh_owned = True
            super().close()

    ftp.fh = ClosingStream(b"old")
    ftp.fh_owned = True
    ftp.cmd_cancel()
    assert ftp.fh is replacement
    assert ftp.fh_owned
    assert ftp.callback is next_callback
    assert not ftp.event_complete
    assert not ftp.request_cancelled
    assert [r.operation_name for r in completed] == ["Get"]
    next_callback.assert_not_called()


@pytest.mark.parametrize("phase", ["flush", "seek", "read", "read_error"])
def test_download_publication_io_replacement_preserves_new_download(phase):
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"])
    replacement = BytesIO(b"new")
    next_callback = MagicMock()
    entered = []

    def replace():
        if not entered:
            entered.append(True)
            ftp.cmd_cancel()
            ftp.cmd_get(["next", "-"], callback=next_callback)
            ftp.fh = replacement

    class PublicationStream(BytesIO):
        def flush(self):
            super().flush()
            if phase == "flush":
                replace()

        def seek(self, offset, whence=0):
            result = super().seek(offset, whence)
            if phase == "seek":
                replace()
            return result

        def read(self, size=-1):
            result = super().read(size)
            if phase in {"read", "read_error"}:
                replace()
                if phase == "read_error":
                    raise OSError("old publication failed after replacement")
            return result

    stream = PublicationStream(b"old")
    BytesIO.seek(stream, 3)
    ftp.fh = stream
    ftp.reached_eof = True
    ftp.requested_size = 3
    with patch("pymavlink.mavftp.sys.stdout", helpers.BinaryStdout()) as stdout:
        ftp._MAVFTP__check_read_finished()
    assert entered == [True]
    assert ftp.fh is replacement
    assert replacement.getvalue() == b"new"
    assert ftp.callback is next_callback
    assert ftp.get_result is None
    assert ftp.callback_failure is None
    assert not ftp.read_complete
    assert not ftp.event_complete
    assert not ftp.request_cancelled
    assert stdout.buffer.getvalue() == b""
    assert [r.operation_name for r in completed] == ["Get"]
    next_callback.assert_not_called()


@pytest.mark.parametrize("raises", [False, True])
def test_stdout_publication_replacement_cannot_fail_or_terminate_new_command(raises):
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"])
    ftp.fh = BytesIO(b"old")
    ftp.fh.seek(3)
    ftp.reached_eof = True
    ftp.requested_size = 3

    class Output:
        def write(self, data):
            ftp.cmd_cancel()
            ftp.cmd_mkdir(["next"], wait=False)
            if raises:
                raise OSError("old output failed after replacement")
            return len(data)

    stdout = MagicMock()
    stdout.buffer = Output()
    with patch("pymavlink.mavftp.sys.stdout", stdout):
        ftp._MAVFTP__check_read_finished()
    assert ftp.last_op.opcode == OP_CreateDirectory
    assert not ftp.event_complete
    assert not ftp.request_cancelled
    assert ftp.callback_failure is None
    assert not ftp.read_complete
    assert [r.operation_name for r in completed] == ["Get"]
    stdout.flush.assert_not_called()


def test_standalone_consumer_seek_replacement_skips_old_data_callback():
    ftp, _master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
    old_callback = MagicMock()
    next_callback = MagicMock()

    class SeekingStream(BytesIO):
        def seek(self, offset, whence=0):
            result = super().seek(offset, whence)
            ftp.cmd_get(["next", "-"], callback=next_callback)
            return result

    stream = SeekingStream(b"old")
    BytesIO.seek(stream, 3)
    ftp.cmd_get(["old", "-"], callback=old_callback)
    ftp.fh = stream
    ftp.reached_eof = True
    ftp.requested_size = 3
    ftp._MAVFTP__check_read_finished()
    old_callback.assert_not_called()
    assert ftp.callback is next_callback
    assert not ftp.request_cancelled


@pytest.mark.parametrize("phase", ["fsync", "close", "replace"])
def test_staged_publication_replacement_preserves_new_staging_ownership(phase):
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "old-destination"])
    new_stream = BytesIO(b"new")
    entered = []

    def replace_operation(*_args):
        if not entered:
            entered.append(True)
            ftp.cmd_get(["next", "new-destination"])
            ftp.fh = new_stream
            ftp.fh_owned = True
            ftp.temp_filename = "new-stage"

    class StagingStream(BytesIO):
        def fileno(self):
            return 123

        def close(self):
            if phase == "close":
                replace_operation()
            super().close()

    stream = StagingStream(b"old")
    stream.seek(3)
    ftp.fh = stream
    ftp.fh_owned = True
    ftp.temp_filename = "old-stage"
    ftp.reached_eof = True
    ftp.requested_size = 3
    with (
        patch("pymavlink.mavftp.os.fstat", return_value=MagicMock(st_size=3)),
        patch("pymavlink.mavftp.os.fsync",
              side_effect=replace_operation if phase == "fsync" else None),
        patch("pymavlink.mavftp.os.replace",
              side_effect=replace_operation if phase == "replace" else None) as publish,
        patch("pymavlink.mavftp.os.unlink") as unlink,
        patch.object(ftp, "_MAVFTP__fsync_directory"),
    ):
        ftp._MAVFTP__check_read_finished()
    assert entered == [True]
    assert ftp.fh is new_stream
    assert ftp.fh_owned
    assert ftp.temp_filename == "new-stage"
    assert ftp.filename == "new-destination"
    assert not ftp.read_complete
    assert not ftp.request_cancelled
    assert not completed
    unlink.assert_not_called()
    if phase != "replace":
        publish.assert_not_called()
    else:
        publish.assert_called_once_with("old-stage", "old-destination")


@pytest.mark.parametrize("repair_opcode", [OP_BurstReadFile, helpers.OP_ReadFile])
def test_gap_progress_nested_burst_preserves_receive_high_water_mark(repair_opcode):
    ftp, master, sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    request = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        request.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", 1000), session=37,
    ))
    burst = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        burst.seq + 1, OP_Ack, OP_BurstReadFile,
        b"x" * 80, offset=80, session=37,
    ))
    entered = []

    def progress(_value):
        if not entered:
            entered.append(True)
            ftp.mavlink_packet(ftp_reply(
                burst.seq + 2, OP_Ack, OP_BurstReadFile,
                b"y" * 80, offset=160, session=37,
            ))

    ftp.callback_progress = progress
    if repair_opcode == helpers.OP_ReadFile:
        ftp.check_read_send()
        repair = master._decode_payload(sent[-1])
    else:
        repair = burst
    ftp.mavlink_packet(ftp_reply(
        repair.seq + 1, OP_Ack, repair_opcode,
        b"z" * 80, offset=0, session=37,
    ))
    assert ftp.fh.tell() == 240
    assert not ftp.read_gaps
    ftp.mavlink_packet(ftp_reply(
        burst.seq + 3, OP_Ack, OP_BurstReadFile,
        b"w" * 80, offset=240, session=37,
    ))
    assert not ftp.read_gaps
    assert ftp.fh.tell() == 320


def test_progress_nested_burst_continuation_cannot_rewind_pending_request():
    ftp, master, sent, _completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["old", "-"], callback=lambda _stream: None)
    request = master._decode_payload(sent[-1])
    ftp.mavlink_packet(ftp_reply(
        request.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", 1000), session=37,
    ))
    burst = master._decode_payload(sent[-1])
    entered = []

    def progress(_value):
        if not entered:
            entered.append(True)
            ftp.mavlink_packet(ftp_reply(
                burst.seq + 2, OP_Ack, OP_BurstReadFile,
                b"y" * 80, offset=80, burst_complete=1, session=37,
            ))

    ftp.callback_progress = progress
    ftp.mavlink_packet(ftp_reply(
        burst.seq + 1, OP_Ack, OP_BurstReadFile,
        b"x" * 80, burst_complete=1, session=37,
    ))
    assert ftp.pending_burst_offset == 160
    assert master._decode_payload(sent[-1]).offset == 160
