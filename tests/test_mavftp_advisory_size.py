"""EOF and contiguous received data, not stat estimates, complete downloads."""

# Regression tests intentionally inspect transfer accounting and use the
# established in-memory MAVLink transport from test_mavftp.
# pylint: disable=protected-access

import struct
from pathlib import Path
from unittest.mock import patch

import pytest

from pymavlink.mavftp import (
    FtpError,
    OP_Ack,
    OP_BurstReadFile,
    OP_Nack,
    OP_OpenFileRO,
    OP_ReadFile,
    OP_TerminateSession,
)
from pymavlink.tests import test_mavftp as helpers
from pymavlink.tests.test_mavftp import ftp_reply


@pytest.mark.parametrize("managed", [False, True])
@pytest.mark.parametrize("explicit_eof", [False, True])
@pytest.mark.parametrize(
    "remote,advertised_size,actual_size",
    [
        ("@PARAM/param.pck", 17544, 16356),
        ("@PARAM/param.pck?withdefaults=1", 17544, 16356),
        ("@SYS/tasks.txt", 100000, 3000),
        ("remote.bin", 17544, 16356),
        ("@SYS/storage.bin", 16384, 3000),
        ("@SYS/crash_dump.bin", 16384, 3000),
        ("empty.bin", 100000, 0),
        ("underestimated.bin", 4, 3000),
    ],
)
def test_staged_download_uses_eof_not_advertised_size(  # pylint: disable=too-many-arguments,too-many-locals
    tmp_path, managed, explicit_eof, remote, advertised_size, actual_size
):
    """Both transports atomically publish a complete stream below an estimate."""
    data = bytes(i % 251 for i in range(actual_size))
    destination = tmp_path / "download.bin"
    destination.write_bytes(b"original")
    if managed:
        ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    else:
        ftp, master = helpers.TestMAVFTPReplyCompletion.make_ftp([])
        sent = completed = None
    ftp.ftp_settings.burst_read_size = 80
    ftp.cmd_get([remote, str(destination)])
    ftp.mavlink_packet(ftp_reply(
        ftp.last_op.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", advertised_size), session=37,
    ))
    stage = ftp.temp_filename
    first_seq = ftp.pending_burst_seq
    assert destination.read_bytes() == b"original"
    assert stage is not None
    # Keep all data in one burst. The final short ACK, or the subsequent EOF
    # NACK at the received high-water mark, proves completion.
    with patch("pymavlink.mavftp.logging.warning") as warning:
        for index, offset in enumerate(range(0, len(data), 80)):
            final = offset + 80 >= len(data)
            ftp.mavlink_packet(ftp_reply(
                first_seq + index, OP_Ack, OP_BurstReadFile,
                data[offset:offset + 80], offset=offset,
                burst_complete=int(final and not explicit_eof), session=37,
            ))
            if explicit_eof or not final:
                assert destination.read_bytes() == b"original"
                assert not ftp.read_complete
        if explicit_eof or not data:
            ftp.mavlink_packet(ftp_reply(
                first_seq + (len(data) + 79) // 80,
                OP_Nack, OP_BurstReadFile,
                bytes([FtpError.EndOfFile]), offset=len(data), session=37,
            ))
    assert destination.read_bytes() == data
    assert ftp.callback_failure is None
    assert ftp.read_complete
    assert not ftp.transfer_active
    assert ftp.fh is None
    assert ftp.temp_filename is None
    assert not Path(stage).exists()
    assert ftp.get_result is None
    assert ftp.requested_size == len(data)
    warning.assert_not_called()
    requests = (
        [master._decode_payload(payload) for payload in sent]
        if managed else helpers.TestMAVFTPReplyCompletion.sent_requests(master)
    )
    terminations = [request for request in requests if request.opcode == OP_TerminateSession]
    assert terminations
    assert {request.session for request in terminations} == {37}
    if managed:
        assert len(terminations) == 1
        assert ftp.event_complete
        assert [result.error_code for result in completed] == [FtpError.Success]


@pytest.mark.parametrize("remote", ["@PARAM/param.pck", "@SYS/tasks.txt"])
def test_estimated_size_download_waits_for_post_eof_gap_repair(
    tmp_path, remote
):
    """An EOF packet cannot publish zeros where an earlier block was lost."""
    destination = tmp_path / "download.bin"
    destination.write_bytes(b"original")
    ftp, master, sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.burst_read_size = 80
    ftp.cmd_get([remote, str(destination)])
    ftp.mavlink_packet(ftp_reply(
        ftp.last_op.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", 100000), session=37,
    ))
    burst_seq = ftp.pending_burst_seq
    ftp.mavlink_packet(ftp_reply(
        burst_seq + 1, OP_Ack, OP_BurstReadFile,
        b"tail", offset=80, burst_complete=1, session=37,
    ))
    assert ftp.reached_eof
    assert ftp.read_gaps == [(0, 80)]
    assert not ftp.event_complete
    assert not completed
    assert destination.read_bytes() == b"original"
    repair = master._decode_payload(sent[-1])
    assert (repair.opcode, repair.offset, repair.size) == (OP_ReadFile, 0, 80)
    ftp.mavlink_packet(ftp_reply(
        repair.seq + 1, OP_Ack, OP_ReadFile, b"x" * 80, session=37,
    ))
    assert destination.read_bytes() == b"x" * 80 + b"tail"
    assert [result.error_code for result in completed] == [FtpError.Success]
    assert ftp.fh is None
    assert ftp.temp_filename is None


@pytest.mark.parametrize("advertised_size", [None, 4, 100000])
def test_staged_download_rejects_missing_received_bytes(tmp_path, advertised_size):
    """A physically truncated staging file is unsafe even without a size ACK."""
    destination = tmp_path / "download.bin"
    destination.write_bytes(b"original")
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.cmd_get(["remote.bin", str(destination)])
    payload = b"" if advertised_size is None else struct.pack("<I", advertised_size)
    ftp.mavlink_packet(ftp_reply(
        ftp.last_op.seq + 1, OP_Ack, OP_OpenFileRO, payload, session=37,
    ))
    # Truncate after receiving the full block, without moving its high-water
    # mark. fstat must catch the missing bytes before atomic publication.
    ftp.mavlink_packet(ftp_reply(
        ftp.pending_burst_seq, OP_Ack, OP_BurstReadFile, b"data", session=37,
    ))
    ftp.fh.truncate(2)
    assert ftp.fh.tell() == 4
    ftp.mavlink_packet(ftp_reply(
        ftp.pending_burst_seq + 1, OP_Nack, OP_BurstReadFile,
        bytes([FtpError.EndOfFile]), offset=4, session=37,
    ))
    assert destination.read_bytes() == b"original"
    assert [result.error_code for result in completed] == [FtpError.InvalidDataSize]
    assert ftp.fh is None
    assert ftp.temp_filename is None


def test_estimated_size_download_with_unrepaired_gap_times_out(tmp_path):
    """Missing bytes still fail and leave the existing destination intact."""
    destination = tmp_path / "download.bin"
    destination.write_bytes(b"original")
    ftp, _master, _sent, completed = helpers.TestMAVFTPReplyCompletion.managed_ftp()
    ftp.ftp_settings.burst_read_size = 80
    ftp.cmd_get(["@PARAM/param.pck", str(destination)])
    ftp.mavlink_packet(ftp_reply(
        ftp.last_op.seq + 1, OP_Ack, OP_OpenFileRO,
        struct.pack("<I", 17544), session=37,
    ))
    ftp.mavlink_packet(ftp_reply(
        ftp.pending_burst_seq + 1, OP_Ack, OP_BurstReadFile,
        b"tail", offset=80, burst_complete=1, session=37,
    ))
    assert ftp.reached_eof
    assert ftp.read_gaps == [(0, 80)]
    with patch("pymavlink.mavftp.time.time", return_value=ftp._event_deadline + 1):
        ftp.idle_task()
    assert destination.read_bytes() == b"original"
    assert [result.error_code for result in completed] == [FtpError.RemoteReplyTimeout]
    assert ftp.fh is None
    assert ftp.temp_filename is None
