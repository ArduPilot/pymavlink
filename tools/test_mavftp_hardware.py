#!/usr/bin/env python3

"""
Exercise MAVFTP end-to-end against a connected flight controller.

This destructive integration test uses a uniquely named temporary path on the
vehicle. It covers the public MAVFTP commands, verifies an upload by download
and CRC, and removes all temporary files and directories when complete (or on
best-effort failure cleanup). Run it only against hardware whose filesystem
may be modified. The default Pixhawk USB port is ``/dev/ttyACM0``; pass a
different device, baud rate, or component ID on the command line as needed.

Known hardware limitation: some flight-controller firmware leaves the FTP
session handshake incomplete after synchronous range reads. This test keeps
the same transport and immediately chains rename/delete after the range
checks, so that firmware limitation is reported as a test failure. Reboot the
controller and remove the uniquely named test file if cleanup did not
complete.

SPDX-FileCopyrightText: 2026 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
"""

# FLAKE_CLEAN

import argparse
import hashlib
import io
import sys
import tempfile
import time
import zlib

from pymavlink import mavutil
from pymavlink.mavftp import FtpError, MAVFTP


def run(device: str, baud: int, component: int) -> None:  # pylint: disable=too-many-branches,too-many-locals,too-many-statements
    payload = (b"pymavlink-mavftp-hardware-test\x00" * 8) + bytes(range(64))
    remote = f"/APM/mavftp_hwtest_{int(time.time())}.bin"
    renamed = remote + ".renamed"
    master = mavutil.mavlink_connection(
        device, baud=baud, source_system=250, autoreconnect=False
    )
    ftp = None
    try:
        if master.wait_heartbeat(timeout=10) is None:
            raise RuntimeError("no MAVLink heartbeat received")
        print(
            f"heartbeat system={master.target_system} component={component}",
            flush=True,
        )
        ftp = MAVFTP(master, target_system=master.target_system, target_component=component)
        result = ftp.cmd_status()
        print(f"status: {result.error_code.name}", flush=True)
        result = ftp.cmd_set(["debug", "0"])
        print(f"set debug: {result.error_code.name}", flush=True)
        result = ftp.cmd_cancel()
        print(f"cancel: {result.error_code.name}", flush=True)
        listing = ftp.cmd_list(["/APM"])
        print(f"list: {listing.error_code.name}", flush=True)
        if listing.error_code != FtpError.Success:
            raise RuntimeError(f"directory listing failed: {listing.error_code.name}")
        timestamped_listing = ftp.cmd_list(["/APM"])
        timestamp_count = sum(
            entry.mtime is not None
            for entry in (timestamped_listing.directory_listing or [])
        )
        print(
            f"timestamped list: {timestamped_listing.error_code.name} "
            f"({timestamp_count} entries with mtime; "
            f"mode={'extension' if ftp.list_time_supported else 'fallback'})",
            flush=True,
        )
        if timestamped_listing.error_code != FtpError.Success:
            raise RuntimeError(
                f"timestamped directory listing failed: "
                f"{timestamped_listing.error_code.name}"
            )

        for invalid_path in ("/APM/€", "/APM/" + ("x" * 240)):
            result = ftp.cmd_list([invalid_path])
            print(f"reject invalid path: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.InvalidArguments:
                raise RuntimeError(
                    f"invalid path was not rejected: {result.error_code.name}"
                )

        with tempfile.NamedTemporaryFile(prefix="mavftp_crc_", suffix=".bin") as local_file:
            local_file.write(payload)
            local_file.flush()
            local_crc = MAVFTP.local_file_crc(local_file.name)
            result = ftp.cmd_ftp(["crclocal", local_file.name])
            print(f"crclocal: {result.error_code.name} (0x{local_crc:08x})", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"local CRC failed: {result.error_code.name}")

        directory = f"/APM/mavftp_hwtest_{int(time.time())}"
        result = ftp.cmd_mkdir([directory])
        print(f"mkdir: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"mkdir failed: {result.error_code.name}")
        result = ftp.cmd_rmdir([directory])
        print(f"rmdir: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"rmdir failed: {result.error_code.name}")

        # Remove leftovers from an interrupted prior run; FileNotFound is fine.
        for path in (remote, renamed):
            ftp.cmd_rm([path])

        result = ftp.cmd_put(["-", remote], fh=io.BytesIO(payload))
        result = ftp.process_ftp_reply("CreateFile", timeout=60)
        print(f"upload: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"upload failed: {result.error_code.name}")

        result = ftp.cmd_crc([remote])
        expected_crc = zlib.crc32(payload) & 0xFFFFFFFF
        print(f"crc: {result.error_code.name} (expected 0x{expected_crc:08x})", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"crc failed: {result.error_code.name}")

        result = ftp.cmd_get([remote, "-"])
        result = ftp.process_ftp_reply("Get", timeout=60)
        print(f"get: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"get failed: {result.error_code.name}")

        result = ftp.cmd_list(["/APM"])
        names = {entry.name for entry in (result.directory_listing or [])}
        print(f"list uploaded file: {result.error_code.name} ({remote.rsplit('/', 1)[-1] in names})", flush=True)
        if result.error_code != FtpError.Success or remote.rsplit("/", 1)[-1] not in names:
            raise RuntimeError("uploaded file was not listed")
        if ftp.list_time_supported:
            uploaded_entry = next(
                (
                    entry
                    for entry in (result.directory_listing or [])
                    if entry.name == remote.rsplit("/", 1)[-1]
                ),
                None,
            )
            if uploaded_entry is None or uploaded_entry.mtime is None:
                raise RuntimeError("timestamped listing omitted the uploaded file mtime")
            if abs(time.time() - uploaded_entry.mtime) > 24 * 60 * 60:
                raise RuntimeError(
                    f"uploaded file mtime is implausible: {uploaded_entry.mtime}"
                )
            print(f"uploaded file mtime: {uploaded_entry.mtime}", flush=True)

        data = ftp.read_sector(remote, 0, len(payload))
        digest = hashlib.sha256(data).hexdigest() if data is not None else "n/a"
        print(f"download: {len(data) if data is not None else 'none'} bytes sha256={digest}", flush=True)
        if data != payload:
            raise RuntimeError("downloaded data does not match uploaded data")

        small_range = ftp.read_sector(remote, 0, 2)
        print(f"small full-burst range: {len(small_range) if small_range is not None else 'none'} bytes", flush=True)
        if small_range != payload[:2]:
            raise RuntimeError("small full-burst range did not match uploaded data")
        high_offset = len(payload) - 2
        tail_range = ftp.read_sector(remote, high_offset, 2)
        print(
            f"high-offset range: {len(tail_range) if tail_range is not None else 'none'} "
            f"bytes at {high_offset}",
            flush=True,
        )
        if tail_range != payload[high_offset:]:
            raise RuntimeError("high-offset range did not match uploaded data")

        # Keep the same transport after synchronous ranges.  This specifically
        # verifies that late termination replies do not poison the next session.
        result = ftp.cmd_rename([remote, renamed])
        print(f"rename: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"rename failed: {result.error_code.name}")
        result = ftp.cmd_rm([renamed])
        print(f"delete: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"delete failed: {result.error_code.name}")

        def failing_upload_callback(_size):
            raise RuntimeError("intentional hardware-test callback failure")

        result = ftp.cmd_put(
            ["-", remote],
            fh=io.BytesIO(payload),
            callback=failing_upload_callback,
        )
        result = ftp.process_ftp_reply("Put", timeout=60)
        print(f"callback failure recovery: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Fail:
            raise RuntimeError(
                f"callback failure was not reported: {result.error_code.name}"
            )
        result = ftp.cmd_rm([remote])
        print(f"callback failure cleanup: {result.error_code.name}", flush=True)
        if result.error_code not in (FtpError.Success, FtpError.FileNotFound):
            raise RuntimeError(
                f"callback failure cleanup failed: {result.error_code.name}"
            )

        for cycle in range(3):
            result = ftp.cmd_put(["-", remote], fh=io.BytesIO(payload))
            result = ftp.process_ftp_reply("Put", timeout=60)
            print(f"chain {cycle + 1} upload: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"chained upload failed: {result.error_code.name}")

            result = ftp.cmd_crc([remote])
            print(f"chain {cycle + 1} crc: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"chained CRC failed: {result.error_code.name}")

            result = ftp.cmd_get([remote, "-"])
            result = ftp.process_ftp_reply("Get", timeout=60)
            print(f"chain {cycle + 1} get: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"chained download failed: {result.error_code.name}")

            chained_range = ftp.read_sector(remote, cycle, 3)
            print(
                f"chain {cycle + 1} range: "
                f"{len(chained_range) if chained_range is not None else 'none'} bytes",
                flush=True,
            )
            if chained_range != payload[cycle : cycle + 3]:
                raise RuntimeError(f"chained range failed in cycle {cycle + 1}")

            result = ftp.cmd_rename([remote, renamed])
            print(f"chain {cycle + 1} rename: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"chained rename failed: {result.error_code.name}")

            result = ftp.cmd_crc([renamed])
            print(f"chain {cycle + 1} renamed crc: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(
                    f"chained renamed CRC failed: {result.error_code.name}"
                )

            result = ftp.cmd_get([renamed, "-"])
            result = ftp.process_ftp_reply("Get", timeout=60)
            print(f"chain {cycle + 1} renamed get: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(
                    f"chained renamed download failed: {result.error_code.name}"
                )

            result = ftp.cmd_rm([renamed])
            print(f"chain {cycle + 1} delete: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"chained delete failed: {result.error_code.name}")

        with tempfile.TemporaryDirectory(prefix="mavftp_params_") as temp_dir:
            values = f"{temp_dir}/values.txt"
            result = ftp.cmd_getparams([values])
            result = ftp.process_ftp_reply("GetParams", timeout=60)
            print(f"getparams: {result.error_code.name}", flush=True)
            if result.error_code != FtpError.Success:
                raise RuntimeError(f"getparams failed: {result.error_code.name}")

        print("MAVFTP HARDWARE TEST PASSED", flush=True)
    finally:
        if ftp is not None:
            for path in (remote, renamed):
                try:
                    ftp.cmd_rm([path])
                except Exception:  # pylint: disable=broad-exception-caught
                    # best-effort cleanup only
                    pass
        master.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--component", type=int, default=1)
    args = parser.parse_args()
    try:
        run(args.device, args.baud, args.component)
    except Exception as error:  # pylint: disable=broad-exception-caught
        print(f"MAVFTP HARDWARE TEST FAILED: {error}", file=sys.stderr, flush=True)
        sys.exit(1)
