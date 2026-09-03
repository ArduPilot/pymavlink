#!/usr/bin/env python3

"""
Exercise MAVFTP end-to-end against a connected flight controller.

This destructive integration test uses a uniquely named temporary path on the
vehicle. It covers the public MAVFTP commands, verifies an upload by download
and CRC, and removes all temporary files and directories when complete (or on
best-effort failure cleanup). Run it only against hardware whose filesystem
may be modified. The default Pixhawk USB port is ``/dev/ttyACM0``; pass a
different device, baud rate, or component ID on the command line as needed.

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

        data = ftp.read_sector(remote, 0, len(payload))
        digest = hashlib.sha256(data).hexdigest() if data is not None else "n/a"
        print(f"download: {len(data) if data is not None else 'none'} bytes sha256={digest}", flush=True)
        if data != payload:
            raise RuntimeError("downloaded data does not match uploaded data")

        result = ftp.cmd_rename([remote, renamed])
        print(f"rename: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"rename failed: {result.error_code.name}")
        result = ftp.cmd_rm([renamed])
        print(f"delete: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"delete failed: {result.error_code.name}")

        directory = f"/APM/mavftp_hwtest_{int(time.time())}"
        result = ftp.cmd_mkdir([directory])
        print(f"mkdir: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"mkdir failed: {result.error_code.name}")
        result = ftp.cmd_rmdir([directory])
        print(f"rmdir: {result.error_code.name}", flush=True)
        if result.error_code != FtpError.Success:
            raise RuntimeError(f"rmdir failed: {result.error_code.name}")

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
