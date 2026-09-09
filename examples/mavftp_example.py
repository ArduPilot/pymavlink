#!/usr/bin/env python3

r"""
Upload a file with MAVFTP, download it again, and verify the round trip.

This is an API example rather than a second MAVFTP command-line client.  The
interesting parts are :func:`upload_file` and :func:`download_file`: each
starts an operation with ``cmd_*`` and then drives the reply loop with
``process_ftp_reply``.

For example::

    python examples/mavftp_example.py \
        --device /dev/ttyACM0 \
        app.lua /APM/Scripts/app.lua

The command leaves the uploaded file on the vehicle.  Add
``--remove-remote`` to clean it up after the download is verified.

The remote file must keep the local file's basename. That lets the example
also exercise ``cmd_crclocal``, ``cmd_crc``, and ``cmd_crccmp`` and verify a
``MATCH`` result after the transfer.

SPDX-FileCopyrightText: 2024-2026 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
"""

import argparse
import filecmp
import logging
import posixpath
from pathlib import Path
from typing import Callable, Optional, Sequence

from pymavlink import mavftp
from pymavlink import mavutil


LOG = logging.getLogger(__name__)


def open_connection(device: str, baudrate: int, source_system: int):
    """Open a MAVLink connection and wait for the vehicle heartbeat."""
    port = mavftp.auto_connect(device)
    LOG.info("Connecting to %s", port.device)
    master = mavutil.mavlink_connection(
        port.device,
        baud=baudrate,
        source_system=source_system,
    )
    mavftp.wait_heartbeat(master)
    return master


def make_ftp(master, debug: int, request_timestamps: bool) -> mavftp.MAVFTP:
    """Create an MAVFTP client and configure the small set used here."""
    ftp = mavftp.MAVFTP(
        master,
        target_system=master.target_system,
        target_component=master.target_component,
    )
    ftp.ftp_settings.debug = debug
    ftp.ftp_settings.list_time = int(request_timestamps)
    return ftp


def show_result(result: mavftp.MAVFTPReturn) -> bool:
    """Display a result and return whether the operation succeeded."""
    result.display_message()
    return result.error_code == mavftp.FtpError.Success


def progress_logger(label: str) -> Callable[[Optional[float]], None]:
    """Return a progress callback suitable for ``cmd_get`` or ``cmd_put``."""
    def report(progress: Optional[float]) -> None:
        if progress is not None:
            LOG.info("%s: %.0f%%", label, progress * 100.0)

    return report


def upload_file(ftp: mavftp.MAVFTP, local_path: Path, remote_path: str, timeout: float) -> bool:
    """Upload one local file using the public MAVFTP transfer API."""
    LOG.info("Uploading %s to %s", local_path, remote_path)

    # cmd_put() starts the CreateFile handshake and returns immediately.
    started = ftp.cmd_put(
        [str(local_path), remote_path],
        progress_callback=progress_logger("upload"),
    )
    if started.error_code != mavftp.FtpError.Success:
        return show_result(started)

    # process_ftp_reply() drives the handshake, writes, retries, and terminate
    # session exchange until the upload is complete.
    return show_result(ftp.process_ftp_reply("put", timeout=timeout))


def download_file(
    ftp: mavftp.MAVFTP, remote_path: str, local_path: Path, timeout: float
) -> bool:
    """Download one remote file using the public MAVFTP transfer API."""
    LOG.info("Downloading %s to %s", remote_path, local_path)

    # cmd_get() creates the request; the library stages the local file and
    # atomically publishes it when process_ftp_reply() completes.
    started = ftp.cmd_get(
        [remote_path, str(local_path)],
        progress_callback=progress_logger("download"),
    )
    if started.error_code != mavftp.FtpError.Success:
        return show_result(started)
    return show_result(ftp.process_ftp_reply("get", timeout=timeout))


def list_directory(ftp: mavftp.MAVFTP, remote_directory: str) -> bool:
    """List a directory and display its entries, including timestamps."""
    LOG.info("Listing %s", remote_directory)
    return show_result(ftp.cmd_list([remote_directory]))


def remove_remote_file(ftp: mavftp.MAVFTP, remote_path: str) -> bool:
    """Remove the uploaded test file when cleanup was requested."""
    LOG.info("Removing %s", remote_path)
    return show_result(ftp.cmd_rm([remote_path]))


def exercise_crc_commands(
    ftp: mavftp.MAVFTP, local_path: Path, remote_path: str, timeout: float
) -> bool:
    """Run local, remote, and local/remote comparison CRC commands."""
    LOG.info("Calculating the local vehicle-compatible CRC")
    if not show_result(ftp.cmd_crclocal([str(local_path)])):
        return False
    local_crc = mavftp.local_file_crc(str(local_path))

    LOG.info("Calculating the remote CRC")
    if not show_result(ftp.cmd_crc([remote_path], timeout=timeout)):
        return False
    if ftp.last_crc != local_crc:
        LOG.error(
            "Remote CRC mismatch for %s: local=0x%08x remote=0x%08x",
            remote_path,
            local_crc,
            ftp.last_crc if ftp.last_crc is not None else 0,
        )
        return False

    remote_directory = posixpath.dirname(remote_path.rstrip("/")) or "/"
    LOG.info("Comparing the local file with its remote counterpart")
    comparison = ftp.cmd_crccmp([str(local_path), remote_directory])
    if not show_result(comparison):
        return False
    if ftp.crccmp_results != ["MATCH"]:
        LOG.error("Unexpected CRC comparison result: %s", ftp.crccmp_results)
        return False
    return True


def argument_parser(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse arguments for the upload/download round-trip example."""
    parser = argparse.ArgumentParser(
        description="Upload a file with MAVFTP, download it, and verify the result."
    )
    parser.add_argument("local_file", type=Path, help="Local file to upload")
    parser.add_argument(
        "remote_file",
        help="Remote destination path; its basename must match LOCAL_FILE",
    )
    parser.add_argument(
        "--download-path",
        type=Path,
        help="Local path for the downloaded copy (default: LOCAL_FILE.download)",
    )
    parser.add_argument(
        "--remove-remote",
        action="store_true",
        help="Remove the remote file after verifying the download",
    )
    parser.add_argument(
        "--no-list",
        action="store_true",
        help="Skip the directory listing before and after the transfer",
    )
    parser.add_argument("--device", default="", help="Serial device (default: autodetect)")
    parser.add_argument(
        "--baudrate", type=int, default=115200, help="Serial baudrate (default: %(default)s)"
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=250,
        help="MAVLink source system ID (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="Timeout for each transfer (default: %(default)s seconds)",
    )
    parser.add_argument(
        "--debug", type=int, choices=(0, 1, 2), default=0, help="MAVFTP debug level"
    )
    parser.add_argument(
        "--no-list-time",
        action="store_true",
        help="Use the baseline listing opcode instead of requesting timestamps",
    )
    parser.add_argument(
        "--loglevel",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
        help="Logging level (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(  # pylint: disable=too-many-return-statements
    argv: Optional[Sequence[str]] = None,
) -> int:
    """Run the upload/download/verify example."""
    args = argument_parser(argv)
    logging.basicConfig(
        level=getattr(logging, args.loglevel),
        format="%(levelname)s - %(message)s",
    )

    if not args.local_file.is_file():
        LOG.error("Local file does not exist: %s", args.local_file)
        return 1
    if posixpath.basename(args.remote_file.rstrip("/")) != args.local_file.name:
        LOG.error(
            "Remote file basename must match local file basename (%s)",
            args.local_file.name,
        )
        return 1
    download_path = args.download_path or Path(f"{args.local_file}.download")
    remote_directory = posixpath.dirname(args.remote_file.rstrip("/")) or "/"
    master = None

    try:
        master = open_connection(args.device, args.baudrate, args.source_system)
        ftp = make_ftp(master, args.debug, not args.no_list_time)

        if not args.no_list and not list_directory(ftp, remote_directory):
            return 1
        if not upload_file(ftp, args.local_file, args.remote_file, args.timeout):
            return 1
        if not download_file(ftp, args.remote_file, download_path, args.timeout):
            return 1

        if not filecmp.cmp(args.local_file, download_path, shallow=False):
            LOG.error("Round-trip verification failed: %s != %s", args.local_file, download_path)
            return 1
        LOG.info("Round-trip verification succeeded: %s == %s", args.local_file, download_path)
        if not exercise_crc_commands(ftp, args.local_file, args.remote_file, args.timeout):
            return 1

        if not args.no_list and not list_directory(ftp, remote_directory):
            return 1
        if args.remove_remote and not remove_remote_file(ftp, args.remote_file):
            return 1
        return 0
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        LOG.error("MAVFTP example failed: %s", exc)
        return 1
    finally:
        if master is not None:
            master.close()


if __name__ == "__main__":
    raise SystemExit(main())
