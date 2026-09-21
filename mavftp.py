#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

"""
MAVLink File Transfer Protocol support - https://mavlink.io/en/services/ftp.html.

Original from MAVProxy/MAVProxy/modules/mavproxy_ftp.py.

SPDX-FileCopyrightText: 2011-2024 Andrew Tridgell, 2024-2026 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
"""

# FLAKE_CLEAN

import contextlib
import glob
import heapq
import logging
import math
import os
import random
import socket
import struct
import sys
import time
import zlib
from argparse import ArgumentParser
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from io import BufferedRandom, BufferedReader, BufferedWriter
from io import BytesIO as SIO  # noqa: N814
from typing import Any, Dict, List, Optional, Set, Tuple, Union, cast

try:
    import argcomplete
    from argcomplete.completers import FilesCompleter

    _ARGCOMPLETE_AVAILABLE = True
except ImportError:
    _ARGCOMPLETE_AVAILABLE = False

    # Dummy class to avoid errors when argcomplete is not available
    class FilesCompleter:  # type: ignore[no-redef] # pylint: disable=too-few-public-methods,missing-class-docstring
        def __init__(self, *args, **kwargs):
            pass


from pymavlink import mavutil

# pylint: disable=too-many-lines
# mypy: disable-error-code="union-attr,arg-type"

from pymavlink.mavftp_op import (
    FTP_OP,
    OP_Ack,
    OP_BurstReadFile,
    OP_CalcFileCRC32,
    OP_CreateDirectory,
    OP_CreateFile,
    OP_ListDirectory,
    OP_ListDirectoryWithTime,
    OP_Nack,
    OP_None,
    OP_OpenFileRO,
    OP_OpenFileWO,
    OP_ReadFile,
    OP_RemoveDirectory,
    OP_RemoveFile,
    OP_Rename,
    OP_ResetSessions,
    OP_TerminateSession,
    OP_TruncateFile,
    OP_WriteFile,
)

ParameterDataType = Union[str, int]
SettingValue = Union[int, float]


# pylint: disable=invalid-name
class FtpError(IntEnum):
    """error codes."""

    Success = 0
    Fail = 1
    FailErrno = 2
    InvalidDataSize = 3
    InvalidSession = 4
    NoSessionsAvailable = 5
    EndOfFile = 6
    UnknownCommand = 7
    FileExists = 8
    FileProtected = 9
    FileNotFound = 10
    NoErrorCodeInPayload = 64
    NoErrorCodeInNack = 65
    NoFilesystemErrorInPayload = 66
    InvalidErrorCode = 67
    PayloadTooLarge = 68
    InvalidOpcode = 69
    InvalidArguments = 70
    PutAlreadyInProgress = 71
    FailToOpenLocalFile = 72
    RemoteReplyTimeout = 73


HDR_Len = 12
MAX_Payload = 239
FTP_SEQ_MODULUS = 1 << 16
FTP_SESSION_MODULUS = 1 << 8
BURST_REPLY_SEQUENCE_WINDOW = 4096
MAX_READ_GAPS = 4096
MAX_READ_RETRIES = 10
MAX_INITIAL_RETRIES = 3
READ_DEADLINE_SECONDS = 5.0
CRC_TIMEOUT_SECONDS = 5.0
# Keep a batch of encoded MAVLink packets below a normal Ethernet MTU.  This
# is used only when the underlying pymavlink link supports collecting writes.
MAX_NETWORK_BATCH = 1200
# The server null-terminates the final byte of its filename buffer.
MAX_FTP_NAME = MAX_Payload - 1
# pylint: enable=invalid-name


@dataclass
class DirectoryEntry:
    """Directory entry, optionally including a Unix modification timestamp."""

    name: str
    is_dir: bool
    size_b: int
    mtime: Optional[int] = None


def local_file_crc(name: str) -> int:
    """Return the ArduPilot-compatible CRC32 of a local file."""
    crc = 0xFFFFFFFF
    with open(name, "rb") as file_handle:
        while True:
            block = file_handle.read(65536)
            if not block:
                break
            crc = zlib.crc32(block, crc)
    return crc ^ 0xFFFFFFFF


class WriteQueue:  # pylint: disable=too-few-public-methods
    """
    Manages a queue of write operations for the MAVFTP class.

    Keeps track of offsets and sizes for pending write operations to ensure orderly processing.
    """

    def __init__(self, ofs: int, size: int) -> None:
        self.ofs = ofs  # Offset where the write operation starts.
        self.size = size  # Size of the data to be written.
        self.last_send = 0  # Timestamp of the last send operation.


class MAVLinkBatchWriter:  # pylint: disable=too-few-public-methods
    """Collect encoded MAVLink packets for one link write."""

    def __init__(self) -> None:
        self.packets: List[bytes] = []

    def write(self, packet: bytes) -> int:
        """Collect one encoded packet."""
        self.packets.append(bytes(packet))
        return len(packet)


class ParamData:
    """A class to manage parameter values and defaults for ArduPilot configuration."""

    def __init__(self) -> None:
        self.params: List[
            Tuple[bytes, float, type]
        ] = []  # params as (name, value, ptype)
        self.defaults: Union[None, List[Tuple[bytes, float, type]]] = (
            None  # defaults as (name, value, ptype)
        )

    def add_param(self, name: bytes, value: float, ptype: type) -> None:
        self.params.append((name, value, ptype))

    def add_default(self, name: bytes, value: float, ptype: type) -> None:
        if self.defaults is None:
            self.defaults = []
        self.defaults.append((name, value, ptype))


class MAVFTPSetting:  # pylint: disable=too-few-public-methods
    """A single MAVFTP setting with a name, type, value and default value."""

    def __init__(self, name: str, s_type: type, default: SettingValue) -> None:
        self.name: str = name
        self.type = s_type
        try:
            normalized_default = s_type(default)
        except OverflowError as exc:
            raise ValueError(f"invalid default for {name}: {default}") from exc
        self.default: SettingValue = normalized_default
        self.value: SettingValue = self.default


class MAVFTPSettings:
    """A collection of MAVFTP settings.

    Timeout settings use exclusive lower bounds because zero or the minimum
    retry interval would disable the state machine's safety margin.
    """

    _BOUNDS = {
        "debug": (0, 2, False),
        "pkt_loss_tx": (0, 100, False),
        "pkt_loss_rx": (0, 100, False),
        "pkt_lag_tx": (0, None, False),
        "pkt_lag_rx": (0, None, False),
        "pkt_lag_jitter_tx": (0, None, False),
        "pkt_lag_jitter_rx": (0, None, False),
        "max_backlog": (1, None, False),
        "burst_read_size": (1, MAX_Payload, False),
        "write_size": (1, MAX_Payload, False),
        "write_qsize": (1, None, False),
        "read_retry_time": (0, None, False),
        "retry_time": (0.1, None, True),
        "crccmp_timeout": (0.1, None, True),
        "idle_detection_time": (0, None, True),
        "list_time": (0, 1, False),
        "list_time_timeout": (0, None, True),
        "list_retries": (0, None, False),
    }

    def __init__(
        self, s_vars: List[Union[MAVFTPSetting, Tuple[str, type, SettingValue]]]
    ) -> None:
        self._vars: Dict[str, MAVFTPSetting] = {}
        for v in s_vars:
            self.append(v)
        self.validate()

    def append(
        self, v: Union[MAVFTPSetting, Tuple[str, type, SettingValue]]
    ) -> None:
        """Add or replace a setting after validating the resulting collection."""
        if isinstance(v, MAVFTPSetting):
            setting = self.__copy_setting(v)
        else:
            (name, s_type, default) = v
            setting = MAVFTPSetting(name, s_type, default)
        candidate_vars = self._vars.copy()
        candidate_vars[setting.name] = setting
        self.__validate_vars(candidate_vars)
        self._vars = candidate_vars

    def validate(self) -> None:
        """Validate settings required by the MAVFTP state machine."""
        self.__validate_vars(self._vars)

    def has_setting(self, name: str) -> bool:
        """Return whether a setting exists."""
        return name in self._vars

    def get_setting(self, name: str) -> MAVFTPSetting:
        """Return a detached snapshot of a named setting."""
        return self.__copy_setting(self._vars[name])

    @classmethod
    def __validate_vars(cls, settings: Dict[str, MAVFTPSetting]) -> None:
        """Validate settings without mutating the active collection."""
        for name, setting in settings.items():
            value = setting.type(setting.value)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            bounds = cls._BOUNDS.get(name)
            if bounds is None:
                continue
            minimum, maximum, exclusive_minimum = bounds
            if (
                (minimum is not None and (value <= minimum if exclusive_minimum else value < minimum))
                or (maximum is not None and value > maximum)
            ):
                raise ValueError(f"invalid value for {name}: {value}")
        idle_detection = settings.get("idle_detection_time")
        read_retry = settings.get("read_retry_time")
        if idle_detection is not None and read_retry is not None:
            if idle_detection.type(idle_detection.value) <= read_retry.type(read_retry.value):
                raise ValueError("idle_detection_time must be greater than read_retry_time")

    @staticmethod
    def __copy_setting(setting: MAVFTPSetting) -> MAVFTPSetting:
        """Return a detached setting snapshot."""
        setting_copy = MAVFTPSetting(setting.name, setting.type, setting.default)
        setting_copy.value = setting.value
        return setting_copy

    def __getattr__(self, name: str) -> Union[int, float]:
        """Get attribute."""
        try:
            setting = self._vars[name]
        except KeyError as exc:
            raise AttributeError(name) from exc
        return cast(Union[int, float], setting.type(setting.value))

    def __setattr__(self, name: str, value: Any) -> None:
        """Set attribute."""
        if name[0] == "_":
            self.__dict__[name] = value
            return
        if name in self._vars:
            setting = self.__copy_setting(self._vars[name])
            try:
                setting.value = setting.type(value)
            except OverflowError as exc:
                raise ValueError(f"invalid value for {name}: {value}") from exc
            candidate_vars = self._vars.copy()
            candidate_vars[name] = setting
            self.__validate_vars(candidate_vars)
            self._vars = candidate_vars
            return
        raise AttributeError


class MAVFTPReturn:
    """The result of a MAVFTP operation."""

    def __init__(  # pylint: disable=too-many-arguments
        self,
        operation_name: str,
        error_code: int,
        system_error: int = 0,
        invalid_error_code: int = 0,
        invalid_opcode: int = 0,
        invalid_payload_size: int = 0,
        directory_listing: Optional[List[DirectoryEntry]] = None,
    ) -> None:
        self.operation_name = operation_name
        self.error_code = error_code
        self.system_error = system_error
        self.invalid_error_code = invalid_error_code
        self.invalid_opcode = invalid_opcode
        self.invalid_payload_size = invalid_payload_size
        self.directory_listing = directory_listing

    def display_message(self) -> None:  # pylint: disable=too-many-branches, too-many-statements # noqa: C901, PLR0912, PLR0915
        if self.error_code == FtpError.Success:
            logging.info("%s succeeded", self.operation_name)
        elif self.error_code == FtpError.Fail:
            logging.error("%s failed, generic error", self.operation_name)
        elif self.error_code == FtpError.FailErrno:
            logging.error(
                "%s failed, system error %u", self.operation_name, self.system_error
            )
        elif self.error_code == FtpError.InvalidDataSize:
            logging.error("%s failed, invalid data size", self.operation_name)
        elif self.error_code == FtpError.InvalidSession:
            logging.error(
                "%s failed, session is not currently open", self.operation_name
            )
        elif self.error_code == FtpError.NoSessionsAvailable:
            logging.error("%s failed, no sessions available", self.operation_name)
        elif self.error_code == FtpError.EndOfFile:
            logging.error("%s failed, offset past end of file", self.operation_name)
        elif self.error_code == FtpError.UnknownCommand:
            logging.error("%s failed, unknown command", self.operation_name)
        elif self.error_code == FtpError.FileExists:
            logging.warning(
                "%s failed, file/directory already exists", self.operation_name
            )
        elif self.error_code == FtpError.FileProtected:
            logging.warning(
                "%s failed, file/directory is protected", self.operation_name
            )
        elif self.error_code == FtpError.FileNotFound:
            logging.warning("%s failed, file/directory not found", self.operation_name)

        elif self.error_code == FtpError.NoErrorCodeInPayload:
            logging.error(
                "%s failed, payload contains no error code", self.operation_name
            )
        elif self.error_code == FtpError.NoErrorCodeInNack:
            logging.error("%s failed, no error code", self.operation_name)
        elif self.error_code == FtpError.NoFilesystemErrorInPayload:
            logging.error(
                "%s failed, file-system error missing in payload", self.operation_name
            )
        elif self.error_code == FtpError.InvalidErrorCode:
            logging.error(
                "%s failed, invalid error code %u",
                self.operation_name,
                self.invalid_error_code,
            )
        elif self.error_code == FtpError.PayloadTooLarge:
            logging.error(
                "%s failed, payload is too long %u",
                self.operation_name,
                self.invalid_payload_size,
            )
        elif self.error_code == FtpError.InvalidOpcode:
            logging.error(
                "%s failed, invalid opcode %u", self.operation_name, self.invalid_opcode
            )
        elif self.error_code == FtpError.InvalidArguments:
            logging.error("%s failed, invalid arguments", self.operation_name)
        elif self.error_code == FtpError.PutAlreadyInProgress:
            logging.error("%s failed, put already in progress", self.operation_name)
        elif self.error_code == FtpError.FailToOpenLocalFile:
            logging.error("%s failed, failed to open local file", self.operation_name)
        elif self.error_code == FtpError.RemoteReplyTimeout:
            logging.error("%s failed, remote reply timeout", self.operation_name)
        else:
            logging.error(
                "%s failed, unknown error %u in display_message()",
                self.operation_name,
                self.error_code,
            )

        if self.directory_listing is not None and len(self.directory_listing) > 0:
            total_size = 0
            for entry in self.directory_listing:
                name = entry.name
                size = entry.size_b
                if entry.is_dir and entry.mtime is None:
                    logging.info("   %s/", name)
                elif entry.is_dir and entry.mtime == 0:
                    logging.info("   %s/\t-", name)
                elif entry.is_dir:
                    try:
                        mtime = datetime.fromtimestamp(entry.mtime).strftime(
                            "%Y-%m-%d %H:%M:%S"
                        )
                    except (OverflowError, OSError, ValueError):
                        mtime = str(entry.mtime)
                    logging.info("   %s/\t%s", name, mtime)
                elif entry.mtime is None:
                    logging.info("   %s\t%u", name, size)
                elif entry.mtime == 0:
                    logging.info("   %s\t%u\t-", name, size)
                else:
                    try:
                        mtime = datetime.fromtimestamp(entry.mtime).strftime(
                            "%Y-%m-%d %H:%M:%S"
                        )
                    except (OverflowError, OSError, ValueError):
                        mtime = str(entry.mtime)
                    logging.info("   %s\t%u\t%s", name, size, mtime)
                total_size += max(0, size)
            logging.info("Total size %.2f kByte", total_size / 1024.0)

    @property
    def return_code(self) -> int:
        return self.error_code


TERMINATE_ATTEMPTS = 2


class MAVFTP:  # pylint: disable=too-many-instance-attributes,too-many-public-methods
    """
    Implements the client-side logic for the MAVLink File Transfer Protocol (FTP) over MAVLink connections.

    Handles file operations such as reading, writing, listing directories, and managing sessions.
    """

    def __init__(  # noqa: PLR0915 pylint: disable=too-many-statements
        self,
        master,
        target_system: int,
        target_component: int,
        settings: Optional[MAVFTPSettings] = None,
    ) -> None:
        if settings is None:
            settings = MAVFTPSettings(
                [
                    ("debug", int, 0),
                    # Request directory mtimes; cmd_list falls back to the
                    # standard opcode when an older FC rejects the extension.
                    ("list_time", int, 1),
                    # Some older servers silently discard unknown opcodes.
                    ("list_time_timeout", float, 3.0),
                    ("list_retries", int, 3),
                    ("pkt_loss_tx", int, 0),
                    ("pkt_loss_rx", int, 0),
                    # Optional link simulation. Delays are in milliseconds,
                    # matching MAVProxy's FTP module settings.
                    ("pkt_lag_tx", float, 0.0),
                    ("pkt_lag_rx", float, 0.0),
                    ("pkt_lag_jitter_tx", float, 0.0),
                    ("pkt_lag_jitter_rx", float, 0.0),
                    ("loss_seed", int, 0),
                    ("crccmp_timeout", float, 120.0),
                    ("max_backlog", int, 5),
                    ("burst_read_size", int, 80),
                    ("write_size", int, 80),
                    ("write_qsize", int, 5),
                    ("idle_detection_time", float, 3.7),
                    ("read_retry_time", float, 1.0),
                    ("retry_time", float, 0.5),
                ]
            )
        self.ftp_settings = settings
        self.seq = 0
        self.session = 0
        self.network = 0
        self.last_op: Union[None, FTP_OP] = None
        self.fh: Union[None, SIO, BufferedReader, BufferedWriter, BufferedRandom] = None
        self.filename: Union[None, str] = None
        self.callback = None
        self.callback_failure: Optional[MAVFTPReturn] = None
        self.callback_progress = None
        self.put_callback = None
        self.put_callback_progress = None
        self.total_size = 0
        self.read_gaps: List[Tuple[int, int]] = []
        self.read_gap_times: Dict[Tuple[int, int], float] = {}
        # FTP permits several ReadFile requests in flight. Track their
        # expected response sequences so delayed replies from a prior request
        # cannot be dispatched as a current gap repair.
        self.pending_read_replies: Dict[int, Tuple[int, int]] = {}
        self.pending_read_requests: Dict[int, FTP_OP] = {}
        self.last_gap_send = 0.0
        self.read_retries = 0
        self.read_total = 0
        self.accepted_reply_generation = 0
        self.remote_file_size: int = 0
        self.remote_size_known = False
        self.duplicates = 0
        self.last_read = None
        self.last_burst_read: Union[None, float] = None
        # The start offset and first expected reply sequence of the active burst.
        # Burst packets are streamed with advancing sequence numbers, so both
        # identify whether a reply belongs to the current burst.
        self.pending_burst_offset: Optional[int] = None
        self.pending_burst_seq: Optional[int] = None
        self.pending_burst_retry = False
        self.pending_burst_request: Optional[FTP_OP] = None
        self.op_start: Union[None, float] = None
        self.dir_offset = 0
        self.last_op_time = time.time()
        self.last_send_time = time.time()
        self.rtt = 0.5
        self.rttvar = 0.25
        self.rtt_valid = False
        # Requests are keyed by their uint16 sequence. Retransmissions do
        # not replace the sample (Karn's algorithm), so a reply to a retry
        # cannot make the adaptive timeout spuriously small.
        self.send_times: Dict[int, Optional[float]] = {}
        self.reached_eof = False
        self.read_complete = False
        # Explicit terminal reply, identified by (request opcode, reply
        # sequence). A boolean here lets a delayed reply from an earlier
        # operation complete whichever command is currently waiting.
        self.completed_reply: Optional[Tuple[int, int]] = None
        # A request can be retried with its original sequence number. This is
        # needed both for idempotent recovery after a lost reply and for
        # distinguishing an already-open session from an initial failure.
        self.request_retries = 0
        self.last_op_reply = False
        self.terminal_timeout = False
        # sequence numbers of in-flight terminate/reset requests, None
        # when nothing is outstanding: replies are correlated by
        # sequence so a stale or duplicated reply from an earlier
        # request cannot mark the current one complete
        self.pending_terminate_seq: Optional[int] = None
        self.pending_reset_seq: Optional[int] = None
        self.backlog = 0
        self.burst_size: int = int(self.ftp_settings.burst_read_size)
        self.write_list: Union[None, Set[int]] = None
        self.write_block_size: int = 0
        self.write_acks = 0
        self.write_acked_bytes = 0
        self.write_total = 0
        self.write_file_size = 0
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        # Do not send WriteFile packets until the remote CreateFile handshake
        # has completed. This matters on real links where the ACK can arrive
        # after an idle-task pass.
        self.write_open = False
        # Uploads have several WriteFile requests in flight. Map each
        # response sequence to its requested offset.
        self.pending_write_replies: Dict[int, int] = {}
        self.pending_write_requests: Dict[int, FTP_OP] = {}
        self.write_inflight: Set[int] = set()
        self.write_last_send: Union[None, float] = None
        self.open_retries = 0
        self.list_result: List[DirectoryEntry] = []
        self.list_temp_result: List[DirectoryEntry] = []
        self.list_with_time = False
        self.list_time_retries = 0
        # A MAVFTP instance has one fixed target, so one capability cache is
        # sufficient and avoids repeat probe delays on old servers.
        self.list_time_supported: Optional[bool] = None
        self.requested_size: int = 0
        self.requested_offset: int = 0
        # The synchronous read/read_sector API returns data to its caller and
        # must never publish a file named after the remote path.
        self.read_to_memory = False
        # set per-download by __handle_open_ro_reply: a securely
        # created unique staging file, so concurrent MAVFTP clients on
        # one host (e.g. parallel simulator test runners) cannot share
        # a staging file and its name is not predictable
        self.temp_filename: Optional[str] = None
        # only close file handles this instance opened itself; cmd_put
        # stores a caller-owned handle in self.fh
        self.fh_owned = False

        self.master = master
        self.target_system = target_system
        self.target_component = target_component
        # Optional deterministic loss/latency simulation used by link tests
        # and by applications that want to exercise poor-link behavior.
        self.loss_rng = random.Random()
        self.active_loss_seed: Optional[int] = None
        self.delay_sequence = 0
        self.tx_delay_queue: List[Tuple[float, int, bytes]] = []
        self.rx_delay_queue: List[Tuple[float, int, Any]] = []
        self.delayed_rx_results: List[Tuple[MAVFTPReturn, bool, bool, bool, bool]] = []
        self.last_tx_deadline = 0.0
        self.last_rx_deadline = 0.0
        self._rx_loss_applied = False
        self._ftp_reply_processing_depth = 0
        # ``get_result`` is the result buffer for synchronous ``read()`` and
        # stdout downloads. File downloads deliberately stream to a staging
        # file and do not retain a second, whole-file in-memory copy.
        self.get_result: Union[None, bytes] = None
        self.last_crc: Optional[int] = None
        self.crccmp_results: List[str] = []
        self.crccmp_start: Optional[float] = None
        self.done = False
        self.transfer_active = False
        # Interactive transfers expose periodic status through cmd_status and
        # the idle hook. Callback-driven operations stay quiet unless the
        # caller explicitly asks for progress callbacks.
        self.show_progress = False
        self.last_status_time = 0.0

        # The standalone client normally resets the flight controller FTP
        # state-machine during construction. Keep construction useful for
        # callers that create an instance before a link is available too.
        if self.master is not None:
            self.pending_reset_seq = self.seq
            self.__send(FTP_OP(self.seq, self.session, OP_ResetSessions, 0, 0, 0, 0, None))
            self.process_ftp_reply("ResetSessions")

    def cmd_ftp(self, args: List[str]) -> MAVFTPReturn:  # noqa: PLR0911 pylint: disable=too-many-branches,too-many-return-statements
        """FTP operations."""
        usage = "Usage: ftp <list|set|get|getparams|put|rm|rmdir|rename|mkdir|status|cancel|crc|crclocal|crccmp>"
        if len(args) < 1:
            logging.error(usage)
            return MAVFTPReturn("FTP command", FtpError.InvalidArguments)
        if args[0] == "list":
            return self.cmd_list(args[1:])
        if args[0] == "set":
            return self.cmd_set(args[1:])
        if args[0] == "get":
            return self.cmd_get(args[1:])
        if args[0] == "getparams":
            return self.cmd_getparams(args[1:])
        if args[0] == "put":
            return self.cmd_put(args[1:])
        if args[0] == "rm":
            return self.cmd_rm(args[1:])
        if args[0] == "rmdir":
            return self.cmd_rmdir(args[1:])
        if args[0] == "rename":
            return self.cmd_rename(args[1:])
        if args[0] == "mkdir":
            return self.cmd_mkdir(args[1:])
        if args[0] == "crc":
            return self.cmd_crc(args[1:])
        if args[0] == "crclocal":
            return self.cmd_crclocal(args[1:])
        if args[0] == "crccmp":
            return self.cmd_crccmp(args[1:])
        if args[0] == "status":
            return self.cmd_status()
        if args[0] == "cancel":
            return self.cmd_cancel()
        logging.error(usage)
        return MAVFTPReturn("FTP command", FtpError.InvalidArguments)

    def __send(  # pylint: disable=too-many-branches
        self,
        op: FTP_OP,
        retry: bool = False,
        writer: Optional[Any] = None,
    ) -> None:
        """Send a request, preserving its sequence number on retransmission."""
        if not retry:
            op.seq = self.seq
            self.request_retries = 0
            self.last_op_reply = False
        payload = op.pack()
        plen = len(payload)
        if plen < MAX_Payload + HDR_Len:
            payload.extend(bytearray([0] * ((HDR_Len + MAX_Payload) - plen)))
        if self.master is None or not hasattr(self.master, "mav"):
            logging.error("FTP: Can't send request, no master")
        elif self.__packet_lost("TX"):
            if self.ftp_settings.debug > 1:
                logging.info("FTP: dropping packet TX")
        else:
            lag = self.__packet_delay("TX")
            if lag == 0:
                self.__transmit_payload(payload, writer)
            else:
                self.delay_sequence += 1
                deadline = max(time.monotonic() + lag, self.last_tx_deadline)
                self.last_tx_deadline = deadline
                heapq.heappush(
                    self.tx_delay_queue,
                    (deadline, self.delay_sequence, bytes(payload)),
                )
        expected_reply_seq = (op.seq + 1) % FTP_SEQ_MODULUS
        if op.opcode == OP_BurstReadFile:
            self.pending_burst_offset = op.offset
            self.pending_burst_seq = expected_reply_seq
            self.pending_burst_retry = retry
            self.pending_burst_request = op
        elif op.opcode == OP_ReadFile:
            self.pending_read_replies[expected_reply_seq] = (op.offset, op.size)
            self.pending_read_requests[expected_reply_seq] = op
        elif op.opcode == OP_WriteFile:
            self.pending_write_replies[expected_reply_seq] = op.offset
            self.pending_write_requests[expected_reply_seq] = op
        if not retry:
            self.seq = (self.seq + 1) % FTP_SEQ_MODULUS
            self.send_times[op.seq] = time.time()
        else:
            # Do not use a reply to a retransmitted request as an RTT sample.
            self.send_times[op.seq] = None
            self.request_retries += 1
        self.last_op = op
        now = time.time()
        if self.ftp_settings.debug > 1:
            logging.info("FTP: > %s dt=%.2f", op, now - self.last_op_time)
        self.last_op_time = time.time()
        self.last_send_time = now

    def __transmit_payload(self, payload: bytes, writer: Optional[Any] = None) -> None:
        """Transmit an already encoded MAVLink FTP payload."""
        if self.master is None or not hasattr(self.master, "mav"):
            return
        if writer is not None:
            mav = self.master.mav
            message = mav.file_transfer_protocol_encode(
                self.network, self.target_system, self.target_component, payload
            )
            packet = message.pack(mav) if hasattr(message, "pack") else bytes(message)
            writer.write(packet)
            if hasattr(mav, "seq"):
                mav.seq = (mav.seq + 1) % 256
            if hasattr(mav, "total_packets_sent"):
                mav.total_packets_sent += 1
            if hasattr(mav, "total_bytes_sent"):
                mav.total_bytes_sent += len(packet)
            return
        self.master.mav.file_transfer_protocol_send(
            self.network, self.target_system, self.target_component, payload
        )

    def __packet_lost(self, direction: str) -> bool:
        """Return whether the configured transport simulator drops a packet."""
        seed = int(getattr(self.ftp_settings, "loss_seed", 0))
        if seed != self.active_loss_seed:
            self.loss_rng.seed(None if seed == 0 else seed)
            self.active_loss_seed = seed
        setting = "pkt_loss_tx" if direction == "TX" else "pkt_loss_rx"
        percent = int(getattr(self.ftp_settings, setting))
        return percent > 0 and self.loss_rng.uniform(0, 100) < percent

    def packet_lost(self, direction: str) -> bool:
        """Return whether the configured transport simulator drops a packet."""
        return self.__packet_lost(direction)

    def __packet_delay(self, direction: str) -> float:
        """Return configured one-way delay in seconds, including jitter."""
        lag_name = "pkt_lag_tx" if direction == "TX" else "pkt_lag_rx"
        jitter_name = "pkt_lag_jitter_tx" if direction == "TX" else "pkt_lag_jitter_rx"
        delay_ms = max(0.0, float(getattr(self.ftp_settings, lag_name, 0.0)))
        jitter_ms = float(getattr(self.ftp_settings, jitter_name, 0.0))
        if jitter_ms > 0:
            delay_ms += self.loss_rng.uniform(0, jitter_ms)
        return delay_ms * 0.001

    def packet_delay(self, direction: str) -> float:
        """Return simulated one-way delay in seconds."""
        return self.__packet_delay(direction)

    def __discard_delayed_traffic(self) -> None:
        """Discard queued traffic belonging to a finished operation."""
        self.tx_delay_queue.clear()
        self.rx_delay_queue.clear()
        self.delayed_rx_results.clear()
        self.last_tx_deadline = 0.0
        self.last_rx_deadline = 0.0

    def __flush_delayed_traffic(self) -> None:
        """Deliver due simulated packets in FIFO order."""
        now = time.monotonic()
        while self.tx_delay_queue and self.tx_delay_queue[0][0] <= now:
            _, _, payload = heapq.heappop(self.tx_delay_queue)
            self.__transmit_payload(payload)
        while self.rx_delay_queue and self.rx_delay_queue[0][0] <= now:
            _, _, message = heapq.heappop(self.rx_delay_queue)
            addressed_to_us = (
                self.__reply_addressed_to_local_client(message)
                and self.__reply_from_configured_target(message)
            )
            if addressed_to_us:
                try:
                    op = self.__op_parse(message)
                except (struct.error, TypeError, ValueError):
                    op = None
            else:
                op = None
            reply_matches_last_op = (
                op is not None
                and self.last_op is not None
                and op.req_opcode == self.last_op.opcode
                and op.seq == (self.last_op.seq + 1) % FTP_SEQ_MODULUS
                and op.session == self.session
            )
            reply_matches_active_request = (
                op is not None
                and op.session == self.session
                and self.__reply_matches_active_request(op)
            )
            packet_ret = self.__dispatch_received_packet(message)
            if self._ftp_reply_processing_depth:
                self.delayed_rx_results.append(
                    (
                        packet_ret,
                        reply_matches_last_op,
                        reply_matches_active_request,
                        addressed_to_us and op is None,
                        addressed_to_us,
                    )
                )

    def __receive_packet(self, message) -> Optional[MAVFTPReturn]:
        """Dispatch a reply immediately or enqueue it for simulated latency."""
        if self.__packet_lost("RX"):
            if self.ftp_settings.debug > 1:
                logging.info("FTP: dropping packet RX")
            return MAVFTPReturn("mavlink_packet", FtpError.Fail)
        lag = self.__packet_delay("RX")
        if lag == 0:
            return self.__dispatch_received_packet(message)
        self.delay_sequence += 1
        deadline = max(time.monotonic() + lag, self.last_rx_deadline)
        self.last_rx_deadline = deadline
        heapq.heappush(
            self.rx_delay_queue,
            (deadline, self.delay_sequence, message),
        )
        return None

    def __dispatch_received_packet(self, message) -> MAVFTPReturn:
        """Dispatch a reply after the transport simulator has handled loss."""
        self._rx_loss_applied = True
        try:
            return self.__mavlink_packet(message)
        finally:
            self._rx_loss_applied = False

    def mavlink_packet(self, message) -> Optional[MAVFTPReturn]:
        """Accept an incoming FTP reply from an event-driven MAVLink loop."""
        if self.master is None:
            return MAVFTPReturn("mavlink_packet", FtpError.Fail)
        if message.get_type() != "FILE_TRANSFER_PROTOCOL":
            return MAVFTPReturn("mavlink_packet", FtpError.Fail)
        return self.__receive_packet(message)

    def idle_task(self) -> bool:
        """Service delayed traffic and run the FTP retry state machine."""
        self.__flush_delayed_traffic()
        return self.__idle_task()

    def __write_link_data(self, link: Any, data: bytes, is_stream: bool) -> None:
        """Write encoded data, handling partial stream writes."""
        if is_stream:
            port = getattr(link, "port", None)
            if port is not None and hasattr(port, "sendall"):
                try:
                    port.sendall(data)
                except BlockingIOError:
                    # A non-blocking stream is temporarily full.  Leave the
                    # operation pending so the normal FTP retry machinery can
                    # resend it; this is not a disconnected link.
                    return
                except OSError:
                    if hasattr(link, "handle_disconnect"):
                        link.handle_disconnect()
                return

        offset = 0
        while offset < len(data):
            written = link.write(data[offset:])
            # Datagram and several pymavlink wrappers return None after a
            # complete write. Integer-returning writers may accept a prefix.
            if written is None or written <= 0:
                return
            offset += written

    def __send_batch(self, operations: List[FTP_OP]) -> None:
        """Send several requests in one underlying link write when possible."""
        if (
            len(operations) <= 1
            or self.master is None
            or not hasattr(self.master, "mav")
            or not hasattr(self.master.mav, "file")
            or not hasattr(self.master.mav, "file_transfer_protocol_encode")
        ):
            for operation in operations:
                self.__send(operation)
            return

        mav = self.master.mav
        signing = getattr(mav, "signing", None)
        # Packing a signed packet consumes both the MAVLink sequence and the
        # signing timestamp. A send callback (or another sender sharing the
        # MAVLink instance) can send before the held batch is flushed, making
        # the deferred packets appear to have old signatures at the receiver.
        # Use the normal send path whenever that ordering matters.
        if getattr(signing, "sign_outgoing", False) or getattr(
            mav, "send_callback", None
        ) is not None:
            for operation in operations:
                self.__send(operation)
            return

        link = mav.file
        collector = MAVLinkBatchWriter()
        for operation in operations:
            self.__send(operation, writer=collector)

        if not collector.packets:
            return
        port = getattr(link, "port", None)
        port_type = getattr(port, "type", None)
        link_name = type(link).__name__
        # WebSocket links also use a stream socket, but ``link.write()`` adds
        # WebSocket framing.  Only raw TCP transports may bypass that wrapper.
        is_stream = link_name in ("mavtcp", "mavtcpin")
        is_network = (
            link_name == "mavudp"
            or port_type in (socket.SOCK_DGRAM, socket.SOCK_STREAM)
            or is_stream
        )
        if not is_network:
            self.__write_link_data(link, b"".join(collector.packets), False)
            return

        batch = bytearray()
        for packet in collector.packets:
            if batch and len(batch) + len(packet) > MAX_NETWORK_BATCH:
                self.__write_link_data(link, bytes(batch), is_stream)
                batch = bytearray()
            batch.extend(packet)
        if batch:
            self.__write_link_data(link, bytes(batch), is_stream)

    def update_rtt(self, sample: float) -> None:
        """Update the smoothed RTT and variance from an unambiguous reply."""
        sample = max(0.001, sample)
        if not self.rtt_valid:
            self.rtt = sample
            self.rttvar = sample / 2.0
            self.rtt_valid = True
            return
        self.rttvar = 0.75 * self.rttvar + 0.25 * abs(self.rtt - sample)
        self.rtt = 0.875 * self.rtt + 0.125 * sample

    def retry_timeout(self) -> float:
        """Return an RTT-sensitive retransmission timeout."""
        minimum = max(0.05, float(self.ftp_settings.retry_time))
        if not self.rtt_valid:
            return max(1.0, minimum)
        return max(minimum, min(10.0, self.rtt + 4.0 * self.rttvar))

    def __initial_request_retry_budget(self) -> float:
        """Return the full wait budget for the initial-request retry ladder."""
        return self.retry_timeout() * sum(
            2 ** min(retry_count, MAX_INITIAL_RETRIES - 1)
            for retry_count in range(MAX_INITIAL_RETRIES + 1)
        )

    @staticmethod
    def __coerce_optional_timeout(
        timeout: Optional[float],
    ) -> Tuple[bool, Optional[float]]:
        """Validate a public optional timeout before sending a request."""
        if timeout is None:
            return True, None
        try:
            timeout = float(timeout)
        except (TypeError, ValueError):
            logging.error("Invalid FTP timeout: %s", timeout)
            return False, None
        if timeout < 0 or not math.isfinite(timeout):
            logging.error("Invalid FTP timeout: %s", timeout)
            return False, None
        return True, timeout

    def __release_staging(self) -> None:
        """Close and remove this instance's own staging resources.
        Caller-owned handles (cmd_put's fh argument) are left alone."""
        if self.fh is not None and self.fh_owned:
            try:
                self.fh.close()
            except (OSError, ValueError):
                pass
        self.fh_owned = False
        if self.temp_filename is not None:
            try:
                os.unlink(self.temp_filename)
            except OSError:
                pass
            self.temp_filename = None

    @staticmethod
    def __create_staging_file(destination_dir: str) -> Tuple[int, str]:
        """Create a private staging file using the caller's current umask.

        ``tempfile.mkstemp`` always creates a 0600 file.  Creating this file
        with ``os.open(..., 0o666)`` instead gives a new destination the same
        permissions as a normal open("wb") without reading the process-global
        umask (which would race other threads creating files).
        """
        for _ in range(100):
            filename = os.path.join(destination_dir, ".mavftp_" + os.urandom(8).hex())
            try:
                return (
                    os.open(os.path.abspath(filename), os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o666),
                    filename,
                )
            except FileExistsError:
                continue
        raise FileExistsError("could not allocate a MAVFTP staging filename")

    @staticmethod
    def __set_staging_mode(
        temp_fd: int, temp_filename: str, destination_mode: Optional[int]
    ) -> None:
        """Apply an existing destination's mode on platforms with or without fchmod."""
        if destination_mode is None:
            return
        if hasattr(os, "fchmod"):
            os.fchmod(temp_fd, destination_mode)
        else:
            os.chmod(temp_filename, destination_mode)

    @staticmethod
    def __fsync_directory(directory: str) -> None:
        """Flush a directory entry after atomically replacing a download."""
        try:
            flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
            directory_fd = os.open(directory, flags)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError as exc:
            logging.warning(
                "FTP: failed to fsync destination directory %s: %s", directory, exc
            )

    def __terminate_session(self) -> MAVFTPReturn:  # pylint: disable=too-many-branches,too-many-statements
        """Terminate current session."""
        # Delayed requests from the old operation must not be delivered after
        # cancellation or completion. The termination packet is queued below
        # after this purge and is therefore the only packet retained.
        self.__discard_delayed_traffic()
        self.pending_terminate_seq = self.seq
        self.__send(
            FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None)
        )
        self.__release_staging()
        self.fh = None
        self.filename = None
        self.read_to_memory = False
        self.remote_size_known = False
        self.transfer_active = False
        self.write_list = None
        self.write_open = False
        self.show_progress = False
        callback = self.callback
        self.callback = None
        if callback is not None:
            # tell caller that the transfer failed
            try:
                callback(None)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: download callback failed during cleanup: %s", exc)
        callback_progress = self.callback_progress
        self.callback_progress = None
        if callback_progress is not None:
            try:
                callback_progress(None)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: download progress callback failed during cleanup: %s", exc)
        put_callback = self.put_callback
        self.put_callback = None
        if put_callback is not None:
            # tell caller that the transfer failed
            try:
                put_callback(None)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: upload callback failed during cleanup: %s", exc)
        put_callback_progress = self.put_callback_progress
        self.put_callback_progress = None
        if put_callback_progress is not None:
            try:
                put_callback_progress(None)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: upload progress callback failed during cleanup: %s", exc)
        self.read_gaps = []
        self.read_total = 0
        self.read_gap_times = {}
        self.pending_read_replies = {}
        self.pending_read_requests = {}
        self.last_read = None
        self.last_burst_read = None
        self.pending_burst_offset = None
        self.pending_burst_seq = None
        self.pending_burst_request = None
        self.reached_eof = False
        self.backlog = 0
        self.duplicates = 0
        self.pending_write_replies = {}
        self.pending_write_requests = {}
        self.write_inflight.clear()
        if self.ftp_settings.debug > 0:
            logging.info("FTP: Terminated session")
        if self.master is None:
            self.pending_terminate_seq = None
            termination_result = MAVFTPReturn(
                "TerminateSession", FtpError.RemoteReplyTimeout
            )
        else:
            termination_timeout = min(1.0, self.retry_timeout())
            termination_result = self.process_ftp_reply(
                "TerminateSession", timeout=termination_timeout
            )
        for _attempt in range(1, TERMINATE_ATTEMPTS):
            if termination_result.error_code == FtpError.Success:
                break
            if self.master is None:
                break
            self.pending_terminate_seq = self.seq
            self.__send(
                FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None)
            )
            termination_result = self.process_ftp_reply(
                "TerminateSession", timeout=termination_timeout
            )
        if termination_result.error_code != FtpError.Success:
            # Do not let an unanswered old handshake block a later operation.
            self.pending_terminate_seq = None
        self.session = (self.session + 1) % FTP_SESSION_MODULUS
        return termination_result

    def __has_active_session(self) -> bool:
        """Return whether a file operation may have opened a remote session."""
        if self.fh is not None or self.write_list is not None:
            return True
        return (
            self.filename is not None
            and self.last_op is not None
            and self.last_op.opcode
            in {
                OP_OpenFileRO,
                OP_BurstReadFile,
                OP_ReadFile,
                OP_CreateFile,
                OP_WriteFile,
            }
        )

    @staticmethod
    def __encode_path(path: str) -> Optional[bytearray]:
        """Encode an FTP path, returning None for unsupported characters."""
        try:
            encoded = bytearray(path, "ascii")
        except UnicodeEncodeError:
            return None
        if not encoded or b"\0" in encoded or len(encoded) > MAX_FTP_NAME:
            return None
        return encoded

    def cmd_list(
        self, args: List[str], timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """List files."""
        valid_timeout, timeout = self.__coerce_optional_timeout(timeout)
        if not valid_timeout:
            return MAVFTPReturn("ListDirectory", FtpError.InvalidArguments)
        self.list_result = []
        self.list_temp_result = []
        if len(args) == 0:
            dname = "/"
        elif len(args) == 1:
            dname = args[0]
        else:
            logging.error("Usage: list [directory]")
            return MAVFTPReturn("ListDirectory", FtpError.InvalidArguments)
        logging.info("Listing %s", dname)
        enc_dname = self.__encode_path(dname)
        if enc_dname is None:
            logging.error("Invalid directory name: %s", dname)
            return MAVFTPReturn("ListDirectory", FtpError.InvalidArguments)
        self.total_size = 0
        self.dir_offset = 0
        self.list_time_retries = 0
        self.list_with_time = (
            bool(getattr(self.ftp_settings, "list_time", 0))
            and self.list_time_supported is not False
        )
        op = FTP_OP(
            self.seq,
            self.session,
            OP_ListDirectoryWithTime if self.list_with_time else OP_ListDirectory,
            len(enc_dname),
            0,
            0,
            self.dir_offset,
            enc_dname,
        )
        self.__send(op)
        if timeout is None:
            timeout = max(5.0, self.__initial_request_retry_budget())
            if self.list_with_time:
                list_time_timeout = float(
                    getattr(self.ftp_settings, "list_time_timeout", 3.0)
                )
                list_retries = int(getattr(self.ftp_settings, "list_retries", 3))
                list_probe_timeout = max(self.retry_timeout(), list_time_timeout)
                timeout = max(
                    timeout,
                    list_probe_timeout * (list_retries + 1)
                    # Leave room for a complete baseline listing after a silent
                    # timestamp probe. That listing has its own retry ladder.
                    + float(self.ftp_settings.idle_detection_time)
                    + self.__initial_request_retry_budget(),
                )
        return self.process_ftp_reply("ListDirectory", timeout=timeout)

    def __list_without_time(self, mark_unsupported: bool = True) -> None:
        """Restart a timestamp listing with the baseline directory opcode."""
        self.list_with_time = False
        if mark_unsupported:
            self.list_time_supported = False
        self.list_time_retries = 0
        self.dir_offset = 0
        self.total_size = 0
        self.list_temp_result = []
        more = self.last_op
        assert more is not None  # noqa: S101
        more.opcode = OP_ListDirectory
        more.offset = 0
        self.__send(more)

    def __handle_list_reply(  # pylint: disable=too-many-boolean-expressions,too-many-branches,too-many-statements
        self, op: FTP_OP, _m
    ) -> MAVFTPReturn:
        """Handle OP_ListDirectory reply."""
        with_time = op.req_opcode == OP_ListDirectoryWithTime
        if with_time != self.list_with_time:
            # A reply to the capability probe may arrive after fallback to
            # the baseline opcode. It belongs to the abandoned request.
            return MAVFTPReturn("ListDirectory", FtpError.Fail)
        if op.opcode == OP_Ack and op.payload is not None:
            if with_time:
                self.list_time_supported = True
            dentries = sorted(op.payload.split(b"\x00"))
            for d in dentries:
                if len(d) == 0:
                    continue
                self.dir_offset += 1
                try:
                    dir_entry = str(d, "ascii")
                except UnicodeDecodeError as error:
                    logging.debug(error)
                    continue
                if dir_entry[0] == "D":
                    # Timestamped servers append the same trailing fields to
                    # directories as to files: D<name>\t<size>\t<mtime>.
                    # A few servers still return bare directory names even
                    # when the extended listing opcode is requested.
                    fields = dir_entry[1:].split("\t")
                    mtime: Optional[int] = None
                    size = 0
                    name = dir_entry[1:]
                    if with_time and len(fields) >= 3:
                        name = "\t".join(fields[:-2])
                        try:
                            size = int(fields[-2])
                        except (ValueError, TypeError, OverflowError):
                            size = 0
                        try:
                            mtime = int(fields[-1])
                        except (ValueError, TypeError, OverflowError):
                            mtime = 0
                    self.list_temp_result.append(
                        DirectoryEntry(name=name, is_dir=True, size_b=size, mtime=mtime)
                    )
                elif dir_entry[0] == "F":
                    size_str = ""
                    try:
                        fields = dir_entry[1:].rsplit("\t", 2 if with_time else 1)
                    except ValueError:
                        logging.error("Invalid file entry: %s", dir_entry)
                        continue
                    expected_fields = 3 if with_time else 2
                    if len(fields) != expected_fields:
                        logging.error("Invalid file entry: %s", dir_entry)
                        continue
                    if with_time:
                        name, size_str, mtime_str = fields
                        try:
                            mtime = int(mtime_str)
                        except (ValueError, TypeError, OverflowError):
                            logging.error("Invalid file mtime: %s", mtime_str)
                            mtime = 0
                    else:
                        name, size_str = fields
                        mtime = None
                    try:
                        size = int(size_str)
                    except (ValueError, TypeError, OverflowError):
                        logging.error("Invalid file size: %s", size_str)
                        size = 0
                    self.list_temp_result.append(
                        DirectoryEntry(name=name, is_dir=False, size_b=size, mtime=mtime)
                    )
                else:
                    logging.info(d)
            # ask for more
            more = self.last_op
            more.offset = self.dir_offset
            self.__send(more)
        elif (
            self.list_with_time
            and self.dir_offset == 0
            and op.opcode == OP_Nack
            and op.payload is not None
            and len(op.payload) >= 1
            and op.payload[0] in (FtpError.Fail, FtpError.UnknownCommand)
        ):
            # Older servers reject the extension. Retry the same listing with
            # the standard opcode before reporting a failure to the caller.
            # ``Fail`` is not a capability answer. Retry the baseline request
            # for this listing, but keep probing on later listings. Only
            # UnknownCommand proves the extension is unsupported.
            self.__list_without_time(
                mark_unsupported=op.payload[0] == FtpError.UnknownCommand
            )
            return MAVFTPReturn("ListDirectory", FtpError.Success)
        elif (
            op.opcode == OP_Nack
            and op.payload is not None
            and len(op.payload) == 1
            and op.payload[0] == FtpError.EndOfFile
        ):
            self.list_result = self.list_temp_result
            self.completed_reply = (op.req_opcode, op.seq)
            return MAVFTPReturn(
                "ListDirectory", FtpError.Success, directory_listing=self.list_result
            )
        else:
            return self.__decode_ftp_ack_and_nack(op)
        return MAVFTPReturn("ListDirectory", FtpError.Success)

    def read_sector(self, path: str, offset: int, size: int) -> Optional[bytes]:
        logging.info("reading sector %s, offset=%u, size=%u", path, offset, size)
        return self.read(path, size, offset)

    def read(self, path: str, size: int, offset: int = 0) -> Optional[bytes]:  # pylint: disable=too-many-statements
        """Get file."""
        if size < 0 or offset < 0:
            logging.error("Invalid read range: offset=%u size=%u", offset, size)
            return None
        enc_fname = self.__encode_path(path)
        if enc_fname is None:
            logging.error("Invalid file name: %s", path)
            return None
        self.get_result = None
        self.requested_offset = offset
        self.requested_size = size
        self.filename = path
        self.read_to_memory = True
        self.show_progress = False
        self.callback = None
        self.callback_failure = None
        self.callback_progress = None
        self.done = False

        logging.info(
            "Getting %s starting at %u reading %u bytes",
            path,
            self.requested_offset,
            self.requested_size,
        )

        self.op_start = time.time()
        self.read_total = 0
        self.reached_eof = False
        self.burst_size = int(self.ftp_settings.burst_read_size)
        if self.burst_size < 1 or self.burst_size > 239:
            self.burst_size = 239
        self.open_retries = 0
        op = FTP_OP(
            self.seq, self.session, OP_OpenFileRO, len(enc_fname), 0, 0, 0, enc_fname
        )
        self.__send(op)
        timeout = time.time() + READ_DEADLINE_SECONDS
        accepted_reply_generation = self.accepted_reply_generation
        if self.master is None:
            return None
        while not self.done and time.time() < timeout:
            try:
                m = self.master.recv_match(
                    type="FILE_TRANSFER_PROTOCOL",
                    blocking=True,
                    timeout=1.0,
                )
                if m is None:
                    accepted_before_idle = self.accepted_reply_generation
                    self.idle_task()
                    if accepted_before_idle != self.accepted_reply_generation:
                        timeout = time.time() + READ_DEADLINE_SECONDS
                        accepted_reply_generation = self.accepted_reply_generation
                    continue
                self.__receive_packet(m)
                # A busy MAVLink link can carry FTP replies for other GCSes
                # (or stale replies for an earlier request).  They must not
                # turn this into an unbounded wait: only a reply accepted by
                # the active transfer is progress for this deadline.
                if accepted_reply_generation != self.accepted_reply_generation:
                    timeout = time.time() + READ_DEADLINE_SECONDS
                    accepted_reply_generation = self.accepted_reply_generation
            except TypeError as e:
                logging.error(e)
            accepted_before_idle = self.accepted_reply_generation
            self.idle_task()
            if accepted_before_idle != self.accepted_reply_generation:
                timeout = time.time() + READ_DEADLINE_SECONDS
                accepted_reply_generation = self.accepted_reply_generation
            time.sleep(0.0001)
        logging.info("loop closed, gaps:%u, done: %u", len(self.read_gaps), self.done)
        if not self.done and self.__has_active_session():
            self.__terminate_session()
        if len(self.read_gaps) == 0:
            return self.get_result
        logging.error("closed read with %u gaps", len(self.read_gaps))
        return None

    def cmd_set(  # pylint: disable=too-many-return-statements
        self, args: List[str]
    ) -> MAVFTPReturn:
        """Set a MAVFTP configuration parameter."""
        if len(args) != 2:
            logging.error("Usage: set PARAMETERNAME PARAMETERVALUE")
            return MAVFTPReturn("Set", FtpError.InvalidArguments)

        setting_name = args[0]

        # Check if parameter exists in settings
        if setting_name not in self.ftp_settings._vars:  # pylint: disable=protected-access
            logging.error("Invalid parameter name: %s", setting_name)
            return MAVFTPReturn("Set", FtpError.InvalidArguments)

        try:
            setting_value = float(args[1])
        except (ValueError, TypeError, OverflowError):
            logging.error("Invalid parameter value: %s", args[1])
            return MAVFTPReturn("Set", FtpError.InvalidArguments)

        setting = self.ftp_settings._vars[setting_name]  # pylint: disable=protected-access
        if not math.isfinite(setting_value):
            logging.error("Invalid parameter value: %s", args[1])
            return MAVFTPReturn("Set", FtpError.InvalidArguments)
        if setting.type is int:
            if not setting_value.is_integer():
                logging.error("Invalid integer parameter value: %s", args[1])
                return MAVFTPReturn("Set", FtpError.InvalidArguments)
            setting_value = int(setting_value)

        try:
            setattr(self.ftp_settings, setting_name, setting_value)
        except (TypeError, ValueError, OverflowError) as exc:
            logging.error("Invalid value for %s: %s", setting_name, exc)
            return MAVFTPReturn("Set", FtpError.InvalidArguments)
        logging.info("Set %s = %s", setting_name, setting_value)
        return MAVFTPReturn("Set", FtpError.Success)

    def cmd_get(
        self, args: List[str], callback=None, progress_callback=None
    ) -> MAVFTPReturn:
        """Get file."""
        if len(args) == 0 or len(args) > 2:
            logging.error("Usage: get [FILENAME <LOCALNAME>]")
            return MAVFTPReturn("OpenFileRO", FtpError.InvalidArguments)
        fname = args[0]
        enc_fname = self.__encode_path(fname)
        if enc_fname is None:
            logging.error("Invalid file name: %s", fname)
            return MAVFTPReturn("OpenFileRO", FtpError.InvalidArguments)
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        self.get_result = None
        if callback is None or self.ftp_settings.debug > 1:
            logging.info("Getting %s to %s", fname, self.filename)
        self.op_start = time.time()
        self.read_to_memory = False
        self.remote_size_known = False
        self.requested_offset = 0
        self.requested_size = 0
        self.callback = callback
        self.callback_failure = None
        self.callback_progress = progress_callback
        self.show_progress = callback is None
        self.last_status_time = 0.0
        self.transfer_active = True
        self.read_retries = 0
        self.duplicates = 0
        self.reached_eof = False
        self.burst_size = int(self.ftp_settings.burst_read_size)
        if self.burst_size < 1 or self.burst_size > 239:
            self.burst_size = 239
        self.remote_file_size = 0
        self.open_retries = 0
        op = FTP_OP(
            self.seq, self.session, OP_OpenFileRO, len(enc_fname), 0, 0, 0, enc_fname
        )
        self.__send(op)
        return MAVFTPReturn("OpenFileRO", FtpError.Success)

    def __handle_open_ro_reply(  # pylint: disable=too-many-branches,too-many-return-statements
        self, op: FTP_OP, _m
    ) -> MAVFTPReturn:
        """Handle OP_OpenFileRO reply."""
        if self.fh is not None:
            # A preserved-sequence retry can leave more than one handshake
            # reply in the link. Once reading has begun, a late reply must not
            # reopen or truncate the local destination.
            return MAVFTPReturn("OpenFileRO", FtpError.Success)
        if op.opcode == OP_Ack:
            if self.filename is None:
                return MAVFTPReturn("OpenFileRO", FtpError.FileNotFound)
            self.session = op.session
            try:
                if self.callback is not None or self.filename == "-" or self.read_to_memory:
                    self.fh = SIO()
                else:
                    self.__release_staging()
                    destination_dir = os.path.dirname(os.path.abspath(self.filename))
                    try:
                        destination_mode = os.stat(self.filename).st_mode & 0o7777
                    except FileNotFoundError:
                        destination_mode = None
                    (temp_fd, self.temp_filename) = self.__create_staging_file(destination_dir)
                    try:
                        # Keep an existing private destination private for the
                        # entire transfer, not only after it is published.
                        self.__set_staging_mode(
                            temp_fd, self.temp_filename, destination_mode
                        )
                        self.fh = os.fdopen(temp_fd, "wb+")
                    except Exception:
                        os.close(temp_fd)
                        raise
                    self.fh_owned = True
                    self.fh.truncate(0)
                    self.fh.seek(self.requested_offset)
            except Exception as ex:  # pylint: disable=broad-except
                logging.error(
                    "FTP: Failed to open local file %s: %s", self.filename, ex
                )
                self.__terminate_session()
                return MAVFTPReturn("OpenFileRO", FtpError.FileNotFound)
            if op.size == 4 and op.payload is not None and len(op.payload) >= 4:
                self.remote_file_size = (
                    op.payload[0]
                    + (op.payload[1] << 8)
                    + (op.payload[2] << 16)
                    + (op.payload[3] << 24)
                )
                if self.ftp_settings.debug > 0:
                    logging.info("Remote file size: %u", self.remote_file_size)
                if not self.read_to_memory:
                    self.requested_size = self.remote_file_size
                self.remote_size_known = True
            else:
                self.remote_file_size = 0
                self.remote_size_known = False
            read = FTP_OP(
                self.seq,
                self.session,
                OP_BurstReadFile,
                self.burst_size,
                0,
                0,
                self.requested_offset if self.read_to_memory else 0,
                None,
            )
            self.last_burst_read = time.time()
            self.__send(read)
            return MAVFTPReturn("OpenFileRO", FtpError.Success)

        # If the first ACK was lost and the request was retried, the server
        # can report that the descriptor is already open. Treat that specific
        # retry result as success; an initial failure remains a failure.
        recovered_open = (
            self.request_retries > 0
            and op.opcode == OP_Nack
            and op.payload is not None
            and len(op.payload) >= 1
            and op.payload[0] == FtpError.Fail
        )
        if recovered_open:
            return self.__handle_open_ro_reply(
                FTP_OP(
                    op.seq,
                    op.session,
                    OP_Ack,
                    0,
                    OP_OpenFileRO,
                    op.burst_complete,
                    op.offset,
                    bytearray(),
                ),
                _m,
            )
        if (
            op.opcode == OP_Nack
            and op.payload is not None
            and len(op.payload) >= 1
            and op.payload[0] == FtpError.NoSessionsAvailable
        ):
            # Keep the operation alive while the server's session table is
            # temporarily full. __idle_task will retry the same request.
            return MAVFTPReturn("OpenFileRO", FtpError.NoSessionsAvailable)
        ret = self.__decode_ftp_ack_and_nack(op)
        if self.callback is None or self.ftp_settings.debug > 0:
            ret.display_message()
        self.__terminate_session()
        return ret

    def __check_read_finished(  # pylint: disable=too-many-branches,too-many-statements
        self,
    ) -> bool:
        """Check if download has completed."""
        if self.fh is None:
            return True
        if self.op_start is None:
            return True
        if len(self.read_gaps) == 0 and (
            self.reached_eof or (self.read_to_memory and self.read_total >= self.requested_size)
        ):
            ofs = self.__read_position()
            dt = max(time.time() - self.op_start, 1.0e-6)
            rate = (ofs / dt) / 1024.0
            publish_result = True
            callback_consumed = self.callback is not None
            if self.callback is not None:
                # The callback owns the downloaded data.  This is also used
                # for virtual MAVFTP paths such as param.pck?withdefaults=1,
                # which must never be treated as local filenames.
                publish_result = False
                self.fh.seek(0)
                try:
                    callback_result = self.callback(self.fh)
                    if (
                        isinstance(callback_result, MAVFTPReturn)
                        and callback_result.error_code != FtpError.Success
                    ):
                        self.callback_failure = callback_result
                except Exception as exc:  # pylint: disable=broad-exception-caught
                    logging.error("FTP: download callback failed: %s", exc)
                    self.callback_failure = MAVFTPReturn("Get", FtpError.Fail)
                finally:
                    self.callback = None
            elif self.read_to_memory:
                publish_result = False
                self.done = True
            elif self.filename == "-":
                self.fh.seek(0)

            # The callback owns the in-memory result.  In particular, it may
            # close its BytesIO handle, so there is no local publication work
            # left to flush, stat, or otherwise perform on that handle.
            if callback_consumed:
                if not self.remote_size_known:
                    self.requested_size = max(0, ofs - self.requested_offset)
                if ofs < self.requested_size:
                    logging.warning("expected %u, got %u", self.requested_size, ofs)
                if self.callback_progress is not None:
                    self.callback_progress = None
                if self.callback_failure is None:
                    self.__finished_status("downloading", self.filename, ofs)
                self.__terminate_session()
                self.read_complete = True
                return True

            assert self.fh is not None  # noqa: S101
            # Final local-file I/O and publication share this cleanup boundary
            # so a delayed buffered-write failure still releases the session.
            try:
                self.fh.flush()
                actual_size = ofs
                if self.read_to_memory or self.filename == "-":
                    self.fh.seek(0)
                    result = self.fh.read()
                    actual_size = len(result)
                elif self.temp_filename is not None:
                    actual_size = os.fstat(self.fh.fileno()).st_size
                else:
                    result = b""
                if self.read_to_memory:
                    self.get_result = result[: self.requested_size]
                elif self.filename == "-":
                    self.get_result = result
                if not self.read_to_memory and not self.remote_size_known:
                    self.requested_size = max(0, actual_size - self.requested_offset)
                if self.read_to_memory and len(self.get_result) < self.requested_size:
                    logging.warning(
                        "expected %u, got %u", self.requested_size, len(self.get_result)
                    )
                elif not self.read_to_memory and actual_size < self.requested_size:
                    logging.warning(
                        "expected %u, got %u", self.requested_size, actual_size
                    )
                if self.read_to_memory:
                    logging.info("read %u bytes", len(self.get_result))
                if self.callback_progress is not None:
                    self.callback_progress = None
                if self.filename == "-" and publish_result:
                    stdout_buffer = getattr(sys.stdout, "buffer", None)
                    if stdout_buffer is not None:
                        stdout_buffer.write(result)
                    else:
                        encoding = getattr(sys.stdout, "encoding", None) or "utf-8"
                        sys.stdout.write(result.decode(encoding, errors="surrogateescape"))
                    sys.stdout.flush()
                elif publish_result and self.filename and self.temp_filename:
                    # The staging file is created beside its destination, so
                    # replacement is atomic and does not copy the complete
                    # download back into RAM.
                    if self.fh_owned:
                        os.fsync(self.fh.fileno())
                        self.fh.close()
                        self.fh_owned = False
                    logging.info(
                        "Publishing staged download %s to %s",
                        self.temp_filename,
                        self.filename,
                    )
                    # os.replace gives the staging file's mode to the final
                    # path.  Its mode was chosen when staging was created:
                    # the old destination's mode when it existed, otherwise
                    # the caller's current umask.
                    os.replace(self.temp_filename, self.filename)
                    self.temp_filename = None
                    self.__fsync_directory(
                        os.path.dirname(os.path.abspath(self.filename))
                    )
                if publish_result and self.filename not in (None, "-"):
                    self.done = True
                    logging.info(
                        "Wrote %u/%u bytes to %s in %.2fs %.1fkByte/s",
                        self.read_total,
                        self.requested_size,
                        self.filename,
                        dt,
                        rate,
                    )
                    logging.info(
                        "terminating with %u out of %u (ofs=%u)",
                        self.read_total,
                        self.requested_size,
                        ofs,
                    )
                if self.callback_failure is None:
                    self.__finished_status("downloading", self.filename, ofs)
            except (OSError, ValueError) as exc:
                logging.error(
                    "FTP: failed to publish local destination %s: %s",
                    self.filename,
                    exc,
                )
                if self.callback_failure is None:
                    self.callback_failure = MAVFTPReturn("Get", FtpError.Fail)
            finally:
                # terminate the remote session and release the staging
                # file even when the destination cannot be written
                self.__terminate_session()
                self.read_complete = True
            return True
        return False

    def __write_payload(self, op: FTP_OP) -> bool:
        """Write payload from a read op, returning whether processing may continue."""
        write_offset = op.offset
        if self.read_to_memory:
            write_offset -= self.requested_offset
        payload_size = len(op.payload) if op.payload is not None else 0
        # A burst may arrive out of order, but never needs to be more than the
        # bounded repair window ahead of the transferred range.  This prevents
        # a peer-supplied offset from creating an unbounded sparse local file.
        maximum_offset = max(
            self.__read_position(), self.requested_size, self.read_total
        ) + (MAX_READ_GAPS + 1) * max(1, self.burst_size)
        if write_offset < 0 or write_offset + payload_size > maximum_offset:
            logging.error("FTP: invalid downloaded payload offset %u", op.offset)
            self.callback_failure = MAVFTPReturn("Get", FtpError.InvalidDataSize)
            self.__terminate_session()
            return False
        try:
            self.fh.seek(write_offset)
            self.fh.write(op.payload)
        except (OSError, ValueError) as exc:
            logging.error("FTP: failed to stage downloaded data: %s", exc)
            self.callback_failure = MAVFTPReturn("Get", FtpError.Fail)
            self.__terminate_session()
            return False
        self.read_total += len(op.payload)
        if self.callback_progress is not None and self.remote_file_size:
            try:
                self.callback_progress(self.read_total / self.remote_file_size)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: download progress callback failed: %s", exc)
                self.callback_failure = MAVFTPReturn("Get", FtpError.Fail)
                self.callback_progress = None
                self.__terminate_session()
                return False
        return True

    def __read_position(self) -> int:
        """Return the current remote offset represented by the read buffer."""
        position = self.fh.tell()
        if self.read_to_memory:
            position += self.requested_offset
        return position

    def __seek_read_position(self, offset: int) -> None:
        """Seek the read buffer to a remote offset."""
        if self.read_to_memory:
            offset -= self.requested_offset
        self.fh.seek(offset)

    def __handle_burst_read(self, op: FTP_OP, _m) -> MAVFTPReturn:  # noqa: PLR0911, PLR0915 pylint: disable=too-many-statements,too-many-branches,too-many-return-statements
        """Handle OP_BurstReadFile reply."""
        if self.fh is None or self.filename is None:
            if op.session != self.session:
                # old session
                return MAVFTPReturn("BurstReadFile", FtpError.InvalidSession)
            logging.warning("FTP: Unexpected burst read reply. Will be discarded")
            logging.info(op)
            return MAVFTPReturn("BurstReadFile", FtpError.Fail)
        size = len(op.payload) if op.payload is not None else 0
        if size > self.burst_size:
            # this server doesn't handle the burst size argument
            self.burst_size = MAX_Payload
            if self.ftp_settings.debug > 0:
                logging.info("FTP: Setting burst size to %u", self.burst_size)
        if op.opcode == OP_Ack and self.fh is not None:
            # A retried request must see its first expected reply before a
            # delayed packet can affect its sequence floor.  Other packets
            # still reach the writer: their offsets can preserve useful data
            # and create gaps for normal repair.
            expected_retry_reply = (
                self.pending_burst_seq is not None
                and op.seq == self.pending_burst_seq
            )
            if not self.pending_burst_retry or expected_retry_reply:
                self.pending_burst_retry = False
            if (
                self.pending_burst_seq is not None
                and self.pending_burst_offset is not None
            ):
                sequence_distance = (op.seq - self.pending_burst_seq) & 0xFFFF
                offset_distance = op.offset - self.pending_burst_offset
                # A stream cannot advance its sequence more times than bytes
                # since its requested offset.  Do not let a delayed reply
                # with an implausibly distant sequence ratchet the floor.
                if BURST_REPLY_SEQUENCE_WINDOW < sequence_distance <= offset_distance:
                    self.pending_burst_seq = (self.pending_burst_seq + 1) & 0xFFFF
            ofs = self.__read_position()
            if op.offset < ofs:
                # writing an earlier portion, possibly remove a gap
                gap = (op.offset, len(op.payload))
                if gap in self.read_gaps:
                    if self.read_gap_times.get(gap, 0) > 0 and self.backlog > 0:
                        self.backlog -= 1
                    self.read_gaps.remove(gap)
                    self.read_gap_times.pop(gap)
                    if self.ftp_settings.debug > 0:
                        logging.info(
                            "FTP: removed gap %u, %u, %u",
                            gap,
                            self.reached_eof,
                            len(self.read_gaps),
                        )
                else:
                    if self.ftp_settings.debug > 0:
                        logging.info(
                            "FTP: dup read reply at %u of len %u ofs=%u",
                            op.offset,
                            op.size,
                            self.__read_position(),
                        )
                    self.duplicates += 1
                    return MAVFTPReturn("BurstReadFile", FtpError.Fail)
                if not self.__write_payload(op):
                    return self.callback_failure or MAVFTPReturn(
                        "BurstReadFile", FtpError.Fail
                    )
                if size > 0:
                    self.last_burst_read = time.time()
                self.__seek_read_position(ofs)
                if self.__check_read_finished():
                    return self.callback_failure or MAVFTPReturn(
                        "BurstReadFile", FtpError.Success
                    )
            elif op.offset > ofs:
                # we have a gap
                gap = (ofs, op.offset - ofs)
                max_read = self.burst_size
                gap_count = (gap[1] + max_read - 1) // max_read
                total_gap_count = len(self.read_gaps) + gap_count
                if total_gap_count > MAX_READ_GAPS:
                    logging.error(
                        "FTP: burst reply creates too many gaps (%u)", total_gap_count
                    )
                    self.__terminate_session()
                    return MAVFTPReturn("BurstReadFile", FtpError.InvalidDataSize)
                while True:
                    if gap[1] <= max_read:
                        self.read_gaps.append(gap)
                        self.read_gap_times[gap] = 0
                        break
                    g = (gap[0], max_read)
                    self.read_gaps.append(g)
                    self.read_gap_times[g] = 0
                    gap = (gap[0] + max_read, gap[1] - max_read)
                if not self.__write_payload(op):
                    return self.callback_failure or MAVFTPReturn(
                        "BurstReadFile", FtpError.Fail
                    )
                if size > 0:
                    self.last_burst_read = time.time()
            else:
                if not self.__write_payload(op):
                    return self.callback_failure or MAVFTPReturn(
                        "BurstReadFile", FtpError.Fail
                    )
                if size > 0:
                    self.last_burst_read = time.time()
            # Burst replies have their own advancing sequence stream.  Keep
            # future requests beyond every accepted reply so a new download
            # cannot reuse a stale reply sequence when a server reuses a
            # session identifier.
            if (
                self.pending_burst_seq is not None
                and self.pending_burst_offset is not None
                and ((op.seq - self.pending_burst_seq) & 0xFFFF)
                <= (op.offset - self.pending_burst_offset)
                and self.__seq_is_at_or_after(op.seq, self.seq)
            ):
                self.seq = (op.seq + 1) & 0xFFFF
            if self.__check_read_finished():
                return self.callback_failure or MAVFTPReturn(
                    "BurstReadFile", FtpError.Success
                )
            if op.burst_complete:
                if op.size > 0 and op.size < self.burst_size:
                    # a burst complete with non-zero size and less than burst packet size
                    # means EOF
                    if (
                        not self.reached_eof
                        and self.op_start
                        and self.ftp_settings.debug > 0
                    ):
                        logging.info(
                            "FTP: EOF at %u with %u gaps t=%.2f",
                            self.__read_position(),
                            len(self.read_gaps),
                            time.time() - self.op_start,
                        )
                    self.reached_eof = True
                    self.pending_burst_offset = None
                    self.pending_burst_seq = None
                    self.pending_burst_request = None
                    if self.__check_read_finished():
                        return self.callback_failure or MAVFTPReturn(
                            "BurstReadFile", FtpError.Success
                        )
                    self.__check_read_send()
                    return MAVFTPReturn("BurstReadFile", FtpError.Success)
                more = self.pending_burst_request
                if more is None:
                    return MAVFTPReturn("BurstReadFile", FtpError.Fail)
                more.offset = op.offset + op.size
                if self.ftp_settings.debug > 0:
                    logging.info(
                        "FTP: burst continue at %u %u", more.offset, self.__read_position()
                    )
                self.__send(more)
            # A valid burst reply may be only one part of the transfer.
            # It is successful even when it does not complete the read.
            return MAVFTPReturn("BurstReadFile", FtpError.Success)
        if op.opcode == OP_Nack:
            nack_result = self.__decode_ftp_ack_and_nack(op)
            if nack_result.error_code == FtpError.EndOfFile:
                if not self.reached_eof and op.offset > self.__read_position():
                    # we lost the last part of the burst
                    if self.ftp_settings.debug > 0:
                        logging.error(
                            "FTP: burst lost EOF %u %u", self.__read_position(), op.offset
                        )
                    return MAVFTPReturn("BurstReadFile", FtpError.Fail)
                if (
                    not self.reached_eof
                    and self.op_start
                    and self.ftp_settings.debug > 0
                ):
                    logging.info(
                        "FTP: EOF at %u with %u gaps t=%.2f",
                        self.__read_position(),
                        len(self.read_gaps),
                        time.time() - self.op_start,
                    )
                self.reached_eof = True
                self.pending_burst_offset = None
                self.pending_burst_seq = None
                self.pending_burst_request = None
                if self.__check_read_finished():
                    return self.callback_failure or MAVFTPReturn(
                        "BurstReadFile", FtpError.Success
                    )
                self.__check_read_send()
                return MAVFTPReturn("BurstReadFile", FtpError.Fail)
            if self.ftp_settings.debug > 0:
                logging.error("FTP: burst nack: %s", op)
            self.__terminate_session()
            return nack_result
        logging.warning("FTP: burst error: %s", op)
        return MAVFTPReturn("BurstReadFile", FtpError.Fail)

    def __handle_reply_read(  # pylint: disable=too-many-branches,too-many-return-statements
        self, op: FTP_OP, _m
    ) -> MAVFTPReturn:
        """Handle OP_ReadFile reply."""
        pending_for_offset = any(
            pending_gap[0] == op.offset
            for pending_gap in self.pending_read_replies.values()
        )
        self.pending_read_replies.pop(op.seq, None)
        self.pending_read_requests.pop(op.seq, None)
        if self.fh is None or self.filename is None:
            if self.ftp_settings.debug > 0:
                logging.warning("FTP: Unexpected read reply")
                logging.warning(op)
            return MAVFTPReturn("ReadFile", FtpError.Fail)
        if self.backlog > 0:
            self.backlog -= 1
        if op.opcode == OP_Ack and self.fh is not None:
            gap = (op.offset, op.size)
            # Match by offset first. A reply for an outstanding gap with a
            # different size means the remote file changed; a short delayed
            # reply for a gap already filled by burst data is just a stale
            # duplicate and must not abort the download.
            requested_gap = next(
                (pending_gap for pending_gap in self.read_gaps if pending_gap[0] == op.offset),
                None,
            )
            if requested_gap == gap:
                self.read_gaps.remove(gap)
                self.read_gap_times.pop(gap)
                self.pending_read_replies = {
                    seq: pending_gap
                    for seq, pending_gap in self.pending_read_replies.items()
                    if pending_gap != gap
                }
                self.pending_read_requests = {
                    seq: pending_read
                    for seq, pending_read in self.pending_read_requests.items()
                    if (pending_read.offset, pending_read.size) != gap
                }
                ofs = self.__read_position()
                if not self.__write_payload(op):
                    return self.callback_failure or MAVFTPReturn(
                        "ReadFile", FtpError.Fail
                    )
                self.__seek_read_position(ofs)
                if self.ftp_settings.debug > 0:
                    logging.info(
                        "FTP: removed gap %u, %u, %u",
                        gap,
                        self.reached_eof,
                        len(self.read_gaps),
                    )
                if self.__check_read_finished():
                    return self.callback_failure or MAVFTPReturn(
                        "ReadFile", FtpError.Success
                    )
            elif requested_gap is not None:
                logging.info("FTP: file size changed to %u", op.offset + op.size)
                self.__terminate_session()
                return MAVFTPReturn("ReadFile", FtpError.Fail)
            else:
                # A short reply for a gap already filled by burst data is a
                # delayed duplicate. Preserve the existing safety check for
                # an unsolicited short reply, which can indicate that the
                # remote file changed underneath the transfer.
                if pending_for_offset or op.size >= self.burst_size:
                    self.duplicates += 1
                    if self.ftp_settings.debug > 0:
                        logging.info("FTP: no gap read %u, %u", gap, len(self.read_gaps))
                else:
                    logging.info("FTP: unexpected short read at %u", op.offset)
                    self.__terminate_session()
                    return MAVFTPReturn("ReadFile", FtpError.Fail)
        elif op.opcode == OP_Nack:
            logging.info(
                "FTP: Read failed with %u gaps %s", len(self.read_gaps), str(op)
            )
            ret = self.__decode_ftp_ack_and_nack(op)
            self.__terminate_session()
            return ret
        self.__check_read_send()
        return MAVFTPReturn("ReadFile", FtpError.Success)

    def cmd_put(  # pylint: disable=too-many-return-statements,too-many-statements
        self, args: List[str], fh=None, callback=None, progress_callback=None
    ) -> MAVFTPReturn:
        """Put file."""
        if len(args) == 0 or len(args) > 2:
            logging.error("Usage: put [FILENAME <REMOTENAME>]")
            return MAVFTPReturn("CreateFile", FtpError.InvalidArguments)
        if self.transfer_active or self.write_list is not None:
            logging.error("FTP: put already in progress")
            return MAVFTPReturn("CreateFile", FtpError.PutAlreadyInProgress)
        self.write_block_size = int(self.ftp_settings.write_size)
        if not 1 <= self.write_block_size <= MAX_Payload:
            logging.error("FTP: write_size must be between 1 and %u", MAX_Payload)
            return MAVFTPReturn("CreateFile", FtpError.InvalidArguments)
        if self.ftp_settings.write_qsize < 1:
            logging.error("FTP: write_qsize must be at least 1")
            return MAVFTPReturn("CreateFile", FtpError.InvalidArguments)
        fname = args[0]
        if len(args) > 1:
            filename = args[1]
        else:
            filename = os.path.basename(fname)
        if filename.endswith("/"):
            filename += os.path.basename(fname)
        enc_fname = self.__encode_path(filename)
        if enc_fname is None:
            logging.error("Invalid remote file name: %s", filename)
            return MAVFTPReturn("CreateFile", FtpError.InvalidArguments)
        self.fh = fh
        self.fh_owned = False
        if self.fh is None:
            try:
                self.fh = open(fname, "rb")  # noqa: SIM115 pylint: disable=consider-using-with
                self.fh_owned = True
            except Exception as ex:  # pylint: disable=broad-exception-caught
                logging.error("FTP: Failed to open %s: %s", fname, ex)
                return MAVFTPReturn("CreateFile", FtpError.FailToOpenLocalFile)
        self.filename = filename
        if callback is None:
            logging.info("Putting %s to %s", fname, self.filename)
        self.fh.seek(0, 2)
        file_size = self.fh.tell()
        self.fh.seek(0)

        # setup write list
        self.write_file_size = file_size

        write_blockcount = file_size // self.write_block_size
        if file_size % self.write_block_size != 0:
            write_blockcount += 1

        self.write_list = set(range(write_blockcount))
        self.write_open = False
        self.write_acks = 0
        self.write_acked_bytes = 0
        self.write_total = write_blockcount
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        self.write_last_send = None
        self.write_inflight.clear()

        self.put_callback = callback
        self.put_callback_progress = progress_callback
        self.callback_failure = None
        self.show_progress = callback is None
        self.last_status_time = 0.0
        self.transfer_active = True
        self.read_retries = 0
        self.op_start = time.time()
        op = FTP_OP(
            self.seq, self.session, OP_CreateFile, len(enc_fname), 0, 0, 0, enc_fname
        )
        self.__send(op)
        return MAVFTPReturn("CreateFile", FtpError.Success)

    def __put_finished(self, flen: int) -> None:
        """Finish a put."""
        progress_callback = self.put_callback_progress
        self.put_callback_progress = None
        put_callback = self.put_callback
        self.put_callback = None
        if progress_callback is not None:
            try:
                progress_callback(1.0)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: upload progress callback failed: %s", exc)
                self.callback_failure = MAVFTPReturn("Put", FtpError.Fail)
        if put_callback is not None:
            try:
                put_callback(flen)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: upload callback failed: %s", exc)
                self.callback_failure = MAVFTPReturn("Put", FtpError.Fail)
        elif self.op_start:
            dt = max(time.time() - self.op_start, 1.0e-6)
            rate = (flen / dt) / 1024.0
            logging.info(
                "Put %u bytes to %s file in %.2fs %.1fkByte/s",
                flen,
                self.filename,
                dt,
                rate,
            )
        self.__finished_status("uploading", self.filename, flen)

    def __transfer_status(self) -> Optional[str]:
        """Return a concise status line for the active transfer."""
        if not self.transfer_active:
            return None
        if self.op_start is None:
            return "Transfer in progress"

        elapsed = max(time.time() - self.op_start, 1.0e-6)
        if self.write_list is not None:
            done = min(self.write_acked_bytes, self.write_file_size)
            if self.write_file_size:
                percentage = 100.0 * done / self.write_file_size
            else:
                percentage = 100.0
            rate = (done / elapsed) / 1024.0
            return (
                f"Uploading {self.filename} - {done:d}/{self.write_file_size:d} "
                f"bytes {percentage:.1f}% {rate:.1f} kByte/sec"
            )

        if self.fh is None:
            return f"Opening {self.filename}"

        if self.remote_size_known:
            percentage = min(
                100.0,
                100.0 * self.read_total / max(1, self.remote_file_size),
            )
            progress = (
                f"{self.read_total:d}/{self.remote_file_size:d} bytes "
                f"{percentage:.1f}%"
            )
        else:
            progress = f"{self.read_total:d} bytes"
        rate = (self.read_total / elapsed) / 1024.0
        return (
            f"Downloading {self.filename} - {progress} {rate:.1f} kByte/sec "
            f"({self.read_retries:d} retries {len(self.read_gaps):d} gaps)"
        )

    def __update_status(self) -> None:
        """Log interactive transfer status at a human-friendly rate."""
        if not self.show_progress:
            return
        now = time.time()
        if now - self.last_status_time < 0.5:
            return
        status = self.__transfer_status()
        if status is not None:
            logging.info("FTP: %s", status)
            self.last_status_time = now

    def __finished_status(
        self, verb: str, filename: Optional[str], size: int
    ) -> None:
        """Log the final interactive transfer status once."""
        if not self.show_progress or self.op_start is None:
            return
        elapsed = max(time.time() - self.op_start, 1.0e-6)
        rate = (size / elapsed) / 1024.0
        logging.info(
            "FTP: Finished %s %s (%u bytes %.1f seconds, %.1f kByte/sec)",
            verb,
            filename,
            size,
            elapsed,
            rate,
        )
        self.show_progress = False

    def __handle_create_file_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle OP_CreateFile reply."""
        if self.fh is None:
            self.__terminate_session()
            return MAVFTPReturn("CreateFile", FtpError.FileNotFound)
        if self.write_open:
            # A retransmitted CreateFile reply can arrive after writes have
            # started; it must not restart or duplicate the upload.
            return MAVFTPReturn("CreateFile", FtpError.Success)
        recovered_create = (
            self.request_retries > 0
            and op.opcode == OP_Nack
            and op.payload is not None
            and len(op.payload) >= 1
            and op.payload[0] == FtpError.Fail
            # A non-empty upload immediately probes the recovered session
            # with WriteFile.  An empty upload has no follow-up exchange, so
            # a generic NACK cannot be distinguished from a real failure.
            and self.write_list is not None
            and len(self.write_list) > 0
        )
        if op.opcode == OP_Ack or recovered_create:
            self.session = op.session
            self.write_open = True
            self.__send_more_writes(op)
        else:
            ret = self.__decode_ftp_ack_and_nack(op)
            self.__terminate_session()
            return ret
        return MAVFTPReturn("CreateFile", FtpError.Success)

    def __write_block_len(self, idx: int) -> int:
        """Return the number of source bytes represented by a write block."""
        offset = idx * self.write_block_size
        return max(0, min(self.write_block_size, self.write_file_size - offset))

    def __send_more_writes(self, completed_reply: Optional[FTP_OP] = None) -> None:
        """Send some more writes."""
        # Keep direct-use compatibility for callers that initialize the write
        # state themselves without a CreateFile handshake. Normal puts have
        # last_op == CreateFile until this flag is raised.
        if (
            not self.write_open
            and self.last_op is not None
            and self.last_op.opcode == OP_CreateFile
        ):
            return
        if self.write_list is None or len(self.write_list) == 0:
            # all done
            self.__put_finished(self.write_file_size)
            self.__terminate_session()
            if completed_reply is not None:
                self.completed_reply = (
                    completed_reply.req_opcode,
                    completed_reply.seq,
                )
            return

        now = time.time()
        if self.write_last_send is not None and now - self.write_last_send > max(
            self.retry_timeout(), 0.2
        ):
            # we seem to have lost a block of replies
            self.write_inflight.clear()
            self.write_pending = 0

        n = min(
            max(0, self.ftp_settings.write_qsize - self.write_pending),
            len(self.write_list - self.write_inflight),
        )
        writes: List[FTP_OP] = []
        for _i in range(n):
            # send in round-robin, skipping any that have been acked
            idx = self.write_idx
            while idx not in self.write_list or idx in self.write_inflight:
                idx = (idx + 1) % self.write_total
            ofs = idx * self.write_block_size
            write = next(
                (
                    pending_write
                    for pending_write in self.pending_write_requests.values()
                    if pending_write.offset == ofs
                ),
                None,
            )
            if write is None:
                self.fh.seek(ofs)
                data = self.fh.read(self.write_block_size)
                write = FTP_OP(
                    self.seq,
                    self.session,
                    OP_WriteFile,
                    len(data),
                    0,
                    0,
                    ofs,
                    bytearray(data),
                )
                writes.append(write)
            else:
                self.__send(write, retry=True)
            self.write_inflight.add(idx)
            self.write_idx = (idx + 1) % self.write_total
            self.write_pending += 1
            self.write_last_send = now
        if writes:
            self.__send_batch(writes)

    def __handle_write_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle OP_WriteFile reply."""
        expected_offset = self.pending_write_replies.get(op.seq)
        if (
            op.opcode == OP_Ack
            and expected_offset is not None
            and op.offset != expected_offset
        ):
            logging.error(
                "FTP: Write ACK sequence %u has offset %u, expected %u",
                op.seq,
                op.offset,
                expected_offset,
            )
            self.__terminate_session()
            return MAVFTPReturn("WriteFile", FtpError.InvalidDataSize)

        expected_offset = self.pending_write_replies.pop(op.seq, None)
        self.pending_write_requests.pop(op.seq, None)
        if expected_offset is not None:
            self.pending_write_replies = {
                seq: offset
                for seq, offset in self.pending_write_replies.items()
                if offset != expected_offset
            }
            self.pending_write_requests = {
                seq: pending_write
                for seq, pending_write in self.pending_write_requests.items()
                if pending_write.offset != expected_offset
            }
        elif op.opcode == OP_Ack:
            # A reply sequence that is no longer pending is a delayed or
            # duplicate ACK. It must not move the receive cursor or release
            # slots belonging to the active write window.
            return MAVFTPReturn("WriteFile", FtpError.Success)
        if self.fh is None:
            self.__terminate_session()
            return MAVFTPReturn("WriteFile", FtpError.FileNotFound)
        if op.opcode != OP_Ack:
            logging.error("FTP: Write failed")
            ret = self.__decode_ftp_ack_and_nack(op)
            self.__terminate_session()
            return ret

        # If an ACK moves strictly less than half a ring forward, the
        # intervening requests were not acknowledged. Release those slots for
        # retry, but only count the block named by this ACK as successfully
        # stored. At exactly half a ring the direction is ambiguous, so wait
        # for normal retry handling rather than release active slots.
        idx = op.offset // self.write_block_size
        count = (idx - self.write_recv_idx) % self.write_total
        if self.write_recv_idx < 0 or (
            0 < count and count * 2 < self.write_total
        ):
            for gap_idx in range(1, count):
                self.write_inflight.discard(
                    (self.write_recv_idx + gap_idx) % self.write_total
                )
            self.write_recv_idx = idx
        if self.write_list is not None and idx in self.write_list:
            self.write_list.discard(idx)
            self.write_acks += 1
            self.write_acked_bytes += self.__write_block_len(idx)
        self.write_inflight.discard(idx)
        self.write_pending = len(self.write_inflight)
        if self.write_inflight:
            # Progress on one block means the link is responsive.  Restart
            # the retry window for the remaining batch instead of timing it
            # from the first packet sent in the batch.
            self.write_last_send = time.time()
        if self.put_callback_progress:
            progress = self.write_acks / float(self.write_total)
            try:
                self.put_callback_progress(progress)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.error("FTP: upload progress callback failed: %s", exc)
                self.put_callback_progress = None
                self.callback_failure = MAVFTPReturn("Put", FtpError.Fail)
                self.__terminate_session()
                return self.callback_failure
        self.__send_more_writes(op)
        return MAVFTPReturn("WriteFile", FtpError.Success)

    def cmd_rm(
        self, args: List[str], timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """Remove file."""
        valid_timeout, timeout = self.__coerce_optional_timeout(timeout)
        if not valid_timeout:
            return MAVFTPReturn("RemoveFile", FtpError.InvalidArguments)
        if len(args) != 1:
            logging.error("Usage: rm [FILENAME]")
            return MAVFTPReturn("RemoveFile", FtpError.InvalidArguments)
        fname = args[0]
        logging.info("Removing file %s", fname)
        enc_fname = self.__encode_path(fname)
        if enc_fname is None:
            logging.error("Invalid file name: %s", fname)
            return MAVFTPReturn("RemoveFile", FtpError.InvalidArguments)
        op = FTP_OP(
            self.seq, self.session, OP_RemoveFile, len(enc_fname), 0, 0, 0, enc_fname
        )
        self.__send(op)
        if timeout is None:
            timeout = max(5.0, self.__initial_request_retry_budget())
        return self.process_ftp_reply(
            "RemoveFile", timeout=timeout
        )

    def cmd_rmdir(
        self, args: List[str], timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """Remove directory."""
        valid_timeout, timeout = self.__coerce_optional_timeout(timeout)
        if not valid_timeout:
            return MAVFTPReturn("RemoveDirectory", FtpError.InvalidArguments)
        if len(args) != 1:
            logging.error("Usage: rmdir [DIRECTORYNAME]")
            return MAVFTPReturn("RemoveDirectory", FtpError.InvalidArguments)
        dname = args[0]
        logging.info("Removing directory %s", dname)
        enc_dname = self.__encode_path(dname)
        if enc_dname is None:
            logging.error("Invalid directory name: %s", dname)
            return MAVFTPReturn("RemoveDirectory", FtpError.InvalidArguments)
        op = FTP_OP(
            self.seq,
            self.session,
            OP_RemoveDirectory,
            len(enc_dname),
            0,
            0,
            0,
            enc_dname,
        )
        self.__send(op)
        if timeout is None:
            timeout = max(5.0, self.__initial_request_retry_budget())
        return self.process_ftp_reply(
            "RemoveDirectory", timeout=timeout
        )

    def __handle_remove_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle remove reply."""
        self.completed_reply = (op.req_opcode, op.seq)
        return self.__decode_ftp_ack_and_nack(op)

    def cmd_rename(
        self, args: List[str], timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """Rename file or directory."""
        valid_timeout, timeout = self.__coerce_optional_timeout(timeout)
        if not valid_timeout:
            return MAVFTPReturn("Rename", FtpError.InvalidArguments)
        if len(args) < 2:
            logging.error("Usage: rename [OLDNAME NEWNAME]")
            return MAVFTPReturn("Rename", FtpError.InvalidArguments)
        name1 = args[0]
        name2 = args[1]
        logging.info("Renaming %s to %s", name1, name2)
        enc_name1 = self.__encode_path(name1)
        enc_name2 = self.__encode_path(name2)
        if enc_name1 is None or enc_name2 is None:
            logging.error("Invalid rename path: %s -> %s", name1, name2)
            return MAVFTPReturn("Rename", FtpError.InvalidArguments)
        enc_both = enc_name1 + b"\x00" + enc_name2
        if len(enc_both) > MAX_FTP_NAME:
            logging.error("Rename paths are too long: %s -> %s", name1, name2)
            return MAVFTPReturn("Rename", FtpError.InvalidArguments)
        op = FTP_OP(self.seq, self.session, OP_Rename, len(enc_both), 0, 0, 0, enc_both)
        self.__send(op)
        if timeout is None:
            timeout = max(5.0, self.__initial_request_retry_budget())
        return self.process_ftp_reply(
            "Rename", timeout=timeout
        )

    def __handle_rename_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle rename reply."""
        self.completed_reply = (op.req_opcode, op.seq)
        return self.__decode_ftp_ack_and_nack(op)

    def cmd_mkdir(
        self, args: List[str], timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """Make directory."""
        valid_timeout, timeout = self.__coerce_optional_timeout(timeout)
        if not valid_timeout:
            return MAVFTPReturn("CreateDirectory", FtpError.InvalidArguments)
        if len(args) != 1:
            logging.error("Usage: mkdir NAME")
            return MAVFTPReturn("CreateDirectory", FtpError.InvalidArguments)
        name = args[0]
        logging.info("Creating directory %s", name)
        enc_name = self.__encode_path(name)
        if enc_name is None:
            logging.error("Invalid directory name: %s", name)
            return MAVFTPReturn("CreateDirectory", FtpError.InvalidArguments)
        op = FTP_OP(
            self.seq, self.session, OP_CreateDirectory, len(enc_name), 0, 0, 0, enc_name
        )
        self.__send(op)
        if timeout is None:
            timeout = max(5.0, self.__initial_request_retry_budget())
        return self.process_ftp_reply(
            "CreateDirectory", timeout=timeout
        )

    def __handle_mkdir_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle mkdir reply."""
        self.completed_reply = (op.req_opcode, op.seq)
        return self.__decode_ftp_ack_and_nack(op)

    def cmd_crc(
        self, args: List[str], timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """Get file crc."""
        if len(args) != 1:
            logging.error("Usage: crc [NAME]")
            return MAVFTPReturn("CalcFileCRC32", FtpError.InvalidArguments)
        # ``process_ftp_reply`` uses zero for an unbounded wait. CRC has a
        # bounded default, so retain that meaning for an explicit zero, but
        # reject negative and non-finite caller timeouts before sending work
        # to the vehicle.
        if timeout is None:
            timeout = CRC_TIMEOUT_SECONDS
        else:
            try:
                timeout = float(timeout)
            except (TypeError, ValueError):
                logging.error("Invalid FTP timeout: %s", timeout)
                return MAVFTPReturn("CalcFileCRC32", FtpError.InvalidArguments)
            if timeout == 0:
                timeout = CRC_TIMEOUT_SECONDS
            elif timeout < 0 or not math.isfinite(timeout):
                logging.error("Invalid FTP timeout: %s", timeout)
                return MAVFTPReturn("CalcFileCRC32", FtpError.InvalidArguments)
        name = args[0]
        self.last_crc = None
        self.filename = name
        self.op_start = time.time()
        logging.info("Getting CRC for %s", name)
        enc_name = self.__encode_path(name)
        if enc_name is None:
            logging.error("Invalid file name: %s", name)
            self.filename = None
            self.op_start = None
            return MAVFTPReturn("CalcFileCRC32", FtpError.InvalidArguments)
        op = FTP_OP(
            self.seq,
            self.session,
            OP_CalcFileCRC32,
            len(enc_name),
            0,
            0,
            0,
            bytearray(enc_name),
        )
        self.__send(op)
        return self.process_ftp_reply("CalcFileCRC32", timeout=timeout)

    @staticmethod
    def local_file_crc(name: str) -> int:
        """Return the ArduPilot-compatible CRC32 of a local file."""
        return local_file_crc(name)

    def cmd_crclocal(self, args: List[str]) -> MAVFTPReturn:
        """Calculate the CRC32 used by the vehicle for a local file."""
        if len(args) != 1:
            logging.error("Usage: crclocal NAME")
            return MAVFTPReturn("CalcLocalFileCRC32", FtpError.InvalidArguments)
        try:
            crc = self.local_file_crc(args[0])
        except (OSError, ValueError) as exc:
            logging.error("crclocal failed for %s: %s", args[0], exc)
            return MAVFTPReturn("CalcLocalFileCRC32", FtpError.FailToOpenLocalFile)
        logging.info("crc: %s 0x%08x", args[0], crc)
        return MAVFTPReturn("CalcLocalFileCRC32", FtpError.Success)

    def cmd_crccmp(  # pylint: disable=too-many-branches,too-many-locals,too-many-statements
        self, args: List[str]
    ) -> MAVFTPReturn:
        """Compare local files with same-named files on the vehicle."""
        self.crccmp_results = []
        if len(args) != 2:
            logging.error("Usage: crccmp WILDCARD DESTDIR")
            return MAVFTPReturn("CRCCompare", FtpError.InvalidArguments)

        pattern, destination = args
        if destination == "":
            logging.error("crccmp: empty DESTDIR, use / for the vehicle's root")
            return MAVFTPReturn("CRCCompare", FtpError.InvalidArguments)

        files = sorted(path for path in glob.glob(pattern) if os.path.isfile(path))
        if not files:
            logging.error("crccmp: no files matching %s", pattern)
            return MAVFTPReturn("CRCCompare", FtpError.FileNotFound)

        by_name: Dict[str, List[str]] = {}
        for path in files:
            by_name.setdefault(os.path.basename(path), []).append(path)
        clashes = {name: paths for name, paths in by_name.items() if len(paths) > 1}
        if clashes:
            for name, paths in sorted(clashes.items()):
                logging.error(
                    "crccmp: %s matches %u local files: %s",
                    name,
                    len(paths),
                    " ".join(paths),
                )
            logging.error("crccmp: duplicate names, narrow the wildcard")
            return MAVFTPReturn("CRCCompare", FtpError.InvalidArguments)

        destination = destination.rstrip("/")
        self.crccmp_start = time.time()
        overall_deadline = self.crccmp_start + float(
            getattr(self.ftp_settings, "crccmp_timeout", 120.0)
        )
        operation_failed = False
        crccmp_received_response = False
        logging.info(
            "crccmp: %u files matching %s against %s",
            len(files),
            pattern,
            destination,
        )

        for file_index, local_name in enumerate(files):
            basename = os.path.basename(local_name)
            remote_name = f"{destination}/{basename}"
            try:
                encoded_name = bytearray(remote_name, "ascii")
            except UnicodeEncodeError:
                logging.error("  ERROR   %s (non-ascii remote path)", basename)
                self.crccmp_results.append("ERROR")
                operation_failed = True
                continue
            if len(encoded_name) > MAX_FTP_NAME:
                logging.error(
                    "  ERROR   %s (remote path over %u bytes)",
                    basename,
                    MAX_FTP_NAME,
                )
                self.crccmp_results.append("ERROR")
                operation_failed = True
                continue
            try:
                local_crc = self.local_file_crc(local_name)
            except (OSError, ValueError) as exc:
                logging.error("  ERROR   %s (%s)", basename, exc)
                self.crccmp_results.append("ERROR")
                operation_failed = True
                continue

            remaining = overall_deadline - time.time()
            if remaining <= 0:
                logging.error("  ERROR   %s (CRC comparison deadline exceeded)", basename)
                self.crccmp_results.append("ERROR")
                operation_failed = True
                for skipped_name in files[file_index + 1 :]:
                    logging.warning(
                        "  SKIPPED %s (CRC comparison deadline exceeded)",
                        os.path.basename(skipped_name),
                    )
                    self.crccmp_results.append("SKIPPED")
                break
            # Before any response, reserve a short window for every later
            # file so a silent vehicle cannot consume the whole batch. Once
            # the vehicle has responded, a slow remote CRC can use the
            # remaining batch budget.
            remaining_files = len(files) - file_index
            if crccmp_received_response:
                crc_timeout = remaining
            else:
                reserved_per_file = min(
                    CRC_TIMEOUT_SECONDS, remaining / remaining_files
                )
                crc_timeout = remaining - reserved_per_file * (remaining_files - 1)
            result = self.cmd_crc([remote_name], timeout=crc_timeout)
            crccmp_received_response |= result.error_code != FtpError.RemoteReplyTimeout
            if result.error_code == FtpError.Success and self.last_crc is not None:
                if self.last_crc == local_crc:
                    logging.info("  MATCH   %s 0x%08x", basename, local_crc)
                    self.crccmp_results.append("MATCH")
                else:
                    logging.info(
                        "  DIFFER  %s local 0x%08x remote 0x%08x",
                        basename,
                        local_crc,
                        self.last_crc,
                    )
                    self.crccmp_results.append("DIFFER")
            elif result.error_code == FtpError.FileNotFound:
                logging.info("  MISSING %s", basename)
                self.crccmp_results.append("MISSING")
            else:
                logging.error("  ERROR   %s (%s)", basename, result.operation_name)
                self.crccmp_results.append("ERROR")
                operation_failed = True

        elapsed = time.time() - (
            self.crccmp_start if self.crccmp_start is not None else time.time()
        )
        logging.info(
            "crccmp: %u match, %u differ, %u missing, %u errors, %u skipped in %.1fs",
            self.crccmp_results.count("MATCH"),
            self.crccmp_results.count("DIFFER"),
            self.crccmp_results.count("MISSING"),
            self.crccmp_results.count("ERROR"),
            self.crccmp_results.count("SKIPPED"),
            elapsed,
        )
        return MAVFTPReturn(
            "CRCCompare", FtpError.Fail if operation_failed else FtpError.Success
        )

    def __handle_crc_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle crc reply."""
        self.completed_reply = (op.req_opcode, op.seq)
        if op.opcode == OP_Ack and (
            op.payload is None or len(op.payload) != 4 or op.size != 4
        ):
            logging.error("FTP: CRC ACK has invalid payload size %u", op.size)
            return MAVFTPReturn("CalcFileCRC32", FtpError.InvalidDataSize)
        if op.opcode == OP_Ack:
            (crc,) = struct.unpack("<I", op.payload)
            self.last_crc = crc
            now = time.time()
            if self.op_start:
                logging.info(
                    "crc: %s 0x%08x in %.1fs", self.filename, crc, now - self.op_start
                )
        return self.__decode_ftp_ack_and_nack(op)

    def cmd_cancel(self) -> MAVFTPReturn:
        """Cancel any pending op."""
        return self.__terminate_session()

    def cmd_status(self) -> MAVFTPReturn:
        """Show status."""
        status = self.__transfer_status()
        if status is None:
            logging.info("No transfer in progress")
        else:
            logging.info("FTP: %s", status)
        return MAVFTPReturn("Status", FtpError.Success)

    def __op_parse(self, m) -> FTP_OP:
        """Parse a FILE_TRANSFER_PROTOCOL msg."""
        raw_payload = bytes(m.payload)
        hdr = bytearray(raw_payload[0:HDR_Len])
        (seq, session, opcode, size, req_opcode, burst_complete, _pad, offset) = (
            struct.unpack("<HBBBBBBI", hdr)
        )
        if len(raw_payload) < HDR_Len + size:
            raise struct.error("FTP payload is shorter than its declared payload")
        payload = bytearray(raw_payload[HDR_Len:HDR_Len + size])
        return FTP_OP(
            seq, session, opcode, size, req_opcode, burst_complete, offset, payload
        )

    def __reply_from_configured_target(self, m) -> bool:
        """Return whether a reply came from this instance's remote target.

        FILE_TRANSFER_PROTOCOL has no separate routing envelope for the
        vehicle that handled a request.  Session IDs are only one byte and
        can therefore collide across vehicles, so use MAVLink's source IDs
        when the message wrapper exposes them.  Minimal test and legacy
        wrappers may not provide source accessors; retain their historical
        behavior in that case.
        """
        try:
            source_system = m.get_srcSystem()
            source_component = m.get_srcComponent()
        except AttributeError:
            return True
        if self.target_system not in (0, source_system):
            logging.info(
                "FTP: reply from wrong system %u, expected %u. Will discard message",
                source_system,
                self.target_system,
            )
            return False
        if self.target_component not in (0, source_component):
            logging.info(
                "FTP: reply from wrong component %u, expected %u. Will discard message",
                source_component,
                self.target_component,
            )
            return False
        return True

    def __reply_addressed_to_local_client(self, m) -> bool:
        """Return whether an FTP reply is addressed to this GCS.

        Perform this routing check before decoding the FTP payload: malformed
        traffic for another client is unrelated noise and must not abort our
        receive loop.
        """
        if self.master is None:
            return False
        return (
            m.target_system == self.master.source_system
            and m.target_component == self.master.source_component
        )

    def __reply_matches_active_request(self, op: FTP_OP) -> bool:
        """Return whether a reply can safely be dispatched to the active operation."""
        if op.req_opcode == OP_BurstReadFile:
            return (
                self.pending_burst_offset is not None
                and self.pending_burst_seq is not None
                and self.__seq_is_at_or_after(op.seq, self.pending_burst_seq)
                and op.offset >= self.pending_burst_offset
            )
        if op.req_opcode == OP_ReadFile:
            return op.seq in self.pending_read_replies
        if op.req_opcode == OP_WriteFile:
            return op.seq in self.pending_write_replies

        if op.req_opcode == OP_TerminateSession:
            return (
                self.pending_terminate_seq is not None
                and op.seq == (self.pending_terminate_seq + 1) % FTP_SEQ_MODULUS
            )

        if (
            self.last_op is not None
            and op.req_opcode == self.last_op.opcode
            and op.seq == (self.last_op.seq + 1) % FTP_SEQ_MODULUS
        ):
            return True

        return False

    @staticmethod
    def __seq_is_at_or_after(seq: int, expected: int) -> bool:
        """Return whether a uint16 sequence is equal to or newer than expected."""
        return ((seq - expected) & 0xFFFF) < 0x8000

    def __mavlink_packet(self, m) -> MAVFTPReturn:  # noqa: PLR0911, PGH004, pylint: disable=too-many-branches,too-many-return-statements,too-many-statements
        """Handle a mavlink packet."""
        operation_name = "mavlink_packet"
        mtype = m.get_type()
        if mtype != "FILE_TRANSFER_PROTOCOL":
            logging.error("FTP: Unexpected MAVLink message type %s", mtype)
            return MAVFTPReturn(operation_name, FtpError.Fail)

        if not self.__reply_addressed_to_local_client(m):
            logging.info(
                "FTP: wrong MAVLink target %u component %u. Will discard message",
                m.target_system,
                m.target_component,
            )
            return MAVFTPReturn(operation_name, FtpError.Fail)

        if not self.__reply_from_configured_target(m):
            return MAVFTPReturn(operation_name, FtpError.InvalidSession)

        try:
            op = self.__op_parse(m)
        except (struct.error, TypeError, ValueError) as exc:
            logging.error("FTP: malformed FILE_TRANSFER_PROTOCOL payload: %s", exc)
            return MAVFTPReturn(operation_name, FtpError.InvalidDataSize)
        now = time.time()
        dt = now - self.last_op_time
        if self.ftp_settings.debug > 1:
            logging.info("FTP: < %s dt=%.2f", op, dt)
        allocated_session_reply = (
            op.opcode == OP_Ack
            and op.req_opcode in {OP_OpenFileRO, OP_CreateFile}
        )
        if op.session != self.session and not allocated_session_reply:
            if self.ftp_settings.debug > 0:
                logging.warning(
                    "FTP: wrong session replied %u expected %u. Will discard message",
                    op.session,
                    self.session,
                )
            return MAVFTPReturn(operation_name, FtpError.InvalidSession)
        if not self._rx_loss_applied and self.__packet_lost("RX"):
            if self.ftp_settings.debug > 1:
                logging.warning("FTP: dropping packet RX")
            return MAVFTPReturn(operation_name, FtpError.Fail)

        if not self.__reply_matches_active_request(op):
            if self.ftp_settings.debug > 0:
                logging.warning("FTP: stale reply. Will discard message: %s", op)
            return MAVFTPReturn(operation_name, FtpError.Fail)

        # Only an active reply demonstrates progress; stale packets must not
        # postpone retransmission of the request that is still outstanding.
        self.last_op_time = now
        if (
            self.last_op is not None
            and op.req_opcode == self.last_op.opcode
            and op.seq == (self.last_op.seq + 1) % FTP_SEQ_MODULUS
        ):
            self.last_op_reply = True

        # Only the first reply to a request is an unambiguous RTT sample.
        # Burst replies advance their sequence number, so later packets do
        # not have a corresponding entry in send_times.
        request_seq = (op.seq - 1) % FTP_SEQ_MODULUS
        sent = self.send_times.pop(request_seq, None)
        if sent is not None:
            self.update_rtt(now - sent)

        read_total_before_reply = self.read_total
        reached_eof_before_reply = self.reached_eof
        read_gaps_before_reply = len(self.read_gaps)
        try:
            if (
                op.opcode == OP_Nack
                and op.payload is not None
                and len(op.payload) == 1
                and op.payload[0] == FtpError.NoSessionsAvailable
                and op.req_opcode
                in {
                    OP_ListDirectory,
                    OP_ListDirectoryWithTime,
                    OP_OpenFileRO,
                    OP_CreateFile,
                    OP_RemoveFile,
                    OP_RemoveDirectory,
                    OP_Rename,
                    OP_CreateDirectory,
                    OP_CalcFileCRC32,
                }
            ):
                # Another client may have consumed the server's session table.
                # Keep the current operation intact and let idle processing retry
                # it instead of failing a callback or tearing down a valid local
                # upload/download setup.
                self.last_op_reply = False
                self.last_op_time = now
                return self.__decode_ftp_ack_and_nack(op)

            if op.req_opcode in {OP_ListDirectory, OP_ListDirectoryWithTime}:
                return self.__handle_list_reply(op, m)
            if op.req_opcode == OP_OpenFileRO:
                return self.__handle_open_ro_reply(op, m)
            if op.req_opcode == OP_BurstReadFile:
                return self.__handle_burst_read(op, m)
            if op.req_opcode == OP_ResetSessions:
                return self.__handle_reset_sessions_reply(op, m)
            if op.req_opcode in {OP_None, OP_TerminateSession}:
                if (
                    op.req_opcode == OP_TerminateSession
                    and self.pending_terminate_seq is not None
                    and op.seq == (self.pending_terminate_seq + 1) % FTP_SEQ_MODULUS
                ):
                    # Ack or Nack (InvalidSession means it was already
                    # closed): the handshake has been answered
                    self.pending_terminate_seq = None
                return MAVFTPReturn(operation_name, FtpError.Success)  # ignore reply
            if op.req_opcode == OP_CreateFile:
                return self.__handle_create_file_reply(op, m)
            if op.req_opcode == OP_WriteFile:
                return self.__handle_write_reply(op, m)
            if op.req_opcode in {OP_RemoveFile, OP_RemoveDirectory}:
                return self.__handle_remove_reply(op, m)
            if op.req_opcode == OP_Rename:
                return self.__handle_rename_reply(op, m)
            if op.req_opcode == OP_CreateDirectory:
                return self.__handle_mkdir_reply(op, m)
            if op.req_opcode == OP_ReadFile:
                return self.__handle_reply_read(op, m)
            if op.req_opcode == OP_CalcFileCRC32:
                return self.__handle_crc_reply(op, m)

            logging.info("FTP Unknown %s", str(op))
            return MAVFTPReturn(operation_name, FtpError.InvalidOpcode)
        finally:
            read_progressed = (
                self.read_total > read_total_before_reply
                or (self.reached_eof and not reached_eof_before_reply)
                or len(self.read_gaps) < read_gaps_before_reply
            )
            # The retry limit protects a continuous burst stall, not an
            # entire download. An accepted payload, EOF, or gap repair proves
            # that the link made forward progress and starts a fresh window.
            if op.req_opcode in {OP_BurstReadFile, OP_ReadFile} and read_progressed:
                self.read_retries = 0
            if op.req_opcode not in {OP_BurstReadFile, OP_ReadFile} or read_progressed:
                self.accepted_reply_generation += 1

    def __send_gap_read(self, g) -> None:
        """Send a read for a gap."""
        (offset, length) = g
        if self.ftp_settings.debug > 0:
            logging.info(
                "FTP: Gap read of %u at %u rem=%u blog=%u",
                length,
                offset,
                len(self.read_gaps),
                self.backlog,
            )
        read = next(
            (
                pending_read
                for pending_read in self.pending_read_requests.values()
                if (pending_read.offset, pending_read.size) == g
            ),
            None,
        )
        if read is None:
            read = FTP_OP(
                self.seq, self.session, OP_ReadFile, length, 0, 0, offset, None
            )
            self.__send(read)
        else:
            self.__send(read, retry=True)
        self.read_gaps.remove(g)
        self.read_gaps.append(g)
        self.last_gap_send = time.time()
        self.read_gap_times[g] = self.last_gap_send
        self.backlog += 1

    def __check_read_send(self) -> None:
        """Keep a bounded window of gap reads in flight."""
        if not self.read_gaps:
            return
        now = time.time()
        timeout = self.retry_timeout()
        for gap in list(self.read_gaps):
            sent = self.read_gap_times.get(gap, 0)
            if sent > 0 and now - sent > timeout:
                self.read_gap_times[gap] = 0
                if self.backlog > 0:
                    self.backlog -= 1

        limit = max(1, int(self.ftp_settings.max_backlog))
        for gap in list(self.read_gaps):
            if self.backlog >= limit:
                break
            if self.read_gap_times.get(gap, 0) == 0:
                self.__send_gap_read(gap)

    def __idle_task(self) -> bool:  # pylint: disable=too-many-branches,too-many-return-statements,too-many-statements
        """Check for file gaps and lost requests."""
        now = time.time()
        if self.ftp_settings.idle_detection_time <= self.ftp_settings.read_retry_time:
            logging.error("idle_detection_time must be greater than read_retry_time")
            return True

        # Keep interactive status useful even while no packet is currently
        # outstanding. Callback-driven transfers deliberately remain quiet.
        self.__update_status()

        # Probe the optional mtime listing opcode like MAVProxy does. Some
        # older servers silently ignore it rather than returning UnknownCommand.
        timestamp_probe_eligible = (
            self.list_with_time
            and self.list_time_supported is not True
            and self.dir_offset == 0
            and self.last_op is not None
            and self.last_op.opcode == OP_ListDirectoryWithTime
        )
        if (
            timestamp_probe_eligible
            and now - self.last_op_time
            >= max(
                self.retry_timeout(),
                float(getattr(self.ftp_settings, "list_time_timeout", 3.0)),
            )
        ):
            if self.list_time_retries >= int(
                getattr(self.ftp_settings, "list_retries", 3)
            ):
                if self.ftp_settings.debug > 0:
                    logging.info("FTP: listing timestamps unsupported, retrying without")
                self.__list_without_time(mark_unsupported=False)
            else:
                self.list_time_retries += 1
                if self.ftp_settings.debug > 0:
                    logging.info("FTP: retrying directory timestamp request")
                self.__send(self.last_op, retry=True)
            return False

        # The optional-opcode probe has its own configured deadline.  Do not
        # let the generic idle detector abandon it before that deadline.
        if timestamp_probe_eligible:
            return False

        # see if we lost an open reply
        if (
            self.op_start is not None
            and now - self.op_start > max(
                float(self.ftp_settings.read_retry_time), self.retry_timeout()
            )
            and self.last_op.opcode == OP_OpenFileRO
        ):
            self.op_start = now
            self.open_retries += 1
            if self.open_retries > MAX_READ_RETRIES:
                # fail the get
                self.op_start = None
                self.terminal_timeout = True
                self.__terminate_session()
                return False  # Not idle yet
            if self.ftp_settings.debug > 0:
                logging.info("FTP: retry open")
            self.__send(self.last_op, retry=True)

        # Reuse the request sequence for one-reply operations. MAVFTP servers
        # commonly cache the reply for that sequence, which makes a retry
        # recover a lost request or ACK without repeating the operation.
        # Limit this to a few exponentially delayed attempts because servers
        # that do not cache requests can still apply a mutation more than once.
        initial_opcodes = {
            OP_ListDirectory,
            OP_ListDirectoryWithTime,
            OP_CreateFile,
            OP_RemoveFile,
            OP_RemoveDirectory,
            OP_Rename,
            OP_CreateDirectory,
            OP_CalcFileCRC32,
        }
        retry_timeout = self.retry_timeout() * 2 ** min(
            self.request_retries, MAX_INITIAL_RETRIES - 1
        )
        initial_request_pending = (
            self.last_op is not None
            and not self.last_op_reply
            and self.last_op.opcode in initial_opcodes
            and not (
                self.last_op.opcode == OP_ListDirectoryWithTime
                and self.dir_offset == 0
                and self.list_time_supported is not True
            )
        )
        crc_request_pending = (
            initial_request_pending and self.last_op.opcode == OP_CalcFileCRC32
        )
        if (
            initial_request_pending
            and not crc_request_pending
            and now - self.last_op_time > retry_timeout
        ):
            if self.request_retries >= MAX_INITIAL_RETRIES:
                logging.error("FTP: request timed out: %s", self.last_op)
                self.terminal_timeout = True
                self.__terminate_session()
                return False
            if self.ftp_settings.debug > 0:
                logging.info("FTP: retry request %s", self.last_op)
            self.__send(self.last_op, retry=True)
            return False

        # A request waiting for its next exponential retry must not be
        # mistaken for an otherwise idle transfer. In particular, this lets
        # the final retry in MAX_INITIAL_RETRIES become due.
        # CRC32 is different: the remote calculation may be long-running, and
        # retrying could start duplicate work. Let process_ftp_reply's caller
        # timeout govern it without retransmitting or applying idle detection.
        if initial_request_pending:
            return False

        if (
            len(self.read_gaps) == 0
            and self.last_burst_read is None
            and self.write_list is None
        ):
            return self.__last_send_time_was_more_than_idle_detection_time_ago(now)

        if self.fh is None:
            return self.__last_send_time_was_more_than_idle_detection_time_ago(now)

        # see if burst read has stalled
        if (
            not self.reached_eof
            and self.last_burst_read is not None
            and now - self.last_burst_read > self.retry_timeout()
        ):
            if self.read_retries >= MAX_READ_RETRIES:
                logging.error(
                    "FTP: burst read timed out after %u retries", self.read_retries
                )
                # __terminate_session clears last_burst_read. Preserve the
                # terminal result so process_ftp_reply cannot retain the
                # earlier OpenFileRO success.
                self.terminal_timeout = True
                self.__terminate_session()
                return False
            dt = now - self.last_burst_read
            self.last_burst_read = now
            if self.ftp_settings.debug > 0:
                logging.info(
                    "FTP: Retry read at %u rtt=%.2f dt=%.2f",
                    self.__read_position(),
                    self.rtt,
                    dt,
                )
            if self.pending_burst_request is not None:
                # Resume at the current high-water mark while preserving the
                # client's reply gate and re-arming the server's burst offset.
                self.pending_burst_request.offset = self.__read_position()
                self.__send(self.pending_burst_request, retry=True)
            self.read_retries += 1

        # see if we can fill gaps
        self.__check_read_send()

        if self.write_list is not None:
            self.__send_more_writes()

        return self.__last_send_time_was_more_than_idle_detection_time_ago(now)

    def __last_send_time_was_more_than_idle_detection_time_ago(
        self, now: float
    ) -> bool:
        return self.last_send_time is not None and now - self.last_send_time > float(
            self.ftp_settings.idle_detection_time
        )

    def __handle_reset_sessions_reply(self, op: FTP_OP, _m) -> MAVFTPReturn:
        """Handle reset sessions reply."""
        if (
            self.pending_reset_seq is not None
            and op.seq == (self.pending_reset_seq + 1) % FTP_SEQ_MODULUS
        ):
            # Ack or Nack, the handshake has been answered; the decoded
            # result below still reports a Nack to the caller
            self.pending_reset_seq = None
        return self.__decode_ftp_ack_and_nack(op)

    def process_ftp_reply(  # pylint: disable=too-many-branches,too-many-locals,too-many-statements,too-many-nested-blocks
        self, operation_name: str, timeout: Optional[float] = None
    ) -> MAVFTPReturn:
        """Execute an FTP operation that requires processing a MAVLink response."""
        start_time = time.time()
        ret = MAVFTPReturn(operation_name, FtpError.Fail)
        default_timeout = timeout is None
        if timeout is None:
            timeout = 5.0
        if operation_name != "TerminateSession":
            self.terminal_timeout = False
        if self.master is None:
            logging.error("FTP: Can't receive reply, no master")
            return MAVFTPReturn(operation_name, FtpError.RemoteReplyTimeout)
        try:
            timeout = float(timeout)
            self.ftp_settings.validate()
        except (TypeError, ValueError) as exc:
            logging.error("Invalid FTP timeout or settings: %s", exc)
            return MAVFTPReturn(operation_name, FtpError.InvalidArguments)
        if timeout < 0 or not math.isfinite(timeout):
            logging.error("Invalid FTP timeout: %s", timeout)
            return MAVFTPReturn(operation_name, FtpError.InvalidArguments)
        if (
            default_timeout
            and self.last_op is not None
            and self.last_op.opcode
            in {
                OP_ListDirectory,
                OP_CreateFile,
                OP_RemoveFile,
                OP_RemoveDirectory,
                OP_Rename,
                OP_CreateDirectory,
            }
        ):
            timeout = max(timeout, self.__initial_request_retry_budget())
        recv_timeout = min(0.1, max(float(self.ftp_settings.retry_time) / 2.0, 0.001))

        # A read operation reports its completion positively: EOF seen
        # with no gaps outstanding (__check_read_finished).  Return as
        # soon as the transfer and its session-terminate handshake are
        # both done instead of waiting out idle_detection_time of link
        # silence - on a fast or simulated link the quiet-period tail
        # dwarfs the transfer itself.  The flag is only ever set while a
        # reply-processing loop runs (this one, or read()'s own driver
        # loop, which does not use this method), so clearing it here
        # cannot lose a completion; idle detection remains the fallback for a lost
        # terminate reply and for operations with no positive
        # completion signal.
        self.read_complete = False
        self.completed_reply = None
        self._ftp_reply_processing_depth += 1
        try:
            while True:  # an FTP operation can have multiple responses
                m = self.master.recv_match(
                    type=["FILE_TRANSFER_PROTOCOL"], timeout=recv_timeout
                )
                if m is not None:
                    # A recipient check does not require decoding the FTP
                    # payload.  Do it first so malformed traffic for another
                    # GCS cannot turn into a local InvalidDataSize result.
                    if (
                        not self.__reply_addressed_to_local_client(m)
                        or not self.__reply_from_configured_target(m)
                    ):
                        self.__receive_packet(m)
                    elif operation_name == "TerminateSession":
                        # The normal packet path validates target, session and
                        # sequence, and also lets a configured RX delay apply to
                        # termination replies. Stale transfer replies are
                        # rejected by __reply_matches_active_request().
                        packet_ret = self.__receive_packet(m)
                        if packet_ret is not None:
                            # Keep the historical terminate-session result for
                            # replies that are not for the pending handshake:
                            # they are ignored and the receive loop eventually
                            # reports a generic failure. Malformed packets still
                            # retain their useful validation error.
                            if self.pending_terminate_seq is None:
                                ret = MAVFTPReturn(operation_name, FtpError.Success)
                            elif packet_ret.error_code == FtpError.InvalidDataSize:
                                ret = MAVFTPReturn(operation_name, packet_ret.error_code)
                            else:
                                ret = MAVFTPReturn(operation_name, FtpError.Fail)
                    else:
                        # Keep a result from the latest request or an active
                        # in-flight request. Packet handlers must still see
                        # stale replies so they can maintain their own state,
                        # but retaining a stale result would make idle fallback
                        # return a previous operation's outcome.
                        try:
                            op = self.__op_parse(m)
                        except (struct.error, TypeError, ValueError) as exc:
                            logging.error(
                                "FTP: malformed FILE_TRANSFER_PROTOCOL payload: %s", exc
                            )
                            ret = MAVFTPReturn(
                                operation_name, FtpError.InvalidDataSize
                            )
                            break
                        reply_matches_last_op = (
                            self.last_op is not None
                            and op.req_opcode == self.last_op.opcode
                            and op.seq == (self.last_op.seq + 1) % FTP_SEQ_MODULUS
                            and op.session == self.session
                        )
                        reply_matches_active_request = (
                            op.session == self.session
                            and self.__reply_matches_active_request(op)
                        )
                        packet_ret = self.__receive_packet(m)
                        # An upload's final CreateFile/WriteFile reply starts a
                        # TerminateSession request before returning here.  Its
                        # result is therefore valid even though last_op is now
                        # the terminate request.
                        completed_upload = (
                            operation_name.lower() == "put"
                            and self.completed_reply is not None
                            and self.completed_reply[0]
                            in {OP_CreateFile, OP_WriteFile}
                        )
                        if packet_ret is not None and (
                            reply_matches_last_op
                            or completed_upload
                            or reply_matches_active_request
                        ):
                            ret = packet_ret
                    if (
                        self.callback_failure is not None
                        and operation_name != "TerminateSession"
                    ):
                        ret = self.callback_failure
                        self.callback_failure = None
                        break
                # Completion is scoped to the terminal reply that produced it.
                # This prevents a delayed ListDirectory EOF from completing a
                # following RemoveFile or a subsequent list operation.
                reply_complete = False
                if self.completed_reply is not None and self.last_op is not None:
                    completed_opcode, completed_seq = self.completed_reply
                    reply_complete = (
                        completed_opcode == self.last_op.opcode
                        and completed_seq == (self.last_op.seq + 1) % FTP_SEQ_MODULUS
                    )
                    # A completed upload sends TerminateSession immediately after
                    # its final CreateFile/WriteFile reply. It is explicitly
                    # scoped to the upload reply type, rather than being a global
                    # latch that any packet handler can set.
                    if (
                        not reply_complete
                        and completed_opcode in {OP_CreateFile, OP_WriteFile}
                        and operation_name.lower() == "put"
                    ):
                        reply_complete = True
                if not reply_complete and self.pending_terminate_seq is None:
                    reply_complete = self.read_complete
                    if not reply_complete:
                        reply_complete = operation_name == "TerminateSession"
                    if not reply_complete and operation_name == "ResetSessions":
                        reply_complete = self.pending_reset_seq is None
                if reply_complete:
                    break
                idle_expired = self.idle_task()
                delayed_reply_accepted = False
                delayed_reply_matches_last_op = False
                malformed_result = False
                delayed_results = self.delayed_rx_results
                self.delayed_rx_results = []
                for (
                    packet_ret,
                    reply_matches_last_op,
                    reply_matches_active_request,
                    malformed,
                    addressed_to_us,
                ) in delayed_results:
                    if malformed:
                        ret = MAVFTPReturn(operation_name, FtpError.InvalidDataSize)
                        malformed_result = True
                        break
                    if operation_name == "TerminateSession":
                        if not addressed_to_us:
                            continue
                        if self.pending_terminate_seq is not None and not (
                            reply_matches_last_op or reply_matches_active_request
                        ):
                            continue
                        if self.pending_terminate_seq is None:
                            ret = MAVFTPReturn(operation_name, FtpError.Success)
                        elif packet_ret.error_code == FtpError.InvalidDataSize:
                            ret = MAVFTPReturn(operation_name, packet_ret.error_code)
                        else:
                            ret = MAVFTPReturn(operation_name, FtpError.Fail)
                        delayed_reply_accepted = True
                        continue
                    completed_upload = (
                        operation_name.lower() == "put"
                        and self.completed_reply is not None
                        and self.completed_reply[0] in {OP_CreateFile, OP_WriteFile}
                        and addressed_to_us
                    )
                    if (
                        reply_matches_last_op
                        or completed_upload
                        or reply_matches_active_request
                    ):
                        ret = packet_ret
                        delayed_reply_accepted = True
                        delayed_reply_matches_last_op |= reply_matches_last_op
                if (
                    self.callback_failure is not None
                    and operation_name != "TerminateSession"
                ):
                    ret = self.callback_failure
                    self.callback_failure = None
                    break
                if malformed_result:
                    break
                if (
                    operation_name != "TerminateSession"
                    and self.terminal_timeout
                    and not delayed_reply_accepted
                ):
                    ret = MAVFTPReturn(operation_name, FtpError.RemoteReplyTimeout)
                    self.terminal_timeout = False
                    break
                # A matching reply belongs to the request which just timed
                # out, even when RX simulation releases it in the same idle
                # pass. Do not report a timeout after its side effects have
                # already been applied. Replies for another active part of a
                # transfer deliberately cannot clear this latch.
                if delayed_reply_matches_last_op:
                    self.terminal_timeout = False
                if idle_expired and not delayed_reply_accepted:
                    if self.last_burst_read is not None and not self.read_complete:
                        ret = MAVFTPReturn(operation_name, FtpError.RemoteReplyTimeout)
                    break
                if timeout > 0 and time.time() - start_time > timeout:  # pylint: disable=chained-comparison
                    logging.error(
                        "FTP: timed out after %f seconds", time.time() - start_time
                    )
                    ret = MAVFTPReturn(operation_name, FtpError.RemoteReplyTimeout)
                    break
        finally:
            self._ftp_reply_processing_depth -= 1
        if (
            ret.error_code != FtpError.Success
            and operation_name != "TerminateSession"
            and self.__has_active_session()
        ):
            self.__terminate_session()
        return ret

    def __decode_ftp_ack_and_nack(
        self, op: FTP_OP, operation_name: str = ""
    ) -> MAVFTPReturn:
        """Decode FTP Acknowledge reply."""
        system_error = 0
        invalid_error_code = 0
        operation_name_dict = {
            OP_None: "None",
            OP_TerminateSession: "TerminateSession",
            OP_ResetSessions: "ResetSessions",
            OP_ListDirectory: "ListDirectory",
            OP_ListDirectoryWithTime: "ListDirectoryWithTime",
            OP_OpenFileRO: "OpenFileRO",
            OP_ReadFile: "ReadFile",
            OP_CreateFile: "CreateFile",
            OP_WriteFile: "WriteFile",
            OP_RemoveFile: "RemoveFile",
            OP_CreateDirectory: "CreateDirectory",
            OP_RemoveDirectory: "RemoveDirectory",
            OP_OpenFileWO: "OpenFileWO",
            OP_TruncateFile: "TruncateFile",
            OP_Rename: "Rename",
            OP_CalcFileCRC32: "CalcFileCRC32",
            OP_BurstReadFile: "BurstReadFile",
        }
        op_ret_name = operation_name or operation_name_dict.get(
            op.req_opcode, "Unknown"
        )
        len_payload = len(op.payload) if op.payload is not None else 0
        if op.opcode == OP_Ack:
            error_code = FtpError.Success
        elif op.opcode == OP_Nack:
            if len_payload <= 0:
                error_code = FtpError.NoErrorCodeInPayload
            elif op.payload is not None and len_payload == 1:
                try:
                    error_code = FtpError(op.payload[0])
                except ValueError:
                    error_code = op.payload[0]  # type: ignore[assignment]
                if error_code == FtpError.Success:
                    error_code = FtpError.NoErrorCodeInNack
                elif error_code == FtpError.FailErrno:
                    error_code = FtpError.NoFilesystemErrorInPayload
                elif error_code not in [
                    FtpError.Fail,
                    FtpError.InvalidDataSize,
                    FtpError.InvalidSession,
                    FtpError.NoSessionsAvailable,
                    FtpError.EndOfFile,
                    FtpError.UnknownCommand,
                    FtpError.FileExists,
                    FtpError.FileProtected,
                    FtpError.FileNotFound,
                ]:
                    invalid_error_code = error_code
                    error_code = FtpError.InvalidErrorCode
            elif (
                op.payload is not None
                and op.payload[0] == FtpError.FailErrno
                and len_payload == 2
            ):
                system_error = op.payload[1]
                error_code = FtpError.FailErrno
            else:
                error_code = FtpError.PayloadTooLarge
        else:
            error_code = FtpError.InvalidOpcode
        return MAVFTPReturn(
            op_ret_name,
            error_code,
            system_error=system_error,
            invalid_error_code=invalid_error_code,
            invalid_payload_size=len_payload,
            invalid_opcode=op.opcode,
        )

    @staticmethod
    def ftp_param_decode(data: bytes) -> Union[None, ParamData]:  # pylint: disable=too-many-locals,too-many-statements,too-many-branches,too-many-return-statements
        """Decode parameter data, returning ParamData."""
        pdata = ParamData()

        magic = 0x671B
        magic_defaults = 0x671C
        if len(data) < 6:
            logging.error(
                "paramftp: Not enough data do decode, only %u bytes", len(data)
            )
            return None
        magic2, num_params, total_params = struct.unpack("<HHH", data[0:6])
        if magic2 not in {magic, magic_defaults}:
            logging.error("paramftp: bad magic 0x%x expected 0x%x", magic2, magic)
            return None
        if num_params > total_params:
            logging.error(
                "paramftp: parameter count %u exceeds total count %u",
                num_params,
                total_params,
            )
            return None
        with_defaults = magic2 == magic_defaults

        # mapping of data type to type length and format
        data_types = {
            1: (1, "b"),
            2: (2, "h"),
            3: (4, "i"),
            4: (4, "f"),
        }

        count = 0
        pad_byte = 0
        last_name = b""
        offset = 6
        data_length = len(data)
        while True:
            while offset < data_length and data[offset] == pad_byte:
                offset += 1  # skip pad bytes

            if offset == data_length:
                break
            if data_length - offset < 2:
                logging.error("paramftp: truncated parameter header")
                return None

            ptype, plen = struct.unpack_from("<BB", data, offset)
            flags = (ptype >> 4) & 0x0F
            has_default = with_defaults and (flags & 1) != 0
            ptype &= 0x0F

            if ptype not in data_types:
                logging.error("paramftp: bad type 0x%x", ptype)
                return None

            (type_len, type_format) = data_types[ptype]
            default_len = type_len if has_default else 0

            name_len = ((plen >> 4) & 0x0F) + 1
            common_len = plen & 0x0F
            value_len = type_len + default_len
            record_len = 2 + name_len + value_len
            record_end = offset + record_len
            if record_end > data_length:
                logging.error("paramftp: truncated parameter record")
                return None
            if common_len > len(last_name):
                logging.error(
                    "paramftp: invalid shared parameter name prefix length %u",
                    common_len,
                )
                return None
            name_start = offset + 2
            value_start = name_start + name_len
            name = last_name[0:common_len] + data[name_start:value_start]
            if len(name) > 16:
                logging.error(
                    "paramftp: parameter name is too long (%u bytes)", len(name)
                )
                return None
            try:
                name.decode("utf-8")
            except UnicodeDecodeError:
                logging.error("paramftp: parameter name is not valid UTF-8")
                return None
            last_name = name
            if with_defaults:
                if has_default:
                    v1, v2 = struct.unpack_from("<" + type_format + type_format, data, value_start)
                    pdata.add_param(name, v1, ptype)
                    pdata.add_default(name, v2, ptype)
                else:
                    (v,) = struct.unpack_from("<" + type_format, data, value_start)
                    pdata.add_param(name, v, ptype)
                    pdata.add_default(name, v, ptype)
            else:
                (v,) = struct.unpack_from("<" + type_format, data, value_start)
                pdata.add_param(name, v, ptype)
            count += 1
            offset = record_end

        if count != num_params:
            logging.error("paramftp: bad count %u should be %u", count, num_params)
            return None

        return pdata

    @staticmethod
    def missionplanner_sort(item: str) -> Tuple[str, ...]:
        """Sorts a parameter name according to the rules defined in the Mission Planner software."""
        return tuple(item.split("_"))

    @staticmethod
    def extract_params(
        pdata: List[Tuple[bytes, float, type]], sort_type: str
    ) -> Dict[str, Tuple[float, type]]:
        """Extract parameter values to an optionally sorted dictionary of name->(value, type)."""
        pdict = {}
        if pdata:
            for name, value, ptype in pdata:
                pdict[name.decode("utf-8")] = (value, ptype)

            if sort_type == "missionplanner":
                pdict = dict(
                    sorted(
                        pdict.items(), key=lambda x: MAVFTP.missionplanner_sort(x[0])
                    )
                )  # sort alphabetically
            elif sort_type == "mavproxy":
                pdict = dict(sorted(pdict.items()))  # sort in ASCIIbetical order
            elif sort_type == "none":
                pass
        return pdict

    @staticmethod
    def save_params(
        pdict: Dict[str, Tuple[float, ParameterDataType]],
        filename: str,
        sort_type: str,
        add_datatype_comments: bool,
        add_timestamp_comment: bool,
    ) -> None:
        """Save Ardupilot parameter information to a local file."""
        if not pdict or not filename:
            return
        with open(filename, "w", encoding="utf-8") as f:
            parameter_data_types = {
                1: "8-bit",
                2: "16-bit",
                3: "32-bit integer",
                4: "32-bit float",
            }
            if add_timestamp_comment:
                f.write(
                    f"# Parameters saved at {datetime.now(tz=None).strftime('%Y-%m-%d %H:%M:%S')}\n"
                )
            for name, (value, datatype) in pdict.items():
                if sort_type == "missionplanner":
                    f.write(f"{name},{format(value, '.6f').rstrip('0').rstrip('.')}")
                elif sort_type in {"mavproxy", "none"}:
                    f.write(f"{name:<16} {value:<8.6f}")

                if add_datatype_comments:
                    datatype_id = int(datatype)
                    f.write(f"  # {parameter_data_types[datatype_id]}")
                f.write("\n")
        logging.info("Outputted %u parameters to %s", len(pdict), filename)

    def cmd_getparams(  # pylint: disable=too-many-arguments
        self,
        args: List[str],
        progress_callback=None,
        sort_type: str = "missionplanner",
        add_datatype_comments: bool = False,
        add_timestamp_comment: bool = False,
    ) -> MAVFTPReturn:
        """Decode the parameter file and save the values and defaults to disk."""

        def decode_and_save_params(fh) -> MAVFTPReturn:
            if fh is None:
                logging.error("FTP: no parameter file handler")
                return MAVFTPReturn("GetParams", FtpError.Fail)
            try:
                data = fh.read()
            except OSError as exp:
                logging.error("FTP: Failed to read file param.pck: %s", exp)
                return MAVFTPReturn("GetParams", FtpError.Fail)
            pdata = MAVFTP.ftp_param_decode(data)
            if pdata is None:
                logging.error("FTP: Failed to decode parameter file param.pck")
                return MAVFTPReturn("GetParams", FtpError.Fail)

            param_values = MAVFTP.extract_params(pdata.params, sort_type)
            param_defaults = MAVFTP.extract_params(pdata.defaults, sort_type)

            param_values_path = args[0]
            param_defaults_path = args[1] if len(args) > 1 else ""

            MAVFTP.save_params(
                param_values,
                param_values_path,
                sort_type,
                add_datatype_comments,
                add_timestamp_comment,
            )
            MAVFTP.save_params(
                param_defaults,
                param_defaults_path,
                sort_type,
                add_datatype_comments,
                add_timestamp_comment,
            )

            if self.ftp_settings.debug > 0:
                for name, (value, _param_type) in param_values.items():
                    if name in param_defaults:
                        logging.info(
                            "%-16s %f (default %f)",
                            name,
                            value,
                            param_defaults[name][0],
                        )
                    else:
                        logging.info("%-16s %f", name, value)
            return MAVFTPReturn("GetParams", FtpError.Success)

        return self.cmd_get(
            [
                "@PARAM/param.pck?withdefaults=1"
                if len(args) > 1
                else "@PARAM/param.pck"
            ],
            callback=decode_and_save_params,
            progress_callback=progress_callback,
        )


# ------------------------------------------------------------
# These functions are to use this script as a standalone tool
# ------------------------------------------------------------


def create_argument_parser() -> ArgumentParser:  # pylint: disable=too-many-statements
    """
    Parses command-line arguments for the script.

    This function sets up an argument parser to handle the command-line arguments for the script.
    """
    parser = ArgumentParser(
        description="MAVFTP - MAVLink File Transfer Protocol https://mavlink.io/en/services/ftp.html"
        " A tool to do file operations between a ground control station and a drone using the MAVLink"
        " protocol."
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="master port baud rate. Default is %(default)s",
    )
    parser.add_argument(  # type: ignore[attr-defined]
        "--device",
        type=str,
        default="",
        help="serial device. For windows use COMx where x is the port number. "
        "For Unix use /dev/ttyUSBx where x is the port number. Default is autodetection",
    ).completer = FilesCompleter(directories=False, allowednames=[".port"])  # type: ignore[attr-defined, no-untyped-call]
    parser.add_argument(
        "--source-system",
        type=int,
        default=250,
        help="MAVLink source system for this GCS. Default is %(default)s",
    )
    parser.add_argument(
        "--loglevel", default="INFO", help="log level. Default is %(default)s"
    )

    # MAVFTP settings
    parser.add_argument(
        "--debug",
        type=int,
        default=0,
        choices=[0, 1, 2],
        help="Debug level 0 for none, 2 for max verbosity. Default is %(default)s",
    )
    parser.add_argument(
        "--pkt_loss_tx",
        type=int,
        default=0,
        help="Packet loss on TX. Default is %(default)s",
    )
    parser.add_argument(
        "--pkt_loss_rx",
        type=int,
        default=0,
        help="Packet loss on RX. Default is %(default)s",
    )
    parser.add_argument(
        "--pkt_lag_tx",
        type=float,
        default=0.0,
        help="One-way TX lag in milliseconds. Default is %(default)s",
    )
    parser.add_argument(
        "--pkt_lag_rx",
        type=float,
        default=0.0,
        help="One-way RX lag in milliseconds. Default is %(default)s",
    )
    parser.add_argument(
        "--pkt_lag_jitter_tx",
        type=float,
        default=0.0,
        help="Uniform extra TX lag in milliseconds. Default is %(default)s",
    )
    parser.add_argument(
        "--pkt_lag_jitter_rx",
        type=float,
        default=0.0,
        help="Uniform extra RX lag in milliseconds. Default is %(default)s",
    )
    parser.add_argument(
        "--loss_seed",
        type=int,
        default=0,
        help="Seed for repeatable loss/jitter simulation. Default is %(default)s",
    )
    parser.add_argument(
        "--list_time",
        type=int,
        default=1,
        choices=[0, 1],
        help="Request directory modification times. Default is %(default)s",
    )
    parser.add_argument(
        "--list_time_timeout",
        type=float,
        default=3.0,
        help="Seconds before retrying a timestamp listing. Default is %(default)s",
    )
    parser.add_argument(
        "--list_retries",
        type=int,
        default=3,
        help="Timestamp-listing retries before compatibility fallback. Default is %(default)s",
    )
    parser.add_argument(
        "--max_backlog", type=int, default=5, help="Max backlog. Default is %(default)s"
    )
    parser.add_argument(
        "--burst_read_size",
        type=int,
        default=80,
        help="Burst read size. Default is %(default)s",
    )
    parser.add_argument(
        "--write_size", type=int, default=80, help="Write size. Default is %(default)s"
    )
    parser.add_argument(
        "--write_qsize",
        type=int,
        default=5,
        help="Write queue size. Default is %(default)s",
    )
    parser.add_argument(
        "--idle_detection_time",
        type=float,
        default=3.7,
        help="Idle detection time. Default is %(default)s",
    )
    parser.add_argument(
        "--read_retry_time",
        type=float,
        default=1.0,
        help="Read retry time. Default is %(default)s",
    )
    parser.add_argument(
        "--retry_time",
        type=float,
        default=0.5,
        help="Retry time. Default is %(default)s",
    )
    parser.add_argument(
        "--crccmp_timeout",
        type=float,
        default=120.0,
        help="Seconds allowed for the whole CRC comparison batch. Default is %(default)s",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    # Set command
    parser_set = subparsers.add_parser(
        "set", help="Set a MAVFTP internal configuration parameter."
    )
    parser_set.add_argument(
        "arg1",
        type=str,
        metavar="setting_name",
        help="MAVFTP internal configuration parameter name.",
    )
    parser_set.add_argument(
        "arg2",
        type=float,
        metavar="setting_value",
        help="MAVFTP internal configuration parameter value.",
    )

    # Get command
    parser_get = subparsers.add_parser(
        "get", help="Get a file from the remote flight controller."
    )
    parser_get.add_argument(
        "arg1",
        type=str,
        metavar="remote_path",
        help="Path to the file on the remote flight controller.",
    )
    parser_get.add_argument(  # type: ignore[attr-defined]
        "arg2",
        nargs="?",
        type=str,
        metavar="local_path",
        help="Optional local path to save the file.",
    ).completer = FilesCompleter()  # type: ignore[attr-defined, no-untyped-call]

    # Getparams command
    parser_getparams = subparsers.add_parser(
        "getparams", help="Get and decode parameters from the remote flight controller."
    )
    parser_getparams.add_argument(  # type: ignore[attr-defined]
        "arg1",
        type=str,
        metavar="param_values_path",
        help="Local path to save the parameter values file to.",
    ).completer = FilesCompleter()  # type: ignore[attr-defined, no-untyped-call]
    parser_getparams.add_argument(  # type: ignore[attr-defined]
        "arg2",
        nargs="?",
        type=str,
        metavar="param_defaults_path",
        help="Optional local path to save the parameter defaults file to.",
    ).completer = FilesCompleter()  # type: ignore[attr-defined, no-untyped-call]
    parser_getparams.add_argument(
        "-s",
        "--sort",
        choices=["none", "missionplanner", "mavproxy"],
        default="missionplanner",
        help="Sort the parameters in the file. Default is %(default)s.",
    )
    parser_getparams.add_argument(
        "-dtc",
        "--add_datatype_comments",
        action="store_true",
        default=False,
        help="Add parameter datatype type comments to the outputted parameter files. Default is %(default)s.",
    )
    parser_getparams.add_argument(
        "-t",
        "--add_timestamp_comment",
        action="store_true",
        default=False,
        help="Add timestamp comment at the top of the file. Default is %(default)s.",
    )

    # Put command
    parser_put = subparsers.add_parser(
        "put", help="Put a file to the remote flight controller."
    )
    parser_put.add_argument(  # type: ignore[attr-defined]
        "arg1",
        type=str,
        metavar="local_path",
        help="Local path to the file to upload to the flight controller.",
    ).completer = FilesCompleter()  # type: ignore[no-untyped-call]
    parser_put.add_argument(
        "arg2",
        nargs="?",
        type=str,
        metavar="remote_path",
        help="Optional remote path where the file should be uploaded on the remote flight controller.",
    )

    # List command
    parser_list = subparsers.add_parser(
        "list", help="List files in a directory on the remote flight controller."
    )
    parser_list.add_argument(
        "arg1",
        nargs="?",
        type=str,
        metavar="remote_path",
        help="Optional path to list files from.",
    )

    # Mkdir command
    parser_mkdir = subparsers.add_parser(
        "mkdir", help="Create a directory on the remote flight controller."
    )
    parser_mkdir.add_argument(
        "arg1", type=str, metavar="remote_path", help="Path to the directory to create."
    )

    # Rmdir command
    parser_rmdir = subparsers.add_parser(
        "rmdir", help="Remove a directory on the remote flight controller."
    )
    parser_rmdir.add_argument(
        "arg1", type=str, metavar="remote_path", help="Path to the directory to remove."
    )

    # Rm command
    parser_rm = subparsers.add_parser(
        "rm", help="Remove a file on the remote flight controller."
    )
    parser_rm.add_argument(
        "arg1", type=str, metavar="remote_path", help="Path to the file to remove."
    )

    # Rename command
    parser_rename = subparsers.add_parser(
        "rename", help="Rename a file or directory on the remote flight controller."
    )
    parser_rename.add_argument(
        "arg1",
        type=str,
        metavar="old_remote_path",
        help="Current path of the file/directory.",
    )
    parser_rename.add_argument(
        "arg2",
        type=str,
        metavar="new_remote_path",
        help="New path for the file/directory.",
    )

    # CRC command
    parser_crc = subparsers.add_parser(
        "crc", help="Calculate the CRC of a file on the remote flight controller."
    )
    parser_crc.add_argument(
        "arg1",
        type=str,
        metavar="remote_path",
        help="Path to the file to calculate the CRC of.",
    )

    # Local CRC command
    parser_crclocal = subparsers.add_parser(
        "crclocal", help="Calculate the vehicle-compatible CRC of a local file."
    )
    parser_crclocal.add_argument(
        "arg1", type=str, metavar="local_path", help="Path to the local file."
    ).completer = FilesCompleter()  # type: ignore[attr-defined, no-untyped-call]

    # CRC comparison command
    parser_crccmp = subparsers.add_parser(
        "crccmp",
        help="Compare local files with same-named files on the remote vehicle.",
    )
    parser_crccmp.add_argument(
        "arg1",
        type=str,
        metavar="wildcard",
        help="Local wildcard selecting files to compare.",
    ).completer = FilesCompleter()  # type: ignore[attr-defined, no-untyped-call]
    parser_crccmp.add_argument(
        "arg2",
        type=str,
        metavar="remote_directory",
        help="Remote directory containing the files.",
    )

    # Add other subparsers commands as needed
    if _ARGCOMPLETE_AVAILABLE:
        argcomplete.autocomplete(parser)
    return parser


def auto_detect_serial() -> List[mavutil.SerialPort]:
    preferred_ports = [
        "*FTDI*",
        "*3D*",
        "*USB_to_UART*",
        "*Ardu*",
        "*PX4*",
        "*Hex_*",
        "*ProfiCNC*",
        "*Holybro_*",
        "*mRo*",
        "*FMU*",
        "*Swift-Flyer*",
        "*Serial*",
        "*CubePilot*",
        "*Qiotek*",
        "*Matek*",
    ]
    serial_list: List[mavutil.SerialPort] = mavutil.auto_detect_serial(
        preferred_list=preferred_ports
    )
    serial_list.sort(key=lambda x: x.device)

    # remove OTG2 ports for dual CDC
    if (
        len(serial_list) == 2
        and serial_list[0].device.startswith("/dev/serial/by-id")
        and serial_list[0].device[:-1] == serial_list[1].device[0:-1]
    ):
        serial_list.pop(1)

    return serial_list


def auto_connect(device) -> mavutil.SerialPort:
    comport = None
    if device:
        comport = mavutil.SerialPort(device=device, description=device)
    else:
        autodetect_serial = auto_detect_serial()
        if autodetect_serial:
            # Resolve the soft link if it's a Linux system
            if os.name == "posix":
                try:
                    dev = autodetect_serial[0].device
                    logging.debug("Auto-detected device %s", dev)
                    # Get the directory part of the soft link
                    softlink_dir = os.path.dirname(dev)
                    # Resolve the soft link and join it with the directory part
                    resolved_path = os.path.abspath(
                        os.path.join(softlink_dir, os.readlink(dev))
                    )
                    autodetect_serial[0].device = resolved_path
                    logging.debug("Resolved soft link %s to %s", dev, resolved_path)
                except OSError:
                    pass  # Not a soft link, proceed with the original device path
            comport = autodetect_serial[0]
        else:
            logging.error(
                "No serial ports found. Please connect a flight controller and try again."
            )
            sys.exit(1)
    return comport


def wait_heartbeat(m) -> None:
    """Wait for a heartbeat so we know the target system IDs."""
    logging.info("Waiting for flight controller heartbeat")
    m.wait_heartbeat(timeout=5)
    logging.info(
        "Heartbeat from system %u, component %u", m.target_system, m.target_component
    )


def main() -> None:  # pylint: disable=too-many-branches
    """For testing/example purposes only."""
    parser = create_argument_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.getLevelName(args.loglevel), format="%(levelname)s - %(message)s"
    )

    exit_code = 1
    master = None
    mav_ftp = None
    try:
        # create a mavlink serial instance
        comport = auto_connect(args.device)
        master = mavutil.mavlink_connection(
            comport.device, baud=args.baudrate, source_system=args.source_system
        )

        # wait for the heartbeat msg to find the system ID
        wait_heartbeat(master)

        try:
            ftp_settings = MAVFTPSettings(
                [
                    ("debug", int, args.debug),
                    ("list_time", int, args.list_time),
                    ("list_time_timeout", float, args.list_time_timeout),
                    ("list_retries", int, args.list_retries),
                    ("pkt_loss_tx", int, args.pkt_loss_tx),
                    ("pkt_loss_rx", int, args.pkt_loss_rx),
                    ("pkt_lag_tx", float, getattr(args, "pkt_lag_tx", 0.0)),
                    ("pkt_lag_rx", float, getattr(args, "pkt_lag_rx", 0.0)),
                    ("pkt_lag_jitter_tx", float, getattr(args, "pkt_lag_jitter_tx", 0.0)),
                    ("pkt_lag_jitter_rx", float, getattr(args, "pkt_lag_jitter_rx", 0.0)),
                    ("loss_seed", int, getattr(args, "loss_seed", 0)),
                    ("max_backlog", int, args.max_backlog),
                    ("burst_read_size", int, args.burst_read_size),
                    ("write_size", int, args.write_size),
                    ("write_qsize", int, args.write_qsize),
                    ("idle_detection_time", float, args.idle_detection_time),
                    ("read_retry_time", float, args.read_retry_time),
                    ("retry_time", float, args.retry_time),
                    ("crccmp_timeout", float, getattr(args, "crccmp_timeout", 120.0)),
                ]
            )
        except (TypeError, ValueError, OverflowError) as exc:
            parser.error(str(exc))

        mav_ftp = MAVFTP(
            master,
            target_system=master.target_system,
            target_component=master.target_component,
            settings=ftp_settings,
        )

        cmd_ftp_args = [args.command]
        if "arg1" in args and args.arg1:
            cmd_ftp_args.append(args.arg1)
        if "arg2" in args and args.arg2:
            cmd_ftp_args.append(args.arg2)

        if args.command == "getparams":
            ret = mav_ftp.cmd_getparams(
                cmd_ftp_args[1:],
                sort_type=args.sort,
                add_datatype_comments=args.add_datatype_comments,
                add_timestamp_comment=args.add_timestamp_comment,
            )
        else:
            ret = mav_ftp.cmd_ftp(cmd_ftp_args)

        if args.command in {"get", "put", "getparams"}:
            ret = mav_ftp.process_ftp_reply(args.command, timeout=500)

        if isinstance(ret, str):
            logging.error(
                "Command returned: %s, but it should return a MAVFTPReturn instead",
                ret,
            )
        elif isinstance(ret, MAVFTPReturn):
            if ret.error_code or args.command in {"list"}:
                ret.display_message()
            exit_code = 0 if ret.error_code == FtpError.Success else 1
        elif ret is None:
            logging.error(
                "Command returned: None, but it should return a MAVFTPReturn instead"
            )
        else:
            logging.error(
                "Command returned: something strange, but it should return a MAVFTPReturn instead"
            )
    finally:
        if mav_ftp is not None:
            try:
                has_active_session = getattr(mav_ftp, "_MAVFTP__has_active_session", None)
                if has_active_session is None:
                    logging.warning("Unable to check MAVFTP session during cleanup")
                elif has_active_session():
                    mav_ftp.cmd_cancel()
            except Exception as exc:  # pylint: disable=broad-exception-caught
                logging.warning("MAVFTP cleanup failed: %s", exc)
        if master is not None:
            with contextlib.suppress(Exception):
                master.close()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
