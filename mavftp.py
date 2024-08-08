#!/usr/bin/env python3

'''
MAVLink File Transfer Protocol support
Original from MAVProxy/MAVProxy/modules/mavproxy_ftp.py

SPDX-FileCopyrightText: 2011-2024 Andrew Tridgell, 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
'''

from argparse import ArgumentParser

from logging import basicConfig as logging_basicConfig
from logging import getLevelName as logging_getLevelName

from logging import debug as logging_debug
from logging import info as logging_info
from logging import warning as logging_warning
from logging import error as logging_error

import struct
from time import time as time_time
from random import uniform as random_uniform
from os import path as os_path

from io import BytesIO as SIO

import sys

from pymavlink import mavutil


# pylint: disable=invalid-name
# opcodes
OP_None = 0
OP_TerminateSession = 1
OP_ResetSessions = 2
OP_ListDirectory = 3
OP_OpenFileRO = 4
OP_ReadFile = 5
OP_CreateFile = 6
OP_WriteFile = 7
OP_RemoveFile = 8
OP_CreateDirectory = 9
OP_RemoveDirectory = 10
OP_OpenFileWO = 11
OP_TruncateFile = 12
OP_Rename = 13
OP_CalcFileCRC32 = 14
OP_BurstReadFile = 15
OP_Ack = 128
OP_Nack = 129

# error codes
ERR_None = 0
ERR_Fail = 1
ERR_FailErrno = 2
ERR_InvalidDataSize = 3
ERR_InvalidSession = 4
ERR_NoSessionsAvailable = 5
ERR_EndOfFile = 6
ERR_UnknownCommand = 7
ERR_FileExists = 8
ERR_FileProtected = 9
ERR_FileNotFound = 10

HDR_Len = 12
MAX_Payload = 239
# pylint: enable=invalid-name

class FTP_OP:  # pylint: disable=invalid-name, too-many-instance-attributes
    """
    Represents an operation in the MAVLink File Transfer Protocol (FTP).

    This class encapsulates the details of a single FTP operation such as reading, writing, listing directories, etc.,
    including the necessary parameters and payload for the operation.
    """
    def __init__(self, seq, session, opcode, size,  # pylint: disable=too-many-arguments
                 req_opcode, burst_complete, offset, payload):
        self.seq = seq                        # Sequence number for the operation.
        self.session = session                # Session identifier.
        self.opcode = opcode                  # Operation code indicating the type of FTP operation.
        self.size = size                      # Size of the operation.
        self.req_opcode = req_opcode          # Request operation code.
        self.burst_complete = burst_complete  # (bool) Flag indicating if the burst transfer is complete.
        self.offset = offset                  # Offset for read/write operations.
        self.payload = payload                # (bytes) Payload for the operation.

    def pack(self):
        '''pack message'''
        ret = struct.pack("<HBBBBBBI", self.seq, self.session, self.opcode, self.size, self.req_opcode, self.burst_complete,
                          0, self.offset)
        if self.payload is not None:
            ret += self.payload
        ret = bytearray(ret)
        return ret

    def __str__(self):
        plen = 0
        if self.payload is not None:
            plen = len(self.payload)
        ret = f"OP seq:{self.seq} sess:{self.session} opcode:{self.opcode} req_opcode:{self.req_opcode}" \
              f" size:{self.size} bc:{self.burst_complete} ofs:{self.offset} plen={plen}"
        if plen > 0:
            ret += f" [{self.payload[0]}]"
        return ret


class WriteQueue:  # pylint: disable=too-few-public-methods
    """
    Manages a queue of write operations for the MAVFTP class.

    Keeps track of offsets and sizes for pending write operations to ensure orderly processing.
    """
    def __init__(self, ofs: int, size: int):
        self.ofs = ofs      # Offset where the write operation starts.
        self.size = size    # Size of the data to be written.
        self.last_send = 0  # Timestamp of the last send operation.


class ParamData():
    """
    A class to manage parameter values and defaults for ArduPilot configuration.
    """
    def __init__(self):
        self.params = []     # params as (name, value, ptype)
        self.defaults = None # defaults as (name, value, ptype)

    def add_param(self, name, value, ptype):
        self.params.append((name, value, ptype))

    def add_default(self, name, value, ptype):
        if self.defaults is None:
            self.defaults = []
        self.defaults.append((name, value, ptype))


class MAVFTP:  # pylint: disable=too-many-instance-attributes
    """
    Implements the client-side logic for the MAVLink File Transfer Protocol (FTP) over MAVLink connections.

    Handles file operations such as reading, writing, listing directories, and managing sessions.
    """
    def __init__(self, master, target_system, target_component):
        self.ftp_settings_debug = 0
        self.ftp_settings_pkt_loss_rx = 0
        self.ftp_settings_pkt_loss_tx = 0
        self.ftp_settings_burst_read_size = 80
        self.ftp_settings_max_backlog = 5
        self.ftp_settings_write_size = 80
        self.ftp_settings_write_qsize = 5
        self.ftp_settings_retry_time = 0.5
        self.seq = 0
        self.session = 0
        self.network = 0
        self.last_op = None
        self.fh = None
        self.filename = None
        self.callback = None
        self.callback_progress = None
        self.put_callback = None
        self.put_callback_progress = None
        self.total_size = 0
        self.read_gaps = []
        self.read_gap_times = {}
        self.last_gap_send = 0
        self.read_retries = 0
        self.read_total = 0
        self.duplicates = 0
        self.last_read = None
        self.last_burst_read = None
        self.op_start = None
        self.dir_offset = 0
        self.last_op_time = time_time()
        self.rtt = 0.5
        self.reached_eof = False
        self.backlog = 0
        self.burst_size = self.ftp_settings_burst_read_size
        self.write_list = None
        self.write_block_size = 0
        self.write_acks = 0
        self.write_total = 0
        self.write_file_size = 0
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        self.write_last_send = None
        self.open_retries = 0

        self.master = master
        self.target_system = target_system
        self.target_component = target_component
        self.op_pending = False

    def __send(self, op):
        '''send a request'''
        op.seq = self.seq
        payload = op.pack()
        plen = len(payload)
        if plen < MAX_Payload + HDR_Len:
            payload.extend(bytearray([0]*((HDR_Len+MAX_Payload)-plen)))
        self.master.mav.file_transfer_protocol_send(self.network, self.target_system, self.target_component, payload)
        self.seq = (self.seq + 1) % 256
        self.last_op = op
        now = time_time()
        if self.ftp_settings_debug > 1:
            logging_info("> %s dt=%.2f", op, now - self.last_op_time)
        self.last_op_time = time_time()
        self.op_pending = True

    def __terminate_session(self):
        '''terminate current session'''
        self.__send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None))
        self.fh = None
        self.filename = None
        self.write_list = None
        if self.callback is not None:
            # tell caller that the transfer failed
            self.callback(None)
            self.callback = None
        if self.callback_progress is not None:
            self.callback_progress(None)
            self.callback_progress = None
        if self.put_callback is not None:
            # tell caller that the transfer failed
            self.put_callback(None)
            self.put_callback = None
        if self.put_callback_progress is not None:
            self.put_callback_progress(None)
            self.put_callback_progress = None
        self.read_gaps = []
        self.read_total = 0
        self.read_gap_times = {}
        self.last_read = None
        self.last_burst_read = None
        self.session = (self.session + 1) % 256
        self.reached_eof = False
        self.backlog = 0
        self.duplicates = 0
        if self.ftp_settings_debug > 0:
            logging_info("Terminated session")

    def cmd_list(self, args):
        '''list files'''
        if len(args) > 0:
            dname = args[0]
        else:
            dname = '/'
        logging_info("Listing %s", dname)
        enc_dname = bytearray(dname, 'ascii')
        self.total_size = 0
        self.dir_offset = 0
        op = FTP_OP(self.seq, self.session, OP_ListDirectory, len(enc_dname), 0, 0, self.dir_offset, enc_dname)
        self.__send(op)

    def __handle_list_reply(self, op, _m):
        '''handle OP_ListDirectory reply'''
        if op.opcode == OP_Ack:
            dentries = sorted(op.payload.split(b'\x00'))
            #logging_info(dentries)
            for d in dentries:
                if len(d) == 0:
                    continue
                self.dir_offset += 1
                try:
                    if sys.version_info.major >= 3:
                        d = str(d, 'ascii')
                    else:
                        d = str(d)
                except Exception:  # pylint: disable=broad-exception-caught
                    continue
                if d[0] == 'D':
                    logging_info(" D %s", d[1:])
                elif d[0] == 'F':
                    (name, size) = d[1:].split('\t')
                    size = int(size)
                    self.total_size += size
                    logging_info("   %s\t%u", name, size)
                else:
                    logging_info(d)
            # ask for more
            more = self.last_op
            more.offset = self.dir_offset
            self.__send(more)
        elif op.opcode == OP_Nack and len(op.payload) == 1 and op.payload[0] == ERR_EndOfFile:
            logging_info("Total size %.2f kByte", self.total_size / 1024.0)
            self.total_size = 0
            self.op_pending = False
        else:
            logging_info('LIST: %s', op)

    def cmd_get(self, args, callback=None, callback_progress=None):
        '''get file'''
        if len(args) == 0:
            logging_error("cmd_get usage: [FILENAME <LOCALNAME>]")
            return
        self.__terminate_session()
        fname = args[0]
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os_path.basename(fname)
        if callback is None or self.ftp_settings_debug > 1:
            logging_info("Getting %s from %s", fname, self.filename)
        self.op_start = time_time()
        self.callback = callback
        self.callback_progress = callback_progress
        self.read_retries = 0
        self.duplicates = 0
        self.reached_eof = False
        self.burst_size = self.ftp_settings_burst_read_size
        if self.burst_size < 1:
            self.burst_size = 239
        elif self.burst_size > 239:
            self.burst_size = 239
        enc_fname = bytearray(fname, 'ascii')
        self.open_retries = 0
        op = FTP_OP(self.seq, self.session, OP_OpenFileRO, len(enc_fname), 0, 0, 0, enc_fname)
        self.__send(op)

    def __handle_open_ro_reply(self, op, _m):
        '''handle OP_OpenFileRO reply'''
        if op.opcode == OP_Ack:
            if self.filename is None:
                return
            try:
                if self.callback is not None or self.filename == '-':
                    self.fh = SIO()
                else:
                    self.fh = open(self.filename, 'wb')  # pylint: disable=consider-using-with
            except Exception as ex:  # pylint: disable=broad-except
                logging_info("Failed to open %s: %s", self.filename, ex)
                self.__terminate_session()
                return
            read = FTP_OP(self.seq, self.session, OP_BurstReadFile, self.burst_size, 0, 0, 0, None)
            self.last_burst_read = time_time()
            self.__send(read)
        else:
            if self.callback is None or self.ftp_settings_debug > 0:
                logging_info("ftp open failed")
            self.__terminate_session()

    def __check_read_finished(self):
        '''check if download has completed'''
        logging_debug("check_read_finished: %s %s", self.reached_eof, self.read_gaps)
        if self.reached_eof and len(self.read_gaps) == 0:
            ofs = self.fh.tell()
            dt = time_time() - self.op_start
            rate = (ofs / dt) / 1024.0
            if self.callback is not None:
                self.fh.seek(0)
                self.callback(self.fh)
                self.callback = None
            elif self.filename == "-":
                self.fh.seek(0)
                logging_info(self.fh.read().decode('utf-8'))
            else:
                logging_info("Got %u bytes from %s in %.2fs %.1fkByte/s", ofs, self.filename, dt, rate)
            self.__terminate_session()
            self.op_pending = False
            return True
        return False

    def __write_payload(self, op):
        '''write payload from a read op'''
        self.fh.seek(op.offset)
        self.fh.write(op.payload)
        self.read_total += len(op.payload)
        if self.callback_progress is not None:
            self.callback_progress(self.read_total, self.read_total+1)

    def __handle_burst_read(self, op, _m):  # pylint: disable=too-many-branches, too-many-statements, too-many-return-statements
        '''handle OP_BurstReadFile reply'''
        if self.ftp_settings_pkt_loss_tx > 0:
            if random_uniform(0, 100) < self.ftp_settings_pkt_loss_tx:
                if self.ftp_settings_debug > 0:
                    logging_warning("FTP: dropping TX")
                return
        if self.fh is None or self.filename is None:
            if op.session != self.session:
                # old session
                return
            logging_warning("FTP Unexpected burst read reply")
            logging_info(op)
            return
        self.last_burst_read = time_time()
        size = len(op.payload)
        if size > self.burst_size:
            # this server doesn't handle the burst size argument
            self.burst_size = MAX_Payload
            if self.ftp_settings_debug > 0:
                logging_info("Setting burst size to %u", self.burst_size)
        if op.opcode == OP_Ack and self.fh is not None:
            ofs = self.fh.tell()
            if op.offset < ofs:
                # writing an earlier portion, possibly remove a gap
                gap = (op.offset, len(op.payload))
                if gap in self.read_gaps:
                    self.read_gaps.remove(gap)
                    self.read_gap_times.pop(gap)
                    if self.ftp_settings_debug > 0:
                        logging_info("FTP: removed gap", gap, self.reached_eof, len(self.read_gaps))
                else:
                    if self.ftp_settings_debug > 0:
                        logging_info("FTP: dup read reply at %u of len %u ofs=%u", op.offset, op.size, self.fh.tell())
                    self.duplicates += 1
                    return
                self.__write_payload(op)
                self.fh.seek(ofs)
                if self.__check_read_finished():
                    return
            elif op.offset > ofs:
                # we have a gap
                gap = (ofs, op.offset-ofs)
                max_read = self.burst_size
                while True:
                    if gap[1] <= max_read:
                        self.read_gaps.append(gap)
                        self.read_gap_times[gap] = 0
                        break
                    g = (gap[0], max_read)
                    self.read_gaps.append(g)
                    self.read_gap_times[g] = 0
                    gap = (gap[0] + max_read, gap[1] - max_read)
                self.__write_payload(op)
            else:
                self.__write_payload(op)
            if op.burst_complete:
                if op.size > 0 and op.size < self.burst_size:
                    # a burst complete with non-zero size and less than burst packet size
                    # means EOF
                    if not self.reached_eof and self.ftp_settings_debug > 0:
                        logging_info("EOF at %u with %u gaps t=%.2f", self.fh.tell(),
                                     len(self.read_gaps), time_time() - self.op_start)
                    self.reached_eof = True
                    if self.__check_read_finished():
                        return
                    self.__check_read_send()
                    return
                more = self.last_op
                more.offset = op.offset + op.size
                if self.ftp_settings_debug > 0:
                    logging_info("FTP: burst continue at %u %u", more.offset, self.fh.tell())
                self.__send(more)
        elif op.opcode == OP_Nack:
            ecode = op.payload[0]
            if self.ftp_settings_debug > 0:
                logging_info("FTP: burst nack: %s", op)
            if ecode in (ERR_EndOfFile, 0):
                if not self.reached_eof and op.offset > self.fh.tell():
                    # we lost the last part of the burst
                    if self.ftp_settings_debug > 0:
                        logging_info("burst lost EOF %u %u", self.fh.tell(), op.offset)
                    return
                if not self.reached_eof and self.ftp_settings_debug > 0:
                    logging_info("EOF at %u with %u gaps t=%.2f", self.fh.tell(),
                                 len(self.read_gaps), time_time() - self.op_start)
                self.reached_eof = True
                if self.__check_read_finished():
                    return
                self.__check_read_send()
            elif self.ftp_settings_debug > 0:
                logging_info("FTP: burst Nack (ecode:%u): %s", ecode, op)
        else:
            logging_warning("FTP: burst error: %s", op)

    def __handle_reply_read(self, op, _m):
        '''handle OP_ReadFile reply'''
        if self.fh is None or self.filename is None:
            if self.ftp_settings_debug > 0:
                logging_warning("FTP Unexpected read reply")
                logging_warning(op)
            return
        if self.backlog > 0:
            self.backlog -= 1
        if op.opcode == OP_Ack and self.fh is not None:
            gap = (op.offset, op.size)
            if gap in self.read_gaps:
                self.read_gaps.remove(gap)
                self.read_gap_times.pop(gap)
                ofs = self.fh.tell()
                self.__write_payload(op)
                self.fh.seek(ofs)
                if self.ftp_settings_debug > 0:
                    logging_info("FTP: removed gap", gap, self.reached_eof, len(self.read_gaps))
                if self.__check_read_finished():
                    return
            elif op.size < self.burst_size:
                logging_info("FTP: file size changed to %u", op.offset+op.size)
                self.__terminate_session()
            else:
                self.duplicates += 1
                if self.ftp_settings_debug > 0:
                    logging_info("FTP: no gap read", gap, len(self.read_gaps))
        elif op.opcode == OP_Nack:
            logging_info("Read failed with %u gaps", len(self.read_gaps), str(op))
            self.__terminate_session()
        self.__check_read_send()

    def cmd_put(self, args, fh=None, callback=None, progress_callback=None):
        '''put file'''
        if len(args) == 0:
            logging_error("cmd_put Usage: [FILENAME <REMOTENAME>]")
            return
        if self.write_list is not None:
            logging_error("put already in progress")
            return
        fname = args[0]
        self.fh = fh
        if self.fh is None:
            try:
                self.fh = open(fname, 'rb')  # pylint: disable=consider-using-with
            except Exception as ex:  # pylint: disable=broad-exception-caught
                logging_error("Failed to open %s: %s", fname, ex)
                return
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os_path.basename(fname)
        if self.filename.endswith("/"):
            self.filename += os_path.basename(fname)
        if callback is None:
            logging_info("Putting %s to %s", fname, self.filename)
        self.fh.seek(0,2)
        file_size = self.fh.tell()
        self.fh.seek(0)

        # setup write list
        self.write_block_size = self.ftp_settings_write_size
        self.write_file_size = file_size

        write_blockcount = file_size // self.write_block_size
        if file_size % self.write_block_size != 0:
            write_blockcount += 1

        self.write_list = set(range(write_blockcount))
        self.write_acks = 0
        self.write_total = write_blockcount
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        self.write_last_send = None

        self.put_callback = callback
        self.put_callback_progress = progress_callback
        self.read_retries = 0
        self.op_start = time_time()
        enc_fname = bytearray(self.filename, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateFile, len(enc_fname), 0, 0, 0, enc_fname)
        self.__send(op)

    def __put_finished(self, flen):
        '''finish a put'''
        if self.put_callback_progress:
            self.put_callback_progress(1.0)
            self.put_callback_progress = None
        if self.put_callback is not None:
            self.put_callback(flen)
            self.put_callback = None
        else:
            dt = time_time() - self.op_start
            rate = (flen / dt) / 1024.0
            logging_info("Put %u bytes to %s file in %.2fs %.1fkByte/s", flen, self.filename, dt, rate)

    def __handle_create_file_reply(self, op, _m):
        '''handle OP_CreateFile reply'''
        if self.fh is None:
            self.__terminate_session()
            self.op_pending = False
            return
        if op.opcode == OP_Ack:
            self.__send_more_writes()
        else:
            logging_error("Create failed")
            self.__terminate_session()
            self.op_pending = False

    def __send_more_writes(self):
        '''send some more writes'''
        if len(self.write_list) == 0:
            # all done
            self.__put_finished(self.write_file_size)
            self.__terminate_session()
            self.op_pending = False
            return

        now = time_time()
        if self.write_last_send is not None:
            if now - self.write_last_send > max(min(10*self.rtt, 1),0.2):
                # we seem to have lost a block of replies
                self.write_pending = max(0, self.write_pending-1)

        n = min(self.ftp_settings_write_qsize-self.write_pending, len(self.write_list))
        for _i in range(n):
            # send in round-robin, skipping any that have been acked
            idx = self.write_idx
            while idx not in self.write_list:
                idx = (idx + 1) % self.write_total
            ofs = idx * self.write_block_size
            self.fh.seek(ofs)
            data = self.fh.read(self.write_block_size)
            write = FTP_OP(self.seq, self.session, OP_WriteFile, len(data), 0, 0, ofs, bytearray(data))
            self.__send(write)
            self.write_idx = (idx + 1) % self.write_total
            self.write_pending += 1
            self.write_last_send = now

    def __handle_write_reply(self, op, _m):
        '''handle OP_WriteFile reply'''
        if self.fh is None:
            self.__terminate_session()
            return
        if op.opcode != OP_Ack:
            logging_warning("Write failed")
            self.__terminate_session()
            return

        # assume the FTP server processes the blocks sequentially. This means
        # when we receive an ack that any blocks between the last ack and this
        # one have been lost
        idx = op.offset // self.write_block_size
        count = (idx - self.write_recv_idx) % self.write_total

        self.write_pending = max(0, self.write_pending - count)
        self.write_recv_idx = idx
        self.write_list.discard(idx)
        self.write_acks += 1
        if self.put_callback_progress:
            self.put_callback_progress(self.write_acks/float(self.write_total))
        self.__send_more_writes()

    def cmd_rm(self, args):
        '''remove file'''
        if len(args) == 0:
            logging_error("cmd_rm Usage: [FILENAME]")
            return
        fname = args[0]
        logging_info("Removing %s", fname)
        enc_fname = bytearray(fname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_RemoveFile, len(enc_fname), 0, 0, 0, enc_fname)
        self.__send(op)

    def cmd_rmdir(self, args):
        '''remove directory'''
        if len(args) == 0:
            logging_error("cmd_rmdir Usage: [DIRECTORYNAME]")
            return
        dname = args[0]
        logging_info("Removing %s", dname)
        enc_dname = bytearray(dname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_RemoveDirectory, len(enc_dname), 0, 0, 0, enc_dname)
        self.__send(op)

    def __handle_remove_reply(self, op, _m):
        '''handle remove reply'''
        if op.opcode != OP_Ack:
            logging_warning("Remove failed %s", op)
        self.op_pending = False

    def cmd_rename(self, args):
        '''rename file'''
        if len(args) < 2:
            logging_error("cmd_rename Usage: [OLDNAME NEWNAME]")
            return
        name1 = args[0]
        name2 = args[1]
        logging_info("Renaming %s to %s", name1, name2)
        enc_name1 = bytearray(name1, 'ascii')
        enc_name2 = bytearray(name2, 'ascii')
        enc_both = enc_name1 + b'\x00' + enc_name2
        op = FTP_OP(self.seq, self.session, OP_Rename, len(enc_both), 0, 0, 0, enc_both)
        self.__send(op)

    def __handle_rename_reply(self, op, _m):
        '''handle rename reply'''
        if op.opcode != OP_Ack:
            logging_error("Rename failed %s", op)
        self.op_pending = False

    def cmd_mkdir(self, args):
        '''make directory'''
        if len(args) < 1:
            logging_error("Usage: mkdir NAME")
            return
        name = args[0]
        logging_info("Creating directory %s", name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateDirectory, len(enc_name), 0, 0, 0, enc_name)
        self.__send(op)

    def __handle_mkdir_reply(self, op, _m):
        '''handle mkdir reply'''
        if op.opcode != OP_Ack:
            logging_error("Create directory failed %s", op)
        self.op_pending = False

    def cmd_crc(self, args):
        '''get crc'''
        if len(args) < 1:
            logging_error("cmd_crc Usage: [NAME]")
            return
        name = args[0]
        self.filename = name
        self.op_start = time_time()
        logging_info("Getting CRC for %s", name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CalcFileCRC32, len(enc_name), 0, 0, 0, bytearray(enc_name))
        self.__send(op)

    def __handle_crc_reply(self, op, _m):
        '''handle crc reply'''
        if op.opcode == OP_Ack and op.size == 4:
            crc, = struct.unpack("<I", op.payload)
            now = time_time()
            logging_info("crc: %s 0x%08x in %.1fs", self.filename, crc, now - self.op_start)
        else:
            logging_error("crc failed %s", op)
        self.op_pending = False

    def cmd_cancel(self):
        '''cancel any pending op'''
        self.__terminate_session()

    def cmd_status(self):
        '''show status'''
        if self.fh is None:
            logging_info("No transfer in progress")
        else:
            ofs = self.fh.tell()
            dt = time_time() - self.op_start
            rate = (ofs / dt) / 1024.0
            logging_info("Transfer at offset %u with %u gaps %u retries %.1f kByte/sec",
                         ofs, len(self.read_gaps), self.read_retries, rate)

    def __op_parse(self, m):
        '''parse a FILE_TRANSFER_PROTOCOL msg'''
        hdr = bytearray(m.payload[0:12])
        (seq, session, opcode, size, req_opcode, burst_complete, _pad, offset) = struct.unpack("<HBBBBBBI", hdr)
        payload = bytearray(m.payload[12:])[:size]
        return FTP_OP(seq, session, opcode, size, req_opcode, burst_complete, offset, payload)

    def __mavlink_packet(self, m):  # pylint: disable=too-many-branches
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "FILE_TRANSFER_PROTOCOL":
            if m.target_system != self.master.source_system or m.target_component != self.master.source_component:
                logging_info("discarding %u:%u", m.target_system, m.target_component)
                return

            op = self.__op_parse(m)
            now = time_time()
            dt = now - self.last_op_time
            if self.ftp_settings_debug > 1:
                logging_info("< %s dt=%.2f", op, dt)
            self.last_op_time = now
            if self.ftp_settings_pkt_loss_rx > 0:
                if random_uniform(0, 100) < self.ftp_settings_pkt_loss_rx:
                    if self.ftp_settings_debug > 1:
                        logging_warning("FTP: dropping packet RX")
                    return

            if op.req_opcode == self.last_op.opcode and op.seq == (self.last_op.seq + 1) % 256:
                self.rtt = max(min(self.rtt, dt), 0.01)
            if op.req_opcode == OP_ListDirectory:
                self.__handle_list_reply(op, m)
            elif op.req_opcode == OP_OpenFileRO:
                self.__handle_open_ro_reply(op, m)
            elif op.req_opcode == OP_BurstReadFile:
                self.__handle_burst_read(op, m)
            elif op.req_opcode == OP_TerminateSession:
                pass
            elif op.req_opcode == OP_CreateFile:
                self.__handle_create_file_reply(op, m)
            elif op.req_opcode == OP_WriteFile:
                self.__handle_write_reply(op, m)
            elif op.req_opcode in [OP_RemoveFile, OP_RemoveDirectory]:
                self.__handle_remove_reply(op, m)
            elif op.req_opcode == OP_Rename:
                self.__handle_rename_reply(op, m)
            elif op.req_opcode == OP_CreateDirectory:
                self.__handle_mkdir_reply(op, m)
            elif op.req_opcode == OP_ReadFile:
                self.__handle_reply_read(op, m)
            elif op.req_opcode == OP_CalcFileCRC32:
                self.__handle_crc_reply(op, m)
            else:
                logging_info('FTP Unknown %s', str(op))

    def __send_gap_read(self, g):
        '''send a read for a gap'''
        (offset, length) = g
        if self.ftp_settings_debug > 0:
            logging_info("Gap read of %u at %u rem=%u blog=%u", length, offset, len(self.read_gaps), self.backlog)
        read = FTP_OP(self.seq, self.session, OP_ReadFile, length, 0, 0, offset, None)
        self.__send(read)
        self.read_gaps.remove(g)
        self.read_gaps.append(g)
        self.last_gap_send = time_time()
        self.read_gap_times[g] = self.last_gap_send
        self.backlog += 1

    def __check_read_send(self):
        '''see if we should send another gap read'''
        if len(self.read_gaps) == 0:
            return
        g = self.read_gaps[0]
        now = time_time()
        dt = now - self.read_gap_times[g]
        if not self.reached_eof:
            # send gap reads once
            for g, v in self.read_gap_times.items():
                if v == 0:
                    self.__send_gap_read(g)
            return
        if self.read_gap_times[g] > 0 and dt > self.ftp_settings_retry_time:
            if self.backlog > 0:
                self.backlog -= 1
            self.read_gap_times[g] = 0

        if self.read_gap_times[g] != 0:
            # still pending
            return
        if not self.reached_eof and self.backlog >= self.ftp_settings_max_backlog:
            # don't fill queue too far until we have got past the burst
            return
        if now - self.last_gap_send < 0.05:
            # don't send too fast
            return
        self.__send_gap_read(g)

    def __idle_task(self):
        '''check for file gaps and lost requests'''
        now = time_time()

        # see if we lost an open reply
        if self.op_start is not None and now - self.op_start > 1.0 and self.last_op.opcode == OP_OpenFileRO:
            self.op_start = now
            self.open_retries += 1
            if self.open_retries > 2:
                # fail the get
                self.op_start = None
                self.__terminate_session()
                return
            if self.ftp_settings_debug > 0:
                logging_info("FTP: retry open")
            send_op = self.last_op
            self.__send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None))
            self.session = (self.session + 1) % 256
            send_op.session = self.session
            self.__send(send_op)

        if len(self.read_gaps) == 0 and self.last_burst_read is None and self.write_list is None:
            return

        if self.fh is None:
            return

        # see if burst read has stalled
        if not self.reached_eof and self.last_burst_read is not None and \
           now - self.last_burst_read > self.ftp_settings_retry_time:
            dt = now - self.last_burst_read
            self.last_burst_read = now
            if self.ftp_settings_debug > 0:
                logging_info("Retry read at %u rtt=%.2f dt=%.2f", self.fh.tell(), self.rtt, dt)
            self.__send(FTP_OP(self.seq, self.session, OP_BurstReadFile, self.burst_size, 0, 0, self.fh.tell(), None))
            self.read_retries += 1

        # see if we can fill gaps
        self.__check_read_send()

    def execute_ftp_operation(self, timeout=5):
        start_time = time_time()
        timed_out = False
        while self.op_pending and not timed_out:
            m = self.master.recv_match(type=['FILE_TRANSFER_PROTOCOL'], timeout=0.1)
            if m is not None:
                self.__mavlink_packet(m)
            self.__idle_task()
            timed_out = time_time() - start_time > timeout
            if timed_out:
                logging_warning("FTP timed out")
                self.__terminate_session()

    @staticmethod
    def ftp_param_decode(data):  # pylint: disable=too-many-locals
        '''decode parameter data, returning ParamData'''
        pdata = ParamData()

        magic = 0x671b
        magic_defaults = 0x671c
        if len(data) < 6:
            return None
        magic2, _num_params, total_params = struct.unpack("<HHH", data[0:6])
        if magic2 not in (magic, magic_defaults):
            logging_error("paramftp: bad magic 0x%x expected 0x%x", magic2, magic)
            return None
        with_defaults = magic2 == magic_defaults
        data = data[6:]

        # mapping of data type to type length and format
        data_types = {
            1: (1, 'b'),
            2: (2, 'h'),
            3: (4, 'i'),
            4: (4, 'f'),
        }

        count = 0
        pad_byte = 0
        last_name = bytes()
        while True:
            while len(data) > 0 and data[0] == pad_byte:
                data = data[1:]  # skip pad bytes

            if len(data) == 0:
                break

            ptype, plen = struct.unpack("<BB", data[0:2])
            flags = (ptype >> 4) & 0x0F
            has_default = with_defaults and (flags & 1) != 0
            ptype &= 0x0F

            if ptype not in data_types:
                logging_error("paramftp: bad type 0x%x", ptype)
                return None

            (type_len, type_format) = data_types[ptype]
            default_len = type_len if has_default else 0

            name_len = ((plen >> 4) & 0x0F) + 1
            common_len = plen & 0x0F
            name = last_name[0:common_len] + data[2:2+name_len]
            vdata = data[2+name_len:2+name_len+type_len+default_len]
            last_name = name
            data = data[2+name_len+type_len+default_len:]
            if with_defaults:
                if has_default:
                    v1, v2, = struct.unpack("<" + type_format + type_format, vdata)
                    pdata.add_param(name, v1, ptype)
                    pdata.add_default(name, v2, ptype)
                else:
                    v, = struct.unpack("<" + type_format, vdata)
                    pdata.add_param(name, v, ptype)
                    pdata.add_default(name, v, ptype)
            else:
                v, = struct.unpack("<" + type_format, vdata)
                pdata.add_param(name, v, ptype)
            count += 1

        if count != total_params:
            logging_error("paramftp: bad count %u should be %u", count, total_params)
            return None

        return pdata


if __name__ == "__main__":

    def argument_parser():
        """
        Parses command-line arguments for the script.

        This function sets up an argument parser to handle the command-line arguments for the script.
        """
        parser = ArgumentParser(description='This main is for testing and development only. '
                                'Usually, the mavftp is called from another script')
        parser.add_argument("--baudrate", type=int,
                        help="master port baud rate", default=115200)
        parser.add_argument("--device", required=True, help="serial device")
        parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                        default=250, help='MAVLink source system for this GCS')
        parser.add_argument("--loglevel", default="INFO", help="log level")
        parser.add_argument("--filename", default="@PARAM/param.pck?withdefaults=1", help="file to fetch")
        parser.add_argument("--decode-parameters", action='store_true', help="decode as a parameter file")

        return parser.parse_args()

    def wait_heartbeat(m):
        '''wait for a heartbeat so we know the target system IDs'''
        logging_info("Waiting for flight controller heartbeat")
        m.wait_heartbeat(timeout=5)
        logging_info("Heartbeat from system %u, component %u", m.target_system, m.target_system)

    def main():
        '''for testing/example purposes only'''
        args = argument_parser()

        logging_basicConfig(level=logging_getLevelName(args.loglevel), format='%(asctime)s - %(levelname)s - %(message)s')

        logging_warning("This main is for testing and development only. "
                        "Usually the backend_mavftp is called from another script")

        # create a mavlink serial instance
        master = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)

        # wait for the heartbeat msg to find the system ID
        wait_heartbeat(master)

        mavftp = MAVFTP(master,
                        target_system=master.target_system,
                        target_component=master.target_component)

        def callback(fh):
            '''called on ftp completion'''
            data = fh.read()
            logging_info("done! got %u bytes", len(data))
            if args.decode_parameters:
                pdata = MAVFTP.ftp_param_decode(data)
                if pdata is None:
                    logging_error("param decode failed")
                    sys.exit(1)

                pdict = {}
                defdict = {}
                for (name,value,_ptype) in pdata.params:
                    pdict[name] = value
                if pdata.defaults:
                    for (name,value,_ptype) in pdata.defaults:
                        defdict[name] = value
                for n in sorted(pdict.keys()):
                    if n in defdict:
                        logging_info("%-16s %f (default %f)", n.decode('utf-8'), pdict[n], defdict[n])
                    else:
                        logging_info("%-16s %f", n.decode('utf-8'), pdict[n])
            sys.exit(0)

        mavftp.cmd_get([args.filename], callback=callback)

        while True:
            m = master.recv_match(type=['FILE_TRANSFER_PROTOCOL'], timeout=0.1)
            if m is not None:
                mavftp.__mavlink_packet(m)
            mavftp.__idle_task()

    main()
