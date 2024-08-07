#!/usr/bin/env python
'''mavlink file transfer support'''

import io
import time, os, sys
import struct
import random
from pymavlink import mavutil

try:
    # py2
    from StringIO import StringIO as SIO
except ImportError:
    # py3
    from io import BytesIO as SIO

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

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

class FTP_OP:
    def __init__(self, seq, session, opcode, size, req_opcode, burst_complete, offset, payload):
        self.seq = seq
        self.session = session
        self.opcode = opcode
        self.size = size
        self.req_opcode = req_opcode
        self.burst_complete = burst_complete
        self.offset = offset
        self.payload = payload

    def pack(self):
        '''pack message'''
        ret = struct.pack("<HBBBBBBI", self.seq, self.session, self.opcode, self.size, self.req_opcode, self.burst_complete, 0, self.offset)
        if self.payload is not None:
            ret += self.payload
        ret = bytearray(ret)
        return ret

    def __str__(self):
        plen = 0
        if self.payload is not None:
            plen = len(self.payload)
        ret = "OP seq:%u sess:%u opcode:%d req_opcode:%u size:%u bc:%u ofs:%u plen=%u" % (self.seq,
                                                                                          self.session,
                                                                                          self.opcode,
                                                                                          self.req_opcode,
                                                                                          self.size,
                                                                                          self.burst_complete,
                                                                                          self.offset,
                                                                                          plen)
        if plen > 0:
            ret += " [%u]" % self.payload[0]
        return ret

class WriteQueue:
    def __init__(self, ofs, size):
        self.ofs = ofs
        self.size = size
        self.last_send = 0

class FTPModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FTPModule, self).__init__(mpstate, "ftp", public=True)
        self.add_command('ftp', self.cmd_ftp, "file transfer",
                         ["<list|get|rm|rmdir|rename|mkdir|crc|cancel|status>",
                          "set (FTPSETTING)",
                          "put (FILENAME) (FILENAME)"])
        self.ftp_settings = mp_settings.MPSettings(
            [('debug', int, 0),
             ('pkt_loss_tx', int, 0),
             ('pkt_loss_rx', int, 0),
             ('max_backlog', int, 5),
             ('burst_read_size', int, 80),
             ('write_size', int, 80),
             ('write_qsize', int, 5),
             ('retry_time', float, 0.5)])
        self.add_completion_function('(FTPSETTING)',
                                     self.ftp_settings.completion)
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
        self.last_op_time = time.time()
        self.rtt = 0.5
        self.reached_eof = False
        self.backlog = 0
        self.burst_size = self.ftp_settings.burst_read_size
        self.write_list = None
        self.write_block_size = 0
        self.write_acks = 0
        self.write_total = 0
        self.write_file_size = 0
        self.write_idx = 0
        self.write_recv_idx = -1
        self.write_pending = 0
        self.write_last_send = None
        self.warned_component = False

    def cmd_ftp(self, args):
        '''FTP operations'''
        usage = "Usage: ftp <list|get|put|rm|rmdir|rename|mkdir|crc>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == 'list':
            self.cmd_list(args[1:])
        elif args[0] == "set":
            self.ftp_settings.command(args[1:])
        elif args[0] == 'get':
            self.cmd_get(args[1:])
        elif args[0] == 'put':
            self.cmd_put(args[1:])
        elif args[0] == 'rm':
            self.cmd_rm(args[1:])
        elif args[0] == 'rmdir':
            self.cmd_rmdir(args[1:])
        elif args[0] == 'rename':
            self.cmd_rename(args[1:])
        elif args[0] == 'mkdir':
            self.cmd_mkdir(args[1:])
        elif args[0] == 'crc':
            self.cmd_crc(args[1:])
        elif args[0] == 'status':
            self.cmd_status()
        elif args[0] == 'cancel':
            self.cmd_cancel()
        else:
            print(usage)

    def send(self, op):
        '''send a request'''
        op.seq = self.seq
        payload = op.pack()
        plen = len(payload)
        if plen < MAX_Payload + HDR_Len:
            payload.extend(bytearray([0]*((HDR_Len+MAX_Payload)-plen)))
        self.master.mav.file_transfer_protocol_send(self.network, self.target_system, self.target_component, payload)
        self.seq = (self.seq + 1) % 256
        self.last_op = op
        now = time.time()
        if self.ftp_settings.debug > 1:
            print("> %s dt=%.2f" % (op, now - self.last_op_time))
        self.last_op_time = time.time()

    def terminate_session(self):
        '''terminate current session'''
        self.send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None))
        self.fh = None
        self.filename = None
        self.write_list = None
        if self.callback is not None:
            # tell caller that the transfer failed
            self.callback(None)
            self.callback = None
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
        if self.ftp_settings.debug > 0:
            print("Terminated session")

    def cmd_list(self, args):
        '''list files'''
        if len(args) > 0:
            dname = args[0]
        else:
            dname = '/'
        print("Listing %s" % dname)
        enc_dname = bytearray(dname, 'ascii')
        self.total_size = 0
        self.dir_offset = 0
        op = FTP_OP(self.seq, self.session, OP_ListDirectory, len(enc_dname), 0, 0, self.dir_offset, enc_dname)
        self.send(op)

    def handle_list_reply(self, op, m):
        '''handle OP_ListDirectory reply'''
        if op.opcode == OP_Ack:
            dentries = sorted(op.payload.split(b'\x00'))
            #print(dentries)
            for d in dentries:
                if len(d) == 0:
                    continue
                self.dir_offset += 1
                try:
                    if sys.version_info.major >= 3:
                        d = str(d, 'ascii')
                    else:
                        d = str(d)
                except Exception:
                    continue
                if d[0] == 'D':
                    print(" D %s" % d[1:])
                elif d[0] == 'F':
                    (name, size) = d[1:].split('\t')
                    size = int(size)
                    self.total_size += size
                    print("   %s\t%u" % (name, size))
                else:
                    print(d)
            # ask for more
            more = self.last_op
            more.offset = self.dir_offset
            self.send(more)
        elif op.opcode == OP_Nack and len(op.payload) == 1 and op.payload[0] == ERR_EndOfFile:
            print("Total size %.2f kByte" % (self.total_size / 1024.0))
            self.total_size = 0
        else:
            print('LIST: %s' % op)

    def cmd_get(self, args, callback=None, callback_progress=None):
        '''get file'''
        if len(args) == 0:
            print("Usage: get FILENAME <LOCALNAME>")
            return
        self.terminate_session()
        fname = args[0]
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        if callback is None or self.ftp_settings.debug > 1:
            print("Getting %s as %s" % (fname, self.filename))
        self.op_start = time.time()
        self.callback = callback
        self.callback_progress = callback_progress
        self.read_retries = 0
        self.duplicates = 0
        self.reached_eof = False
        self.burst_size = self.ftp_settings.burst_read_size
        if self.burst_size < 1:
            self.burst_size = 239
        elif self.burst_size > 239:
            self.burst_size = 239
        enc_fname = bytearray(fname, 'ascii')
        self.open_retries = 0
        op = FTP_OP(self.seq, self.session, OP_OpenFileRO, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def handle_open_RO_reply(self, op, m):
        '''handle OP_OpenFileRO reply'''
        if op.opcode == OP_Ack:
            if self.filename is None:
                return
            try:
                if self.callback is not None or self.filename == '-':
                    self.fh = SIO()
                else:
                    self.fh = open(self.filename, 'wb')
            except Exception as ex:
                print("Failed to open %s: %s" % (self.filename, ex))
                self.terminate_session()
                return
            read = FTP_OP(self.seq, self.session, OP_BurstReadFile, self.burst_size, 0, 0, 0, None)
            self.last_burst_read = time.time()
            self.send(read)
        else:
            if self.callback is None or self.ftp_settings.debug > 0:
                print("ftp open failed")
            self.terminate_session()

    def check_read_finished(self):
        '''check if download has completed'''
        if self.reached_eof and len(self.read_gaps) == 0:
            ofs = self.fh.tell()
            dt = time.time() - self.op_start
            rate = (ofs / dt) / 1024.0
            if self.callback is not None:
                self.fh.seek(0)
                self.callback(self.fh)
                self.callback = None
            elif self.filename == "-":
                self.fh.seek(0)
                if sys.version_info.major < 3:
                    print(self.fh.read())
                else:
                    print(self.fh.read().decode('utf-8'))
            else:
                print("Wrote %u bytes to %s in %.2fs %.1fkByte/s" % (ofs, self.filename, dt, rate))
            self.terminate_session()
            return True
        return False

    def write_payload(self, op):
        '''write payload from a read op'''
        self.fh.seek(op.offset)
        self.fh.write(op.payload)
        self.read_total += len(op.payload)
        if self.callback_progress is not None:
            self.callback_progress(self.fh, self.read_total)
    
    def handle_burst_read(self, op, m):
        '''handle OP_BurstReadFile reply'''
        if self.ftp_settings.pkt_loss_tx > 0:
            if random.uniform(0,100) < self.ftp_settings.pkt_loss_tx:
                if self.ftp_settings.debug > 0:
                    print("FTP: dropping TX")
                return
        if self.fh is None or self.filename is None:
            if op.session != self.session:
                # old session
                return
            print("FTP Unexpected burst read reply")
            print(op)
            return
        self.last_burst_read = time.time()
        size = len(op.payload)
        if size > self.burst_size:
            # this server doesn't handle the burst size argument
            self.burst_size = MAX_Payload
            if self.ftp_settings.debug > 0:
                print("Setting burst size to %u" % self.burst_size)
        if op.opcode == OP_Ack and self.fh is not None:
            ofs = self.fh.tell()
            if op.offset < ofs:
                # writing an earlier portion, possibly remove a gap
                gap = (op.offset, len(op.payload))
                if gap in self.read_gaps:
                    self.read_gaps.remove(gap)
                    self.read_gap_times.pop(gap)
                    if self.ftp_settings.debug > 0:
                        print("FTP: removed gap", gap, self.reached_eof, len(self.read_gaps))
                else:
                    if self.ftp_settings.debug > 0:
                        print("FTP: dup read reply at %u of len %u ofs=%u" % (op.offset, op.size, self.fh.tell()))
                    self.duplicates += 1
                    return
                self.write_payload(op)
                self.fh.seek(ofs)
                if self.check_read_finished():
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
                self.write_payload(op)
            else:
                self.write_payload(op)
            if op.burst_complete:
                if op.size > 0 and op.size < self.burst_size:
                    # a burst complete with non-zero size and less than burst packet size
                    # means EOF
                    if not self.reached_eof and self.ftp_settings.debug > 0:
                        print("EOF at %u with %u gaps t=%.2f" % (self.fh.tell(), len(self.read_gaps), time.time() - self.op_start))
                    self.reached_eof = True
                    if self.check_read_finished():
                        return
                    self.check_read_send()
                    return
                more = self.last_op
                more.offset = op.offset + op.size
                if self.ftp_settings.debug > 0:
                    print("FTP: burst continue at %u %u" % (more.offset, self.fh.tell()))
                self.send(more)
        elif op.opcode == OP_Nack:
            ecode = op.payload[0]
            if self.ftp_settings.debug > 0:
                print("FTP: burst nack: ", op)
            if ecode == ERR_EndOfFile or ecode == 0:
                if not self.reached_eof and op.offset > self.fh.tell():
                    # we lost the last part of the burst
                    if self.ftp_settings.debug > 0:
                        print("burst lost EOF %u %u" % (self.fh.tell(), op.offset))
                    return
                if not self.reached_eof and self.ftp_settings.debug > 0:
                    print("EOF at %u with %u gaps t=%.2f" % (self.fh.tell(), len(self.read_gaps), time.time() - self.op_start))
                self.reached_eof = True
                if self.check_read_finished():
                    return
                self.check_read_send()
            elif self.ftp_settings.debug > 0:
                print("FTP: burst Nack (ecode:%u): %s" % (ecode, op))
        else:
            print("FTP: burst error: %s" % op)

    def handle_reply_read(self, op, m):
        '''handle OP_ReadFile reply'''
        if self.fh is None or self.filename is None:
            if self.ftp_settings.debug > 0:
                print("FTP Unexpected read reply")
                print(op)
            return
        if self.backlog > 0:
            self.backlog -= 1
        if op.opcode == OP_Ack and self.fh is not None:
            gap = (op.offset, op.size)
            if gap in self.read_gaps:
                self.read_gaps.remove(gap)
                self.read_gap_times.pop(gap)
                ofs = self.fh.tell()
                self.write_payload(op)
                self.fh.seek(ofs)
                if self.ftp_settings.debug > 0:
                    print("FTP: removed gap", gap, self.reached_eof, len(self.read_gaps))
                if self.check_read_finished():
                    return
            elif op.size < self.burst_size:
                print("FTP: file size changed to %u" % op.offset+op.size)
                self.terminate_session()
            else:
                self.duplicates += 1
                if self.ftp_settings.debug > 0:
                    print("FTP: no gap read", gap, len(self.read_gaps))
        elif op.opcode == OP_Nack:
            print("Read failed with %u gaps" % len(self.read_gaps), str(op))
            self.terminate_session()
        self.check_read_send()
            
    def cmd_put(self, args, fh=None, callback=None, progress_callback=None):
        '''put file'''
        if len(args) == 0:
            print("Usage: put FILENAME <REMOTENAME>")
            return
        if self.write_list is not None:
            print("put already in progress")
            return
        fname = args[0]
        self.fh = fh
        if self.fh is None:
            try:
                self.fh = open(fname, 'rb')
            except Exception as ex:
                print("Failed to open %s: %s" % (fname, ex))
                return
        if len(args) > 1:
            self.filename = args[1]
        else:
            self.filename = os.path.basename(fname)
        if self.filename.endswith("/"):
            self.filename += os.path.basename(fname)
        if callback is None:
            print("Putting %s as %s" % (fname, self.filename))
        self.fh.seek(0,2)
        file_size = self.fh.tell()
        self.fh.seek(0)

        # setup write list
        self.write_block_size = self.ftp_settings.write_size
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
        self.op_start = time.time()
        enc_fname = bytearray(self.filename, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateFile, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def put_finished(self, flen):
        '''finish a put'''
        if self.put_callback_progress:
            self.put_callback_progress(1.0)
            self.put_callback_progress = None
        if self.put_callback is not None:
            self.put_callback(flen)
            self.put_callback = None
        else:
            print("Sent file of length ", flen)
        
    def handle_create_file_reply(self, op, m):
        '''handle OP_CreateFile reply'''
        if self.fh is None:
            self.terminate_session()
            return
        if op.opcode == OP_Ack:
            self.send_more_writes()
        else:
            print("Create failed")
            self.terminate_session()

    def send_more_writes(self):
        '''send some more writes'''
        if len(self.write_list) == 0:
            # all done
            self.put_finished(self.write_file_size)
            self.terminate_session()
            return

        now = time.time()
        if self.write_last_send is not None:
            if now - self.write_last_send > max(min(10*self.rtt, 1),0.2):
                # we seem to have lost a block of replies
                self.write_pending = max(0, self.write_pending-1)

        n = min(self.ftp_settings.write_qsize-self.write_pending, len(self.write_list))
        for i in range(n):
            # send in round-robin, skipping any that have been acked
            idx = self.write_idx
            while idx not in self.write_list:
                idx = (idx + 1) % self.write_total
            ofs = idx * self.write_block_size
            self.fh.seek(ofs)
            data = self.fh.read(self.write_block_size)
            write = FTP_OP(self.seq, self.session, OP_WriteFile, len(data), 0, 0, ofs, bytearray(data))
            self.send(write)
            self.write_idx = (idx + 1) % self.write_total
            self.write_pending += 1
            self.write_last_send = now

    def handle_write_reply(self, op, m):
        '''handle OP_WriteFile reply'''
        if self.fh is None:
            self.terminate_session()
            return
        if op.opcode != OP_Ack:
            print("Write failed")
            self.terminate_session()
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
        self.send_more_writes()

    def cmd_rm(self, args):
        '''remove file'''
        if len(args) == 0:
            print("Usage: rm FILENAME")
            return
        fname = args[0]
        print("Removing %s" % fname)
        enc_fname = bytearray(fname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_RemoveFile, len(enc_fname), 0, 0, 0, enc_fname)
        self.send(op)

    def cmd_rmdir(self, args):
        '''remove directory'''
        if len(args) == 0:
            print("Usage: rmdir FILENAME")
            return
        dname = args[0]
        print("Removing %s" % dname)
        enc_dname = bytearray(dname, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_RemoveDirectory, len(enc_dname), 0, 0, 0, enc_dname)
        self.send(op)

    def handle_remove_reply(self, op, m):
        '''handle remove reply'''
        if op.opcode != OP_Ack:
            print("Remove failed %s" % op)

    def cmd_rename(self, args):
        '''rename file'''
        if len(args) < 2:
            print("Usage: rename OLDNAME NEWNAME")
            return
        name1 = args[0]
        name2 = args[1]
        print("Renaming %s to %s" % (name1, name2))
        enc_name1 = bytearray(name1, 'ascii')
        enc_name2 = bytearray(name2, 'ascii')
        enc_both = enc_name1 + b'\x00' + enc_name2
        op = FTP_OP(self.seq, self.session, OP_Rename, len(enc_both), 0, 0, 0, enc_both)
        self.send(op)

    def handle_rename_reply(self, op, m):
        '''handle rename reply'''
        if op.opcode != OP_Ack:
            print("Rename failed %s" % op)

    def cmd_mkdir(self, args):
        '''make directory'''
        if len(args) < 1:
            print("Usage: mkdir NAME")
            return
        name = args[0]
        print("Creating directory %s" % name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CreateDirectory, len(enc_name), 0, 0, 0, enc_name)
        self.send(op)

    def handle_mkdir_reply(self, op, m):
        '''handle mkdir reply'''
        if op.opcode != OP_Ack:
            print("Create directory failed %s" % op)

    def cmd_crc(self, args):
        '''get crc'''
        if len(args) < 1:
            print("Usage: crc NAME")
            return
        name = args[0]
        self.filename = name
        self.op_start = time.time()
        print("Getting CRC for %s" % name)
        enc_name = bytearray(name, 'ascii')
        op = FTP_OP(self.seq, self.session, OP_CalcFileCRC32, len(enc_name), 0, 0, 0, bytearray(enc_name))
        self.send(op)

    def handle_crc_reply(self, op, m):
        '''handle crc reply'''
        if op.opcode == OP_Ack and op.size == 4:
            crc, = struct.unpack("<I", op.payload)
            now = time.time()
            print("crc: %s 0x%08x in %.1fs" % (self.filename, crc, now - self.op_start))
        else:
            print("crc failed %s" % op)

    def cmd_cancel(self):
        '''cancel any pending op'''
        self.terminate_session()

    def cmd_status(self):
        '''show status'''
        if self.fh is None:
            print("No transfer in progress")
        else:
            ofs = self.fh.tell()
            dt = time.time() - self.op_start
            rate = (ofs / dt) / 1024.0
            print("Transfer at offset %u with %u gaps %u retries %.1f kByte/sec" % (ofs, len(self.read_gaps), self.read_retries, rate))

    def op_parse(self, m):
        '''parse a FILE_TRANSFER_PROTOCOL msg'''
        hdr = bytearray(m.payload[0:12])
        (seq, session, opcode, size, req_opcode, burst_complete, pad, offset) = struct.unpack("<HBBBBBBI", hdr)
        payload = bytearray(m.payload[12:])[:size]
        return FTP_OP(seq, session, opcode, size, req_opcode, burst_complete, offset, payload)

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "FILE_TRANSFER_PROTOCOL":
            if (m.target_system != self.settings.source_system or
                m.target_component != self.settings.source_component):
                if m.target_system == self.settings.source_system and not self.warned_component:
                    self.warned_component = True
                    print("FTP reply for mavlink component %u" % m.target_component)
                return

            op = self.op_parse(m)
            now = time.time()
            dt = now - self.last_op_time
            if self.ftp_settings.debug > 1:
                print("< %s dt=%.2f" % (op, dt))
            self.last_op_time = now
            if self.ftp_settings.pkt_loss_rx > 0:
                if random.uniform(0,100) < self.ftp_settings.pkt_loss_rx:
                    if self.ftp_settings.debug > 1:
                        print("FTP: dropping packet RX")
                    return

            if op.req_opcode == self.last_op.opcode and op.seq == (self.last_op.seq + 1) % 256:
                self.rtt = max(min(self.rtt, dt), 0.01)
            if op.req_opcode == OP_ListDirectory:
                self.handle_list_reply(op, m)
            elif op.req_opcode == OP_OpenFileRO:
                self.handle_open_RO_reply(op, m)
            elif op.req_opcode == OP_BurstReadFile:
                self.handle_burst_read(op, m)
            elif op.req_opcode == OP_TerminateSession:
                pass
            elif op.req_opcode == OP_CreateFile:
                self.handle_create_file_reply(op, m)
            elif op.req_opcode == OP_WriteFile:
                self.handle_write_reply(op, m)
            elif op.req_opcode in [OP_RemoveFile, OP_RemoveDirectory]:
                self.handle_remove_reply(op, m)
            elif op.req_opcode == OP_Rename:
                self.handle_rename_reply(op, m)
            elif op.req_opcode == OP_CreateDirectory:
                self.handle_mkdir_reply(op, m)
            elif op.req_opcode == OP_ReadFile:
                self.handle_reply_read(op, m)
            elif op.req_opcode == OP_CalcFileCRC32:
                self.handle_crc_reply(op, m)
            else:
                print('FTP Unknown %s' % str(op))

    def send_gap_read(self, g):
        '''send a read for a gap'''
        (offset, length) = g
        if self.ftp_settings.debug > 0:
            print("Gap read of %u at %u rem=%u blog=%u" % (length, offset, len(self.read_gaps), self.backlog))
        read = FTP_OP(self.seq, self.session, OP_ReadFile, length, 0, 0, offset, None)
        self.send(read)
        self.read_gaps.remove(g)
        self.read_gaps.append(g)
        self.last_gap_send = time.time()
        self.read_gap_times[g] = self.last_gap_send
        self.backlog += 1

    def check_read_send(self):
        '''see if we should send another gap read'''
        if len(self.read_gaps) == 0:
            return
        g = self.read_gaps[0]
        now = time.time()
        dt = now - self.read_gap_times[g]
        if not self.reached_eof:
            # send gap reads once
            for g in self.read_gap_times.keys():
                if self.read_gap_times[g] == 0:
                    self.send_gap_read(g)
            return
        if self.read_gap_times[g] > 0 and dt > self.ftp_settings.retry_time:
            if self.backlog > 0:
                self.backlog -= 1
            self.read_gap_times[g] = 0

        if self.read_gap_times[g] != 0:
            # still pending
            return
        if not self.reached_eof and self.backlog >= self.ftp_settings.max_backlog:
            # don't fill queue too far until we have got past the burst
            return
        if now - self.last_gap_send < 0.05:
            # don't send too fast
            return
        self.send_gap_read(g)

    def idle_task(self):
        '''check for file gaps and lost requests'''
        now = time.time()

        # see if we lost an open reply
        if self.op_start is not None and now - self.op_start > 1.0 and self.last_op.opcode == OP_OpenFileRO:
            self.op_start = now
            self.open_retries += 1
            if self.open_retries > 2:
                # fail the get
                self.op_start = None
                self.terminate_session()
                return
            if self.ftp_settings.debug > 0:
                print("FTP: retry open")
            send_op = self.last_op
            self.send(FTP_OP(self.seq, self.session, OP_TerminateSession, 0, 0, 0, 0, None))
            self.session = (self.session + 1) % 256
            send_op.session = self.session
            self.send(send_op)

        if len(self.read_gaps) == 0 and self.last_burst_read is None and self.write_list is None:
            return

        if self.fh is None:
            return

        # see if burst read has stalled
        if not self.reached_eof and self.last_burst_read is not None and now - self.last_burst_read > self.ftp_settings.retry_time:
            dt = now - self.last_burst_read
            self.last_burst_read = now
            if self.ftp_settings.debug > 0:
                print("Retry read at %u rtt=%.2f dt=%.2f" % (self.fh.tell(), self.rtt, dt))
            self.send(FTP_OP(self.seq, self.session, OP_BurstReadFile, self.burst_size, 0, 0, self.fh.tell(), None))
            self.read_retries += 1

        # see if we can fill gaps
        self.check_read_send()

        if self.write_list is not None:
            self.send_more_writes()

def init(mpstate):
    '''initialise module'''
    return FTPModule(mpstate)
