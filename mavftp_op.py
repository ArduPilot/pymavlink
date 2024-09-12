
import struct
from typing import Optional


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


# pylint: disable=too-many-arguments
# pylint: disable=too-many-instance-attributes


class FTP_OP:
    def __init__(
        self,
        seq: int,
        session: int,
        opcode: int,
        size: int,
        req_opcode: int,
        burst_complete: int,
        offset: int,
        payload: Optional[bytearray],
    ):
        self.seq = seq
        self.session = session
        self.opcode = opcode
        self.size = size
        self.req_opcode = req_opcode
        self.burst_complete = burst_complete
        self.offset = offset
        self.payload = payload

    def pack(self) -> bytearray:
        """pack message"""
        ret = struct.pack(
            "<HBBBBBBI",
            self.seq,
            self.session,
            self.opcode,
            self.size,
            self.req_opcode,
            self.burst_complete,
            0,
            self.offset,
        )
        if self.payload is not None:
            ret += self.payload
        ret = bytearray(ret)
        return ret

    def __str__(self) -> str:
        plen = 0
        if self.payload is not None:
            plen = len(self.payload)
        ret = f"OP seq:{self.seq} sess:{self.session} opcode:{self.opcode} req_opcode:{self.req_opcode} size:{self.size} bc:{self.burst_complete} ofs:{self.offset} plen={plen}"
        if self.payload is not None and plen > 0:
            ret += f" [{self.payload[0]}]"
        return ret
