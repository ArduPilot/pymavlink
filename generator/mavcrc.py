'''
MAVLink CRC-16/MCRF4XX code

Copyright Andrew Tridgell
Released under GNU LGPL version 3 or later
'''
import sys
from builtins import object


class x25crc(object):
    """CRC-16/MCRF4XX - based on checksum.h from mavlink library"""

    def __init__(self, buf=None):
        self.crc = 0xFFFF
        if buf is not None:
            self.accumulate(buf)

    def accumulate(self, buf):
        """add in some more bytes (it also accepts strings)"""
        if sys.version_info[0] == 2:
            if type(buf) is str:
                buf = bytearray(buf)
            elif type(buf).__name__ == 'unicode':
                # we can't use the identifier unicode in python3
                buf = bytearray(buf.encode())
        elif type(buf) is str:
            buf = buf.encode()

        accum = self.crc
        for b in buf:
            tmp = b ^ (accum & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        self.crc = accum

    def accumulate_str(self, buf):
        """
        Provided for backwards compatibility. accumulate now also works on strings.
        """
        return self.accumulate(buf)
