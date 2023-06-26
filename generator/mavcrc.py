'''
MAVLink CRC-16/MCRF4XX code

Copyright Andrew Tridgell
Released under GNU LGPL version 3 or later
'''


class x25crc:
    """CRC-16/MCRF4XX - based on checksum.h from mavlink library"""

    def __init__(self, buf=None):
        self.crc = 0xFFFF
        if buf is not None:
            self.accumulate(buf)

    def accumulate(self, buf):
        """add in some more bytes (it also accepts strings)"""
        if type(buf) is str:
            buf = buf.encode()

        accum = self.crc
        for b in buf:
            tmp = b ^ (accum & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        self.crc = accum
