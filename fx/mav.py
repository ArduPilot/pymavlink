from io import BytesIO
from typing import List, Union

from pymavlink import mavutil


class MavSerializer(mavutil.mavfile):
    def __init__(self):
        mavutil.mavfile.__init__(self, BytesIO(), None)
        self._serialized_data = bytes()

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, data: bytes):
        self._data = data

    def recv(self, n=None):
        if n is None:
            n = self.mav.bytes_needed()
        if n >= len(self._data):
            sliced_data = self._data
            self._data = bytes()
            return sliced_data
        sliced_data = self._data[0:n]
        self._data = self._data[n:]
        return sliced_data

    def has_bytes(self):
        return bool(self._data)

    def deserialize(self, data: bytes = None) -> List[Union[None, dict]]:
        ret = []
        if data:
            self._data = data
        while self.has_bytes():
            msg = self.recv_match()
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            ret.append(msg.to_dict())
        return ret

    def close(self, n=None):
        self.fd.close()

    def get_buffered_sent_data(self) -> bytes:
        data = self._serialized_data
        self._serialized_data = bytes()
        return data

    def write(self, buf):
        self._serialized_data = buf
