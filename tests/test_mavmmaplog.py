"""Instance indexing must decode payload fields without reading packet trailers."""
import struct

import pytest

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2


@pytest.fixture(params=[(mavlink1, False), (mavlink2, False), (mavlink2, True)],
                ids=['mavlink1', 'mavlink2', 'mavlink2-signed'])
def encoder(request, monkeypatch):
    dialect, signed = request.param
    monkeypatch.setattr(mavutil, 'mavlink', dialect)
    monkeypatch.setattr(mavutil, 'current_dialect', 'ardupilotmega')
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    if signed:
        mav.signing.secret_key = b'x' * 32
        mav.signing.link_id = 2
        mav.signing.timestamp = 123456789
        mav.signing.sign_outgoing = True
    return mav


def write_log(path, encoder, messages):
    with path.open('wb') as output:
        for index, message in enumerate(messages):
            output.write(struct.pack('>Q', 1000000000000000 + index * 10000))
            output.write(message.pack(encoder))
            encoder.seq = (encoder.seq + 1) % 256


@pytest.mark.parametrize('name', [b'CPITCH', b'', b'ABCDEFGHIJ', b'A\x00garbage',
                                 b'CPITCH ', b'\xffNAME'])
def test_named_instance_keys_match_decoded_messages(tmp_path, encoder, name):
    # Only the first message of a type is decoded normally by the indexer;
    # later names exercise its fast path, including a fully omitted field.
    messages = [encoder.named_value_float_encode(1, b'FIRST', 1.0)]
    messages.extend(encoder.named_value_float_encode(i + 2, name, float(i)) for i in range(4))
    path = tmp_path / 'instances.tlog'
    write_log(path, encoder, messages)

    log = mavutil.mavlink_connection(str(path))
    try:
        keys = {key for key in log.messages if key.startswith('NAMED_VALUE_FLOAT[')}
        decoded_names = []
        while True:
            message = log.recv_match(type='NAMED_VALUE_FLOAT')
            if message is None:
                break
            decoded_names.append(message.name)
        assert decoded_names == [message.name for message in messages]
        assert keys == {'NAMED_VALUE_FLOAT[%s]' % name for name in decoded_names}
        assert log.counts[mavutil.mavlink.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT] == len(messages)
    finally:
        log.close()


def test_fully_omitted_numeric_instance_still_indexes_message(tmp_path, encoder):
    messages = [encoder.distance_sensor_encode(1, 0, 0, 0, 0, instance, 0, 0)
                for instance in (1, 0, 2, 0)]
    path = tmp_path / 'numeric.tlog'
    write_log(path, encoder, messages)

    log = mavutil.mavlink_connection(str(path))
    try:
        assert {key for key in log.messages if key.startswith('DISTANCE_SENSOR[')} == {
            'DISTANCE_SENSOR[0]', 'DISTANCE_SENSOR[1]', 'DISTANCE_SENSOR[2]'}
        assert log.counts[mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR] == len(messages)
        decoded = []
        while True:
            message = log.recv_match(type='DISTANCE_SENSOR')
            if message is None:
                break
            decoded.append(message.id)
        assert decoded == [1, 0, 2, 0]
    finally:
        log.close()
