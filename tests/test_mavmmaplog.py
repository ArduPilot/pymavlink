"""Instance indexing must decode payload fields without reading packet trailers."""
import struct

import pytest

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2


@pytest.fixture(params=[(mavlink1, False, 1), (mavlink2, False, 1),
                        (mavlink2, True, 1), (mavlink2, False, 0xFFFFFFFF),
                        (mavlink2, True, 0x80000000)],
                ids=['mavlink1', 'mavlink2', 'mavlink2-signed', 'wide', 'wide-signed'])
def encoder(request, monkeypatch):
    dialect, signed, source = request.param
    monkeypatch.setattr(mavutil, 'mavlink', dialect)
    monkeypatch.setattr(mavutil, 'current_dialect', 'ardupilotmega')
    mav = dialect.MAVLink(None, srcSystem=source, srcComponent=1)
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


@pytest.mark.parametrize('signed', [False, True])
def test_mixed_header_widths_and_filtered_reads(tmp_path, monkeypatch, signed):
    monkeypatch.setattr(mavutil, 'mavlink', mavlink2)
    monkeypatch.setattr(mavutil, 'current_dialect', 'ardupilotmega')
    mav = mavlink2.MAVLink(None, srcComponent=11)
    mav.signing.secret_key = b'x' * 32
    mav.signing.sign_outgoing = signed
    mav.signing.timestamp = 123456789
    path = tmp_path / 'mixed.tlog'
    expected = []
    offsets = []
    with path.open('wb') as output:
        for source in (42, 0xFEDCBA98, 43, 0x80000001, 44):
            for target in (7, 256, 0xFFFFFFFF):
                mav.srcSystem = source
                message = mav.command_long_encode(target, 1, 300, 0, 1, 2, 3, 4, 5, 6, 7)
                offsets.append(output.tell())
                output.write(struct.pack('>Q', 1000000000000000 + len(expected) * 10000))
                output.write(message.pack(mav))
                expected.append((source, target))
                mav.seq += 1
    with mavutil.mavlink_connection(str(path)) as log:
        assert log.offsets[mavlink2.MAVLINK_MSG_ID_COMMAND_LONG] == offsets
        for _ in range(2):
            actual = []
            while True:
                message = log.recv_match(type='COMMAND_LONG')
                if message is None:
                    break
                actual.append((message.get_srcSystem(), message.target_system))
            assert actual == expected
            log.rewind()


@pytest.mark.parametrize('flags', range(8))
def test_truncated_extended_frame_is_not_indexed(tmp_path, monkeypatch, flags):
    monkeypatch.setattr(mavutil, 'mavlink', mavlink2)
    monkeypatch.setattr(mavutil, 'current_dialect', 'ardupilotmega')
    mav = mavlink2.MAVLink(None, srcSystem=0xFFFFFFFF if flags & 2 else 42)
    mav.signing.secret_key = b'x' * 32
    mav.signing.sign_outgoing = bool(flags & 1)
    target = 0x80000000 if flags & 4 else 7
    frame = mav.command_long_encode(target, 1, 300, 0, 1, 2, 3, 4, 5, 6, 7).pack(mav)
    path = tmp_path / 'truncated.tlog'
    for length in range(1, len(frame)):
        path.write_bytes(struct.pack('>Q', 1000000000000000) + frame[:length])
        with mavutil.mavlink_connection(str(path)) as log:
            assert log._count == 0


@pytest.mark.parametrize('flags', [0x08, 0x80, 0x88, 0x09])
@pytest.mark.parametrize('padding', [0, 40, 200])
def test_unknown_incompat_flags_are_not_indexed(tmp_path, monkeypatch, flags, padding):
    monkeypatch.setattr(mavutil, 'mavlink', mavlink2)
    monkeypatch.setattr(mavutil, 'current_dialect', 'ardupilotmega')
    from pymavlink.generator.mavcrc import x25crc
    header = bytes([253, padding, flags, 0, 0, 42, 1, 76, 0, 0])
    unsupported = header + b'\xfd' * padding
    crc = x25crc(unsupported[1:])
    crc.accumulate(bytes([152]))
    unsupported += struct.pack('<H', crc.crc)
    if flags & mavlink2.MAVLINK_IFLAG_SIGNED:
        unsupported += bytes(mavlink2.MAVLINK_SIGNATURE_BLOCK_LEN)
    mav = mavlink2.MAVLink(None, srcSystem=42, srcComponent=1)
    good = mav.command_long_encode(7, 1, 300, 0, 1, 2, 3, 4, 5, 6, 7).pack(mav)
    stamp = struct.pack('>Q', 1000000000000000)
    path = tmp_path / 'unsupported.tlog'
    path.write_bytes(stamp + unsupported + stamp + good)
    with mavutil.mavlink_connection(str(path)) as log:
        assert log._count == 1
        assert log.offsets[mavlink2.MAVLINK_MSG_ID_COMMAND_LONG] == [len(stamp + unsupported)]
        assert log.recv_match(type='COMMAND_LONG').command == 300
        assert log.recv_match(type='COMMAND_LONG') is None
