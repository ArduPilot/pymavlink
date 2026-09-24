#!/usr/bin/env python3

"""
Test 32 bit system IDs (MAVLINK_IFLAG_SYSID32) and extended header
targeting (MAVLINK_IFLAG_TARGET32)
"""

import hashlib
import importlib.util
import pathlib
import struct
import sys

import pytest

from pymavlink.generator import mavgen

SYSID_SMALL = 42
SYSID_BIG = 0x0A000001  # 10.0.0.1
TARGET_SMALL = 7
TARGET_BIG = 0x0A000002  # 10.0.0.2

# Independently computed X.25 wire vectors for COMMAND_LONG packed with
# (sysid, 11, target, 250, 300, 1, 1.0 ... 7.0) at seq 0.
# Cross-language wire compatibility depends on these matching exactly.
GOLDEN = {
    (SYSID_SMALL, TARGET_SMALL): "fd210000002a0b4c00000000803f0000004000004040000080400000a0400000c0400000e0402c0107fa01ce94",
    (SYSID_SMALL, TARGET_BIG): "fd210400002a0b4c00000200000a0000803f0000004000004040000080400000a0400000c0400000e0402c01fffa018df2",
    (SYSID_BIG, TARGET_SMALL): "fd210200000100000a0b4c00000000803f0000004000004040000080400000a0400000c0400000e0402c0107fa019d0f",
    (SYSID_BIG, TARGET_BIG): "fd210600000100000a0b4c00000200000a0000803f0000004000004040000080400000a0400000c0400000e0402c01fffa0140e5",
}
# signed with key bytes([42]*32), link_id 3, timestamp 1000
GOLDEN_SIGNED = "fd210700000100000a0b4c00000200000a0000803f0000004000004040000080400000a0400000c0400000e0402c01fffa01d03903e80300000000349dea1b799d"
SIGNING_KEY = bytes([42] * 32)


class FakeFile:
    def write(self, b):
        pass


@pytest.fixture(scope="module", params=["Python", "Python3"])
def mav(tmp_path_factory, request):
    """generate the v2.0 common dialect from the in-tree generator"""
    xml = pathlib.Path(__file__).parent / "snapshottests" / "resources" / "common.xml"
    out = tmp_path_factory.mktemp("sysid32") / "mavcommon.py"
    mavgen.mavgen(
        mavgen.Opts(output=str(out), language=request.param, wire_protocol="2.0", validate=False),
        [str(xml)],
    )
    spec = importlib.util.spec_from_file_location("mavcommon_sysid32", out)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["mavcommon_sysid32"] = mod
    spec.loader.exec_module(mod)
    return mod


def pack_command_long(mav, sysid, target, **link_kwargs):
    link = mav.MAVLink(FakeFile(), srcSystem=sysid, srcComponent=11)
    for k, v in link_kwargs.items():
        setattr(link.signing, k, v)
    msg = mav.MAVLink_command_long_message(target, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7)
    return msg.pack(link)


@pytest.mark.parametrize("sysid", [SYSID_SMALL, SYSID_BIG])
@pytest.mark.parametrize("target", [TARGET_SMALL, TARGET_BIG])
def test_round_trip(mav, sysid, target):
    buf = pack_command_long(mav, sysid, target)

    rx = mav.MAVLink(FakeFile())
    m = rx.parse_char(buf)
    assert m is not None
    assert m.get_type() == "COMMAND_LONG"
    assert m.get_srcSystem() == sysid
    assert m.get_srcComponent() == 11
    # header target is overlaid onto the decoded field
    assert m.target_system == target
    assert m.target_component == 250
    assert m.get_target_system() == target
    assert m.get_target_component() == 250
    assert m.command == 300
    assert m.param7 == 7.0

    hdr = m.get_header()
    assert ((hdr.incompat_flags & mav.MAVLINK_IFLAG_SYSID32) != 0) == (sysid > 255)
    assert ((hdr.incompat_flags & mav.MAVLINK_IFLAG_TARGET32) != 0) == (target > 255)
    assert hdr.compat_flags == 0

    # payload must not contain the >255 target (it goes in the header)
    if target > 255:
        payload = m.get_payload()
        if len(payload) > 30:
            assert payload[30] == 255


@pytest.mark.parametrize("sysid", [SYSID_SMALL, SYSID_BIG])
@pytest.mark.parametrize("target", [TARGET_SMALL, TARGET_BIG])
def test_golden_frames(mav, sysid, target):
    """frames must match the C implementation byte for byte"""
    buf = pack_command_long(mav, sysid, target)
    assert buf == bytes.fromhex(GOLDEN[(sysid, target)])


@pytest.mark.parametrize("sysid", [SYSID_SMALL, SYSID_BIG])
@pytest.mark.parametrize("target", [256, TARGET_BIG, 0xFFFFFFFF])
@pytest.mark.parametrize("signed", [False, True])
@pytest.mark.parametrize("message_type", ["COMMAND_LONG"])
@pytest.mark.parametrize("component", [0, 12, 250])
def test_header_target(mav, sysid, target, signed, message_type, component):
    """Wide targets work independently of the source ID width."""
    if message_type == "COMMAND_LONG":
        normal = bytearray(pack_command_long(mav, SYSID_SMALL, TARGET_SMALL))
        crc_extra = mav.MAVLink_command_long_message.crc_extra
    else:
        link = mav.MAVLink(FakeFile(), srcSystem=SYSID_SMALL, srcComponent=11)
        normal = bytearray(mav.MAVLink_heartbeat_message(2, 3, 81, 0, 4, 3).pack(link))
        crc_extra = mav.MAVLink_heartbeat_message.crc_extra
    payload = normal[10:-2]
    if message_type == "COMMAND_LONG":
        payload[31] = component
        payload[30] = 99  # conflicting payload system; component stays in the payload

    flags = mav.MAVLINK_IFLAG_TARGET32
    if sysid > 255:
        flags |= mav.MAVLINK_IFLAG_SYSID32
    if signed:
        flags |= mav.MAVLINK_IFLAG_SIGNED

    header = mav.MAVLink_header(
        normal[7],
        incompat_flags=flags,
        compat_flags=0,
        mlen=len(payload),
        seq=0,
        srcSystem=sysid,
        srcComponent=11,
        target_system=target,
    ).pack()
    assert len(header) == (17 if sysid > 255 else 14)

    buf = bytearray(header) + payload
    crc = mav.x25crc(buf[1:])
    crc.accumulate(bytes([crc_extra]))
    buf.extend((crc.crc & 0xFF, crc.crc >> 8))
    if signed:
        buf.extend(struct.pack("<BQ", 3, 1000)[:7])
        buf.extend(hashlib.sha256(SIGNING_KEY + buf).digest()[:6])

    rx = mav.MAVLink(FakeFile())
    if signed:
        rx.signing.secret_key = SIGNING_KEY
        rx.signing.timestamp = 999
    # Check incremental framing and synchronization with the next ordinary frame.
    for byte in buf[:-1]:
        assert rx.parse_char(bytes([byte])) is None
    msg = rx.parse_char(buf[-1:])
    assert msg is not None
    assert msg.get_type() == message_type
    assert msg.get_srcSystem() == sysid
    assert msg.get_target_system() == target
    assert msg.get_target_component() == component
    assert msg.get_signed() == signed
    if message_type == "COMMAND_LONG":
        assert msg.target_system == target
        assert msg.target_component == component
    rx.signing.secret_key = None
    following = rx.parse_char(normal)
    assert following.get_type() == message_type
    assert following.get_header().incompat_flags == 0

    # All four target-header bytes are protected by the checksum/signature.
    for offset in range(len(header) - 4, len(header)):
        damaged = bytearray(buf)
        damaged[offset] ^= 1
        with pytest.raises(mav.MAVError, match="CRC"):
            mav.MAVLink(FakeFile()).parse_char(damaged)


def test_golden_signed(mav):
    buf = pack_command_long(
        mav,
        SYSID_BIG,
        TARGET_BIG,
        secret_key=SIGNING_KEY,
        sign_outgoing=True,
        link_id=3,
        timestamp=1000,
    )
    assert buf == bytes.fromhex(GOLDEN_SIGNED)


def test_parse_c_signed_frame(mav):
    """a signed extended frame from the C implementation must verify"""
    rx = mav.MAVLink(FakeFile())
    rx.signing.secret_key = SIGNING_KEY
    rx.signing.timestamp = 999
    m = rx.parse_char(bytes.fromhex(GOLDEN_SIGNED))
    assert m is not None
    assert m.get_type() == "COMMAND_LONG"
    assert m.get_signed()
    assert m.get_srcSystem() == SYSID_BIG
    assert m.target_system == TARGET_BIG


def test_signing_round_trip(mav):
    buf = pack_command_long(
        mav,
        SYSID_BIG,
        TARGET_BIG,
        secret_key=SIGNING_KEY,
        sign_outgoing=True,
        link_id=1,
        timestamp=2000,
    )
    rx = mav.MAVLink(FakeFile())
    rx.signing.secret_key = SIGNING_KEY
    rx.signing.timestamp = 1999
    m = rx.parse_char(buf)
    assert m is not None and m.get_type() == "COMMAND_LONG"
    assert m.get_signed()

    # corrupted signature must be rejected
    bad = bytearray(buf)
    bad[-1] ^= 0x40
    rx = mav.MAVLink(FakeFile())
    rx.robust_parsing = True
    rx.signing.secret_key = SIGNING_KEY
    rx.signing.timestamp = 1999
    m = rx.parse_char(bytes(bad))
    assert isinstance(m, mav.MAVLink_bad_data)


def test_corrupt_extended_header(mav):
    """corrupting an extended header byte must fail the CRC"""
    buf = bytearray(pack_command_long(mav, SYSID_BIG, TARGET_BIG))
    buf[7] ^= 0x40  # high byte of the 32 bit sysid
    rx = mav.MAVLink(FakeFile())
    rx.robust_parsing = True
    m = rx.parse_char(bytes(buf))
    assert isinstance(m, mav.MAVLink_bad_data)


@pytest.mark.parametrize("flags", range(8))
@pytest.mark.parametrize("unknown_flag", [0x08, 0x10, 0x20, 0x40, 0x80])
def test_unknown_incompat_flags(mav, flags, unknown_flag):
    """Direct and streaming decoders reject every unsupported incompat bit."""
    link = mav.MAVLink(FakeFile(), srcSystem=SYSID_BIG if flags & 2 else SYSID_SMALL)
    link.signing.secret_key = SIGNING_KEY
    link.signing.sign_outgoing = bool(flags & 1)
    link.signing.timestamp = 1000
    message = mav.MAVLink_command_long_message(TARGET_BIG if flags & 4 else TARGET_SMALL,
                                               250, 300, 1, 1, 2, 3, 4, 5, 6, 7)
    valid = bytearray(message.pack(link))
    assert valid[2] == flags

    def receiver(robust=False):
        rx = mav.MAVLink(FakeFile())
        rx.robust_parsing = robust
        rx.signing.secret_key = SIGNING_KEY if flags & 1 else None
        rx.signing.timestamp = 999
        return rx

    assert receiver().decode(valid).get_type() == "COMMAND_LONG"
    rejected = bytearray(valid)
    rejected[2] |= unknown_flag
    signature_len = 13 if flags & 1 else 0
    crc_offset = len(rejected) - signature_len - 2
    crc = mav.x25crc(rejected[1:crc_offset])
    crc.accumulate(bytes([message.crc_extra]))
    rejected[crc_offset:crc_offset + 2] = struct.pack("<H", crc.crc)
    if signature_len:
        rejected[-6:] = hashlib.sha256(SIGNING_KEY + rejected[:-6]).digest()[:6]

    # Valid checksums/signatures ensure rejection is due to the flag itself.
    with pytest.raises(mav.MAVError, match="invalid incompat_flags"):
        receiver().decode(rejected)
    for robust in [False, True]:
        rx = receiver(robust)
        for byte in rejected[:-1]:
            assert rx.parse_char(bytes([byte])) is None
        if robust:
            bad = rx.parse_char(rejected[-1:])
            assert isinstance(bad, mav.MAVLink_bad_data)
            assert "invalid incompat_flags" in bad.reason
            assert rx.total_receive_errors == 1
        else:
            with pytest.raises(mav.MAVError, match="invalid incompat_flags"):
                rx.parse_char(rejected[-1:])
        assert rx.parse_char(valid).get_type() == "COMMAND_LONG"


def test_unknown_compat_flags_and_mavlink1_sequence(mav):
    """The incompat mask must not reject compat bits or MAVLink1's sequence."""
    link = mav.MAVLink(FakeFile(), srcSystem=SYSID_SMALL)
    message = mav.MAVLink_heartbeat_message(2, 3, 81, 0, 4, 3)
    v2 = bytearray(message.pack(link))
    v2[3] = 0xFF
    crc = mav.x25crc(v2[1:-2])
    crc.accumulate(bytes([message.crc_extra]))
    v2[-2:] = struct.pack("<H", crc.crc)
    link.seq = 0xFF
    v1 = bytearray(message.pack(link, force_mavlink1=True))
    assert v1[2] == 0xFF
    for packet in [v1, v2]:
        assert mav.MAVLink(FakeFile()).decode(packet).get_type() == "HEARTBEAT"
        assert mav.MAVLink(FakeFile()).parse_char(packet).get_type() == "HEARTBEAT"


def test_mavlink1_guards(mav):
    link = mav.MAVLink(FakeFile(), srcSystem=SYSID_BIG, srcComponent=11)
    msg = mav.MAVLink_heartbeat_message(2, 3, 81, 0, 4, 3)
    with pytest.raises(mav.MAVError):
        msg.pack(link, force_mavlink1=True)

    link = mav.MAVLink(FakeFile(), srcSystem=SYSID_SMALL, srcComponent=11)
    msg = mav.MAVLink_command_long_message(TARGET_BIG, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7)
    with pytest.raises(mav.MAVError):
        msg.pack(link, force_mavlink1=True)


def test_legacy_compat(mav):
    """Small IDs retain the ordinary MAVLink2 frame byte-for-byte."""
    buf = pack_command_long(mav, SYSID_SMALL, TARGET_SMALL)
    assert buf[2:4] == b"\0\0"
    assert not hasattr(mav, "MAVLINK_CFLAG_SYSID32")
    assert buf.hex() == GOLDEN[(SYSID_SMALL, TARGET_SMALL)]


def test_heartbeat_sysid32_no_target(mav):
    """messages without target fields work with 32 bit sysids"""
    link = mav.MAVLink(FakeFile(), srcSystem=100000, srcComponent=1)
    msg = mav.MAVLink_heartbeat_message(2, 3, 81, 0, 4, 3)
    buf = msg.pack(link)
    rx = mav.MAVLink(FakeFile())
    m = rx.parse_char(buf)
    assert m is not None and m.get_type() == "HEARTBEAT"
    assert m.get_srcSystem() == 100000
    assert m.get_target_system() is None
    assert m.get_target_component() is None


@pytest.mark.parametrize("sysid", [SYSID_SMALL, SYSID_BIG])
@pytest.mark.parametrize("target", [0, TARGET_SMALL, 255, 256, 0xFFFFFFFF])
@pytest.mark.parametrize("signed", [False, True])
def test_field_target(mav, sysid, target, signed):
    link = mav.MAVLink(FakeFile(), srcSystem=sysid, srcComponent=11)
    link.signing.secret_key = SIGNING_KEY
    link.signing.sign_outgoing = signed
    link.signing.timestamp = 1000
    msg = mav.MAVLink_command_long_message(target, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7)
    buf = msg.pack(link)
    rx = mav.MAVLink(FakeFile())
    rx.signing.secret_key = SIGNING_KEY if signed else None
    rx.signing.timestamp = 999
    decoded = rx.parse_char(buf)
    assert decoded.get_target_system() == target
    assert decoded.get_target_component() == 250
    assert decoded.get_srcSystem() == sysid
    assert decoded.get_header().incompat_flags == (4 if target > 255 else 0) | (2 if sysid > 255 else 0) | int(signed)
    # Repacking retains the decoded target field.
    link.signing.timestamp = 1000
    assert decoded.pack(link) == buf
    if sysid > 255 or target > 255:
        with pytest.raises(mav.MAVError):
            msg.pack(link, force_mavlink1=True)
    msg.target_system = 0
    assert not msg.pack(link)[2] & mav.MAVLINK_IFLAG_TARGET32


def test_payload_target_can_change_after_packing(mav):
    link = mav.MAVLink(FakeFile())
    msg = mav.MAVLink_command_long_message(TARGET_BIG, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7)
    msg.pack(link)
    msg.target_system = TARGET_SMALL
    decoded = mav.MAVLink(FakeFile()).parse_char(msg.pack(link))
    assert decoded.target_system == TARGET_SMALL
    assert not decoded.get_header().incompat_flags & mav.MAVLINK_IFLAG_TARGET32


@pytest.mark.parametrize('target', [0, 7, 256, 0xFFFFFFFF])
def test_retarget_received_command(mav, target):
    link = mav.MAVLink(FakeFile(), srcSystem=42, srcComponent=11)
    message = mav.MAVLink_command_long_message(256, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7)
    received = mav.MAVLink(FakeFile()).parse_char(message.pack(link))
    received.target_system = target
    received.target_component = 19
    forwarded = mav.MAVLink(FakeFile()).parse_char(received.pack(link))
    assert bool(forwarded.get_header().incompat_flags & mav.MAVLINK_IFLAG_TARGET32) == (target > 255)
    assert forwarded.get_target_system() == target
    assert forwarded.get_target_component() == 19


@pytest.mark.parametrize("target", [0, 255, 256, 0xFFFFFFFF])
def test_equivalent_target_field(mav, target):
    link = mav.MAVLink(FakeFile(), srcSystem=42)
    message = mav.MAVLink_manual_control_message(target, 1, 2, 3, 4, 5)
    decoded = mav.MAVLink(FakeFile()).parse_char(message.pack(link))
    assert decoded.target == target
    assert decoded.get_target_system() == target
    assert decoded.get_header().incompat_flags == (4 if target > 255 else 0)


def test_no_targetless_sender_api(mav):
    message = mav.MAVLink_heartbeat_message(2, 3, 81, 0, 4, 3)
    assert not hasattr(message, 'set_target')
    assert message.pack(mav.MAVLink(FakeFile(), srcSystem=0xFFFFFFFF))[2] == 2
