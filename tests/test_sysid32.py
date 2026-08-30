#!/usr/bin/env python3

"""
Test 32 bit system IDs (MAVLINK_IFLAG_SYSID32) and extended header
targeting (MAVLINK_IFLAG_TARGETTED_SYSID32)
"""

import importlib.util
import pathlib
import sys

import pytest

from pymavlink.generator import mavgen

SYSID_SMALL = 42
SYSID_BIG = 0x0A000001  # 10.0.0.1
TARGET_SMALL = 7
TARGET_BIG = 0x0A000002  # 10.0.0.2

# golden frames produced by the C implementation
# (generator/C/include_v2.0) for COMMAND_LONG packed with
# (sysid, 11, target, 250, 300, 1, 1.0 ... 7.0) at seq 0.
# Cross-language wire compatibility depends on these matching exactly.
GOLDEN = {
    (SYSID_SMALL, TARGET_SMALL): "fd210001002a0b4c00000000803f0000004000004040000080400000a0400000c0400000e0402c0107fa01190a",
    (SYSID_SMALL, TARGET_BIG): "fd210401002a0b4c00000200000afa0000803f0000004000004040000080400000a0400000c0400000e0402c0100fa018ffb",
    (SYSID_BIG, TARGET_SMALL): "fd210201000100000a0b4c00000000803f0000004000004040000080400000a0400000c0400000e0402c0107fa01da8f",
    (SYSID_BIG, TARGET_BIG): "fd210601000100000a0b4c00000200000afa0000803f0000004000004040000080400000a0400000c0400000e0402c0100fa0187b5",
}
# signed with key bytes([42]*32), link_id 3, timestamp 1000
GOLDEN_SIGNED = (
    "fd210701000100000a0b4c00000200000afa0000803f0000004000004040000080400000a0"
    "400000c0400000e0402c0100fa01d22103e80300000000b7ef98e9a4c8"
)
SIGNING_KEY = bytes([42] * 32)


class FakeFile:
    def write(self, b):
        pass


@pytest.fixture(scope="module")
def mav(tmp_path_factory):
    """generate the v2.0 common dialect from the in-tree generator"""
    xml = pathlib.Path(__file__).parent / "snapshottests" / "resources" / "common.xml"
    out = tmp_path_factory.mktemp("sysid32") / "mavcommon.py"
    mavgen.mavgen(
        mavgen.Opts(output=str(out), language="Python3", wire_protocol="2.0", validate=False),
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
    assert ((hdr.incompat_flags & mav.MAVLINK_IFLAG_TARGETTED_SYSID32) != 0) == (target > 255)
    assert hdr.compat_flags == mav.MAVLINK_CFLAG_SYSID32

    # payload must not contain the >255 target (it goes in the header)
    if target > 255:
        payload = m.get_payload()
        if len(payload) > 30:
            assert payload[30] == 0


@pytest.mark.parametrize("sysid", [SYSID_SMALL, SYSID_BIG])
@pytest.mark.parametrize("target", [TARGET_SMALL, TARGET_BIG])
def test_golden_frames(mav, sysid, target):
    """frames must match the C implementation byte for byte"""
    buf = pack_command_long(mav, sysid, target)
    assert buf == bytes.fromhex(GOLDEN[(sysid, target)])


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
    """a small-ID frame differs from a MAVLink 2.0 frame only in
    compat_flags and CRC, and legacy frames (compat 0) still parse"""
    buf = pack_command_long(mav, SYSID_SMALL, TARGET_SMALL)
    assert buf[2] == 0  # incompat_flags
    assert buf[3] == mav.MAVLINK_CFLAG_SYSID32

    # rebuild as a legacy frame with compat_flags 0
    legacy = bytearray(buf)
    legacy[3] = 0
    crc = mav.x25crc(legacy[1:-2])
    crc.accumulate(bytes([mav.MAVLink_command_long_message.crc_extra]))
    legacy[-2] = crc.crc & 0xFF
    legacy[-1] = crc.crc >> 8

    rx = mav.MAVLink(FakeFile())
    m = rx.parse_char(bytes(legacy))
    assert m is not None and m.get_type() == "COMMAND_LONG"
    assert m.get_srcSystem() == SYSID_SMALL
    assert m.target_system == TARGET_SMALL

    # only compat byte and CRC differ
    assert bytes(legacy[:3]) == buf[:3]
    assert bytes(legacy[4:-2]) == buf[4:-2]


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
