#!/usr/bin/env python3
"""
Python-generator pack/unpack round-trip tests for the TEST_TYPES message,
including the float16_t field/array (a raw IEEE-754 binary16 value).

Unlike the C and C++11 generators, the Python generator has no
auto-generated per-type testsuite, so this exercises the mavgen_python.py
struct-format mapping directly.
"""

import importlib
import math
import shutil
import struct
import sys
from pathlib import Path

try:
    from pymavlink.generator import mavgen
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from generator import mavgen


def _generate_test_dialect():
    """Generate the "test" dialect's Python module into a scratch dir and import it."""
    xml_filepath = Path(__file__).resolve().parents[2] / "message_definitions" / "v1.0" / "test.xml"
    output_dir = Path(__file__).resolve().parents[1] / ".tmp" / "python-roundtrip-generator"
    shutil.rmtree(output_dir, ignore_errors=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / "test_dialect.py"

    ok = mavgen.mavgen(
        mavgen.Opts(
            output=str(output_file),
            language="Python3",
            wire_protocol="2.0",
            validate=False,
        ),
        [str(xml_filepath)],
    )
    assert ok is True

    sys.path.insert(0, str(output_dir))
    try:
        module = importlib.import_module("test_dialect")
    finally:
        sys.path.remove(str(output_dir))
    return module


def _make_message(dialect, f16, f16_array):
    return dialect.MAVLink_test_types_message(
        c=b"A", s=b"hello", u8=42, u16=1234, f16=f16, u32=99999, u64=123456789012345,
        s8=-12, s16=-1234, s32=-99999, s64=-123456789012345, f=3.14, d=2.71828,
        u8_array=[1, 2, 3], u16_array=[10, 20, 30], f16_array=f16_array,
        u32_array=[100, 200, 300], u64_array=[1000, 2000, 3000],
        s8_array=[-1, -2, -3], s16_array=[-10, -20, -30], s32_array=[-100, -200, -300],
        s64_array=[-1000, -2000, -3000], f_array=[1.1, 2.2, 3.3], d_array=[4.4, 5.5, 6.6],
    )


def test_float16_roundtrip_exact_values():
    """float16_t scalar and array fields round-trip exactly for half-representable values."""
    dialect = _generate_test_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)

    values = [0.0, -0.0, 1.0, -1.0, 1.5, -2.25, 3.0, 0.5, 100.0, -100.0, 65504.0]
    for v in values:
        msg = _make_message(dialect, v, [v, 0.0, -v])
        buf = msg.pack(mav)
        decoded = mav.decode(bytearray(buf))
        assert decoded.f16 == v, f"scalar f16 mismatch for {v!r}"
        assert decoded.f16_array == [v, 0.0, -v], f"array f16 mismatch for {v!r}"
        # every other field must be unaffected by the new type's presence
        assert decoded.u16 == 1234
        assert decoded.s == "hello"
        assert decoded.f == struct.unpack("<f", struct.pack("<f", 3.14))[0]


def test_float16_roundtrip_special_values():
    """float16_t handles +-inf and NaN the same way the struct 'e' format does."""
    dialect = _generate_test_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)

    msg = _make_message(dialect, math.inf, [math.nan, -math.inf, 65504.0])
    buf = msg.pack(mav)
    decoded = mav.decode(bytearray(buf))

    assert math.isinf(decoded.f16) and decoded.f16 > 0
    assert math.isnan(decoded.f16_array[0])
    assert math.isinf(decoded.f16_array[1]) and decoded.f16_array[1] < 0
    assert decoded.f16_array[2] == 65504.0


def test_float16_precision_loss_matches_struct_e_format():
    """Values not exactly representable in half precision truncate the same way
    Python's own struct 'e' format truncates them (i.e. mavgen's format-char
    mapping for float16_t really is 'e', not something silently different)."""
    dialect = _generate_test_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)

    v = 1.0 / 3.0
    expected = struct.unpack("<e", struct.pack("<e", v))[0]
    assert expected != v  # sanity: this value does lose precision in half float

    msg = _make_message(dialect, v, [v, v, v])
    buf = msg.pack(mav)
    decoded = mav.decode(bytearray(buf))
    assert decoded.f16 == expected


def test_float16_wire_bytes_match_c_bit_patterns():
    """Cross-check the raw on-the-wire bytes for the f16 field against
    known-correct IEEE-754 binary16 bit patterns (the same patterns
    independently verified against the C mavlink_float_to_float16() helper
    and against Python's own struct 'e' format), to catch any
    endianness/struct-format mismatch between the Python and C generators.

    Field offset within the payload is fixed by TEST_TYPES' wire-order
    sort (mavparse.py sorts fields by decreasing type_length): f16 is a
    2-byte field, landing right after the 2-byte u16 field at payload
    offset 146. The payload itself starts 10 bytes into a MAVLink2 frame
    (STX, LEN, INCOMPAT_FLAGS, COMPAT_FLAGS, SEQ, SYSID, COMPID, 3-byte
    MSGID).
    """
    dialect = _generate_test_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)

    known_patterns = {
        1.0: 0x3C00,
        1.5: 0x3E00,
        -2.25: 0xC080,
        3.0: 0x4200,
        0.5: 0x3800,
        100.0: 0x5640,
    }
    payload_offset = 10
    f16_wire_offset = 146
    for value, expected_bits in known_patterns.items():
        msg = _make_message(dialect, value, [0.0, 0.0, 0.0])
        buf = msg.pack(mav)
        f16_bytes = buf[payload_offset + f16_wire_offset: payload_offset + f16_wire_offset + 2]
        assert struct.unpack("<H", f16_bytes)[0] == expected_bits
