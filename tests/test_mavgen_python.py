#!/usr/bin/env python3
"""
Tests for the Python generator.
"""

import importlib.util
from pathlib import Path
import shutil
import sys

try:
    from pymavlink.generator import mavgen
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from generator import mavgen


_common_dialect = None


def common_dialect():
    """Generate the common dialect once and import it as a module."""
    global _common_dialect
    if _common_dialect is not None:
        return _common_dialect

    xml_filepath = Path(__file__).parent / "snapshottests" / "resources" / "common.xml"
    output_dir = Path(__file__).resolve().parents[1] / ".tmp" / "python-generator"
    shutil.rmtree(output_dir, ignore_errors=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_filepath = output_dir / "generated_common.py"

    ok = mavgen.mavgen(
        mavgen.Opts(
            output=str(output_filepath),
            language="Python3",
            wire_protocol="2.0",
            validate=False,
        ),
        [str(xml_filepath)],
    )
    assert ok is True

    spec = importlib.util.spec_from_file_location("generated_common", output_filepath)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _common_dialect = module
    return module


def test_char_array_field_accepts_str_bytes_and_bytearray():
    """char[] fields accept str as well as bytes/bytearray. See issue #1225."""
    dialect = common_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    expected = dialect.MAVLink_statustext_message(6, b"hello world").pack(mav)

    for text in ("hello world", b"hello world", bytearray(b"hello world")):
        msg = dialect.MAVLink_statustext_message(6, text)
        assert msg.text == "hello world"
        assert msg._text_raw == b"hello world"
        assert msg.pack(mav) == expected


def test_param_id_accepts_str_bytes_and_bytearray():
    dialect = common_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    expected = dialect.MAVLink_param_value_message(b"P", 1.0, 9, 1, 0).pack(mav)

    for param_id in ("P", b"P", bytearray(b"P")):
        msg = dialect.MAVLink_param_value_message(param_id, 1.0, 9, 1, 0)
        assert msg.param_id == "P"
        assert msg.pack(mav) == expected


def test_char_array_field_is_truncated_at_first_null():
    dialect = common_dialect()
    msg = dialect.MAVLink_statustext_message(6, b"abc\x00def")

    assert msg.text == "abc"
    assert msg._text_raw == b"abc\x00def"


def test_char_array_field_replaces_non_ascii():
    dialect = common_dialect()
    msg = dialect.MAVLink_statustext_message(6, "café")

    assert msg.text == "caf?"
    assert msg._text_raw == b"caf?"


def test_statustext_round_trips_through_the_parser():
    dialect = common_dialect()
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    buf = dialect.MAVLink_statustext_message(6, "round trip").pack(mav)

    decoded = dialect.MAVLink(None).decode(bytearray(buf))

    assert decoded.get_type() == "STATUSTEXT"
    assert decoded.text == "round trip"
