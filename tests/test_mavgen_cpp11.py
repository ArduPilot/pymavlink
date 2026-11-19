#!/usr/bin/env python3
"""
Tests for the C++11 generator.
"""

from pathlib import Path
import shutil
import sys

try:
    from pymavlink.generator import mavgen
    from pymavlink.generator import mavgen_cpp11
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from generator import mavgen
    from generator import mavgen_cpp11


def test_enum_remove_prefix():
    assert mavgen_cpp11.enum_remove_prefix("MAV_CMD", "MAV_CMD_NAV_WAYPOINT") == "NAV_WAYPOINT"
    assert mavgen_cpp11.enum_remove_prefix("MAV_FRAME", "MAV_FRAME_GLOBAL") == "GLOBAL"
    # a remainder starting with a digit gets the last prefix component back
    assert mavgen_cpp11.enum_remove_prefix("MAV_SYS_STATUS_SENSOR", "MAV_SYS_STATUS_SENSOR_3D_GYRO") == "SENSOR_3D_GYRO"
    # an entry whose name is a prefix of the enum name must keep its last component
    assert mavgen_cpp11.enum_remove_prefix("SOME_ENUM_NAME", "SOME_ENUM") == "ENUM"
    assert mavgen_cpp11.enum_remove_prefix("SOME_ENUM_NAME", "SOME_ENUM_NAME") == "NAME"
    assert mavgen_cpp11.enum_remove_prefix("SOME_ENUM_NAME", "SOME") == "SOME"


def test_cpp11_generator_enum_entry_is_prefix_of_enum_name(tmp_path):
    xml_filepath = tmp_path / "test.xml"
    xml_filepath.write_text("""<?xml version="1.0"?>
<mavlink>
  <version>3</version>
  <enums>
    <enum name="SOME_ENUM_NAME">
      <entry value="0" name="SOME_ENUM"/>
      <entry value="1" name="SOME_ENUM_NAME_VALUE"/>
    </enum>
  </enums>
  <messages>
  </messages>
</mavlink>
""")
    output_dir = Path(__file__).resolve().parents[1] / ".tmp" / "cpp11-generator"
    shutil.rmtree(output_dir, ignore_errors=True)
    output_dir.mkdir(parents=True, exist_ok=True)

    ok = mavgen.mavgen(
        mavgen.Opts(
            output=str(output_dir),
            language="C++11",
            wire_protocol="2.0",
            validate=False,
        ),
        [str(xml_filepath)],
    )

    assert ok is True

    header = (output_dir / "test" / "test.hpp").read_text(encoding="utf-8")
    assert "enum class SOME_ENUM_NAME" in header
    assert "    ENUM=0," in header
    assert "    VALUE=1," in header
