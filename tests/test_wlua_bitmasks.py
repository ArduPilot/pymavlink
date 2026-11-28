"""Exercise generated wide bitfields in Wireshark, not just their Lua text."""
import importlib.util
import os
import re
import shutil
import struct
import subprocess

import pytest

from pymavlink.generator import mavgen


BITS = (0, 32, 53, 63)


@pytest.fixture(scope="module")
def wide_bitfield(tmp_path_factory):
    tshark = shutil.which("tshark")
    if tshark is None:
        pytest.skip("tshark (Wireshark CLI) not found")
    version = subprocess.check_output([tshark, "--version"], text=True)
    match = re.search(r"Wireshark\) (\d+)\.(\d+)", version)
    if match and tuple(map(int, match.groups())) < (4, 2):
        pytest.skip("64-bit boolean masks require Wireshark 4.2 or later")
    directory = tmp_path_factory.mktemp("wide-bitfield")
    definition = directory / "wide.xml"
    entries = "".join('<entry value="{}" name="BIT{}"/>'.format(1 << bit, bit) for bit in BITS)
    definition.write_text('''<mavlink><version>3</version><enums>
        <enum name="WIDE_FLAGS" bitmask="true">{}</enum></enums><messages>
        <message id="200" name="FLAGS"><description>Wide bitfield regression.</description>
        <field type="uint64_t" name="flags" enum="WIDE_FLAGS">Flags</field>
        </message></messages></mavlink>'''.format(entries))
    lua = directory / "wide.lua"
    python = directory / "wide.py"
    for language, output in (("WLua", lua), ("Python3", python)):
        assert mavgen.mavgen(mavgen.Opts(output=str(output), language=language, wire_protocol="2.0"),
                             [str(definition)])
    spec = importlib.util.spec_from_file_location("wide", python)
    dialect = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dialect)
    return tshark, lua, dialect


@pytest.mark.parametrize("value", [0, 1, 1 << 32, 1 << 53, 1 << 63, (1 << 63) | 1,
                                   sum(1 << bit for bit in BITS)])
def test_wide_bitfield(wide_bitfield, tmp_path, value):
    tshark, lua, dialect = wide_bitfield
    packet = dialect.MAVLink_flags_message(value).pack(dialect.MAVLink(None))
    # USER0 is a link type explicitly registered by the generated dissector.
    capture = tmp_path / "flags.pcap"
    capture.write_bytes(struct.pack("<IHHIIII", 0xa1b2c3d4, 2, 4, 0, 0, 65535, 147)
                        + struct.pack("<IIII", 0, 0, len(packet), len(packet)) + packet)
    fields = ["mavlink_proto.FLAGS_flags.BIT{}".format(bit) for bit in BITS]
    command = [tshark, "-n", "-r", str(capture), "-Xlua_script:" + str(lua), "-Tfields", "-Eseparator=,"]
    for field in fields:
        command.extend(["-e", field])
    env = dict(os.environ, WIRESHARK_CONFIG_DIR=str(tmp_path / "empty"), HOME=str(tmp_path / "empty"))
    result = subprocess.run(command, capture_output=True, text=True, env=env, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "Lua: Error" not in result.stderr, result.stderr
    fields = result.stdout.strip().split(",")
    assert all(field in ("True", "False", "1", "0") for field in fields), result.stdout
    assert [field in ("True", "1") for field in fields] == [bool(value & (1 << bit)) for bit in BITS]
