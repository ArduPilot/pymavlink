"""Generated C getters on packed, zero-trimmed MAVLink 2 messages.

See https://github.com/ArduPilot/pymavlink/issues/1142
"""
from pathlib import Path
import shutil
import subprocess

import pytest

from pymavlink.generator import mavgen

SOURCE = Path(__file__).parent / 'c_getters' / 'msg_len.c'
XML = Path(__file__).parent / 'snapshottests/resources/common.xml'


@pytest.mark.parametrize('aligned_fields', [False, True])
def test_c_getters_ignore_checksum(tmp_path, aligned_fields):
    gcc = shutil.which('gcc')
    if gcc is None:
        pytest.skip('gcc is not installed')
    headers = tmp_path / 'c'
    assert mavgen.mavgen(mavgen.Opts(output=str(headers), language='C',
                                     wire_protocol='2.0', validate=False), [str(XML)])
    exe = tmp_path / 'msg_len'
    subprocess.run([gcc, '-std=c99', '-Wall', '-Werror', '-Wno-address-of-packed-member',
                    '-DMAVLINK_ALIGNED_FIELDS=%d' % aligned_fields,
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I' + str(headers), str(SOURCE), '-o', str(exe)],
                   check=True, capture_output=True, text=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
