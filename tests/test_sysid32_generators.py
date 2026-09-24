"""Execute generated parsers against extended frames and embedded valid packets.

Optional language tools are detected at runtime. See tests/sysid32/README.md.
"""
import ctypes
import ctypes.util
import hashlib
import os
from pathlib import Path
import shutil
import struct
import subprocess

import pytest

from pymavlink.generator import mavgen
from pymavlink.generator.mavcrc import x25crc

ROOT = Path(__file__).resolve().parents[1]
RESOURCES = Path(__file__).parent / 'sysid32'
XML = Path(__file__).parent / 'snapshottests/resources/common.xml'


def run(args, **kwargs):
    result = subprocess.run([str(a) for a in args], text=True, capture_output=True, **kwargs)
    if result.returncode != 0:
        pytest.fail(result.stdout + result.stderr, pytrace=False)
    return result.stdout


def tool(name):
    path = shutil.which(name)
    if path is None:
        pytest.skip('%s is not installed' % name)
    return path


def generate(directory, language, protocol='2.0', xml=XML):
    assert mavgen.mavgen(mavgen.Opts(output=str(directory), language=language,
                                  wire_protocol=protocol, validate=False), [str(xml)])
    return directory


def frame(flags=0, payload=None, source=42, target=7, msgid=0, extra=50, v1=False):
    if payload is None:
        payload = struct.pack('<IBBBBB', 0, 2, 3, 81, 4, 3)
    if v1:
        header = struct.pack('<BBBBBB', 254, len(payload), 0, source, 11, msgid)
    else:
        header = struct.pack('<BBBBB', 253, len(payload), flags, 0, 0)
        header += struct.pack('<I' if flags & 2 else '<B', source)
        header += bytes([11]) + msgid.to_bytes(3, 'little')
        if flags & 4:
            header += struct.pack('<I', target)
    packet = header + payload
    crc = x25crc(packet[1:])
    crc.accumulate(bytes([extra]))
    packet += struct.pack('<H', crc.crc)
    if flags & 1:
        packet += struct.pack('<BQ', 3, 1000)[:7]
        packet += hashlib.sha256(bytes([42] * 32) + packet).digest()[:6]
    return packet


@pytest.fixture(scope='module')
def streams(tmp_path_factory):
    directory = tmp_path_factory.mktemp('sysid32-streams')
    legacy = frame()
    legacy1 = frame(v1=True)
    # A rejected payload contains CRC-valid v1 and v2 packets. Neither may leak
    # out of a parser as a received message, even with fragmented input.
    for flags in [2, 3, 4, 5, 6, 7, 128, 129]:
        for length in [0, 80, 255]:
            payload = (legacy + legacy1).ljust(length, b'\xfd')[:length]
            unsupported = frame(flags, payload, source=0xFEDCBA98 if flags & 2 else 42,
                                target=0xABCDEF12, msgid=76, extra=152)
            name = '%d-%d' % (flags, length)
            (directory / (name + '.frame')).write_bytes(unsupported)
            (directory / (name + '.v2')).write_bytes(unsupported + legacy)
            (directory / (name + '.v1')).write_bytes(unsupported + legacy1)
    (directory / 'legacy').write_bytes(legacy)
    return directory


def test_c_field_target(tmp_path):
    headers = generate(tmp_path / 'c', 'C')
    exe = tmp_path / 'check'
    run([tool('gcc'), '-std=c99', '-Wall', '-Werror', '-Wno-address-of-packed-member',
         '-I' + str(headers), RESOURCES / 'target.c', '-o', exe])
    actual = run([exe]).splitlines()
    expected = []
    for source in [42, 0xABCDEF12]:
        for target in [0, 7, 255, 256, 0xFFFFFFFF]:
            for signed in [0, 1]:
                flags = (4 if target > 255 else 0) | (2 if source > 255 else 0) | signed
                payload = struct.pack('<7fHBBB', 1, 2, 3, 4, 5, 6, 7, 300, target if target <= 255 else 255, 250, 1)
                expected.append(frame(flags, payload, source=source, target=target, msgid=76, extra=152).hex())
    assert actual == expected


@pytest.mark.parametrize('separate_helpers', [False, True])
@pytest.mark.parametrize('aligned_fields', [False, True])
def test_c_generic_target_getter(tmp_path, separate_helpers, aligned_fields):
    headers = generate(tmp_path / 'c', 'C')
    exe = tmp_path / 'get-target'
    flags = ['-DMAVLINK_ALIGNED_FIELDS=%d' % aligned_fields,
             '-fsanitize=undefined', '-fno-sanitize-recover=all']
    sources = [RESOURCES / 'get-target.c']
    if separate_helpers:
        flags.append('-DMAVLINK_SEPARATE_HELPERS')
        implementation = tmp_path / 'helpers.c'
        implementation.write_text('#include "common/mavlink.h"\n#include "mavlink_helpers.h"\n')
        sources.append(implementation)
    run([tool('gcc'), '-std=c99', '-Wall', '-Werror', '-Wno-address-of-packed-member',
         *flags, '-I' + str(headers), *sources, '-lm', '-o', exe])
    run([exe])


@pytest.mark.parametrize('protocol', ['1.0', '2.0'])
def test_c_no_per_message_target_system_getters(tmp_path, protocol):
    headers = generate(tmp_path / 'c', 'C', protocol=protocol)
    for header in (headers / 'common').glob('mavlink_msg_*.h'):
        assert '_get_target_system(' not in header.read_text()
    # MANUAL_CONTROL names its system target "target", so name matching alone
    # would leave this unsafe getter behind. Both removed APIs must fail to compile.
    for getter in ['mavlink_msg_command_int_get_target_system',
                   'mavlink_msg_manual_control_get_target']:
        source = tmp_path / 'removed-getter.c'
        source.write_text('#include "common/mavlink.h"\n'
                          'uint32_t get(const mavlink_message_t *msg) { return %s(msg); }\n' % getter)
        result = subprocess.run([tool('gcc'), '-std=c99', '-Werror=implicit-function-declaration',
                                 '-Wno-address-of-packed-member', '-I' + str(headers),
                                 '-fsyntax-only', str(source)], text=True, capture_output=True)
        assert result.returncode != 0
        assert getter in result.stderr
        assert 'implicit declaration' in result.stderr
