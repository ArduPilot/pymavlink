"""Build and run the extended header C regression tests.

The sources live in generator/C/test/mutation_parser/. They are compiled
against C headers generated from common.xml for MAVLink 2.0 and run with
UBSan enabled. parse_runtime_test.c is built twice: plain, and with
-DMAVLINK_CHECK_MESSAGE_LENGTH, because some checks only apply to one build.
"""
from pathlib import Path
import shutil
import subprocess

import pytest

from pymavlink.generator import mavgen

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ROOT / 'generator/C/test/mutation_parser'
XML = Path(__file__).parent / 'snapshottests/resources/common.xml'


def tool(name):
    path = shutil.which(name)
    if path is None:
        pytest.skip('%s is not installed' % name)
    return path


def run(args):
    result = subprocess.run([str(a) for a in args], text=True, capture_output=True)
    if result.returncode != 0:
        pytest.fail(result.stdout + result.stderr, pytrace=False)
    return result.stdout


@pytest.fixture(scope='module')
def headers(tmp_path_factory):
    directory = tmp_path_factory.mktemp('c-parser-headers')
    assert mavgen.mavgen(mavgen.Opts(output=str(directory), language='C',
                                     wire_protocol='2.0', validate=False), [str(XML)])
    return directory


def build_and_run(headers, source, exe, *defines):
    run([tool('gcc'), '-std=c99', '-Wall', '-Werror', '-O0',
         '-Wno-address-of-packed-member',
         '-fsanitize=undefined', '-fno-sanitize-recover=all',
         *defines, '-I' + str(headers), SOURCES / source, '-o', exe])
    return run([exe])


@pytest.mark.parametrize('check_length', [False, True])
def test_parse_runtime(tmp_path, headers, check_length):
    defines = ['-DMAVLINK_CHECK_MESSAGE_LENGTH'] if check_length else []
    output = build_and_run(headers, 'parse_runtime_test.c', tmp_path / 'parse', *defines)
    assert 'PASS' in output


def test_convenience_sender(tmp_path, headers):
    output = build_and_run(headers, 'convenience_sender_test.c', tmp_path / 'sender')
    assert 'PASS' in output
