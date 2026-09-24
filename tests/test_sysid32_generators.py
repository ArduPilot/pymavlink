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


def test_java_rejects_extensions(tmp_path, streams):
    java = generate(tmp_path / 'java', 'Java')
    sources = list(java.rglob('*.java'))
    run([tool('javac'), '-d', tmp_path / 'classes', *sources, RESOURCES / 'Reject.java'])
    # Only the fixture's extension may change; a parent directory can contain .v2.
    java_streams = tmp_path / 'parent.v2' / 'frames'
    shutil.copytree(streams, java_streams)
    run([tool('java'), '-cp', tmp_path / 'classes', 'Reject', java_streams])


def test_cs_rejects_extensions(tmp_path, streams):
    cs = generate(tmp_path / 'cs', 'CS')
    exe = tmp_path / 'reject.exe'
    run([tool('mcs'), '-unsafe', '-out:' + str(exe), *cs.glob('*.cs'), RESOURCES / 'Reject.cs'])
    run([tool('mono'), exe, streams])


def node_environment():
    env = os.environ.copy()
    modules = ROOT / 'generator/javascript/node_modules'
    missing = [name for name in ('jspack', 'long', 'underscore')
               if not (modules / name / 'package.json').is_file()]
    if missing:
        pytest.skip('Missing JavaScript dependencies: %s (npm install in generator/javascript)' % ', '.join(missing))
    env['NODE_PATH'] = str(modules) + os.pathsep + env.get('NODE_PATH', '')
    return env


@pytest.mark.parametrize('missing', ['jspack', 'long', 'underscore'])
def test_node_environment_missing_dependencies(tmp_path, monkeypatch, missing):
    # A clean checkout has vendored jspack/long symlinks, but no underscore.
    modules = tmp_path / 'generator/javascript/node_modules'
    modules.mkdir(parents=True)
    for name in ('jspack', 'long', 'underscore'):
        if name != missing:
            (modules / name).symlink_to(ROOT / 'generator/javascript/node_modules' / name)
    monkeypatch.setitem(node_environment.__globals__, 'ROOT', tmp_path)
    with pytest.raises(pytest.skip.Exception, match='Missing JavaScript dependencies: .*' + missing):
        node_environment()


@pytest.mark.parametrize('language', ['JavaScript', 'JavaScript_Stable'])
def test_stable_javascript_rejects_extensions(tmp_path, streams, language):
    generated = generate(tmp_path / 'mavlink.js', language)
    run([tool('node'), RESOURCES / 'reject-stable.js', generated, streams], env=node_environment())


def test_nextgen_rejects_unknown_incompat_flags(tmp_path, streams):
    generated = generate(tmp_path / 'mavlink.js', 'JavaScript_NextGen')
    run([tool('node'), RESOURCES / 'reject-nextgen.js', generated, streams], env=node_environment())


@pytest.mark.parametrize('protocol,mask', [('1.0', 0), ('2.0', 7)])
def test_nextgen_protocol_mask(tmp_path, protocol, mask):
    generated = generate(tmp_path / 'mavlink.js', 'JavaScript_NextGen', protocol)
    version = '10' if protocol == '1.0' else '20'
    script = ("const m = require(process.argv[1]).mavlink%s; "
              "require('assert').strictEqual(m.MAVLINK_IFLAG_MASK, %u);" % (version, mask))
    run([tool('node'), '-e', script, generated], env=node_environment())


def test_nextgen_field_target(tmp_path):
    generated = generate(tmp_path / 'mavlink.js', 'JavaScript_NextGen')
    actual = run([tool('node'), RESOURCES / 'target.js', generated], env=node_environment()).splitlines()
    expected = []
    for source in [42, 0xABCDEF12]:
        for target in [0, 7, 255, 256, 0xFFFFFFFF]:
            for signed in [0, 1]:
                flags = (4 if target > 255 else 0) | (2 if source > 255 else 0) | signed
                payload = struct.pack('<7fHBBB', 1, 2, 3, 4, 5, 6, 7, 300, target if target <= 255 else 255, 250, 1)
                expected.append(frame(flags, payload, source=source, target=target, msgid=76, extra=152).hex())
    assert actual == expected


def test_cpp_rejects_extensions(tmp_path, streams):
    headers = generate(tmp_path / 'cpp', 'C++11')
    exe = tmp_path / 'reject'
    run([tool('g++'), '-std=c++11', '-I' + str(headers), RESOURCES / 'reject.cpp', '-o', exe])
    run([exe, streams])


def lua_run(script):
    """Use the shared library where only a Lua 5.1 executable is installed."""
    path = ctypes.util.find_library('lua5.4') or ctypes.util.find_library('lua5.3')
    if not path:
        pytest.skip('Lua 5.3 or newer is required')
    lua = ctypes.CDLL(path)
    lua.luaL_newstate.restype = ctypes.c_void_p
    lua.luaL_openlibs.argtypes = [ctypes.c_void_p]
    lua.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lua.lua_pcallk.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ssize_t, ctypes.c_void_p]
    lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    lua.lua_tolstring.restype = ctypes.c_char_p
    lua.lua_close.argtypes = [ctypes.c_void_p]
    state = lua.luaL_newstate()
    try:
        lua.luaL_openlibs(state)
        result = lua.luaL_loadstring(state, script.encode())
        if result == 0:
            result = lua.lua_pcallk(state, 0, 0, 0, 0, None)
        assert result == 0, lua.lua_tolstring(state, -1, None)
    finally:
        lua.lua_close(state)


def test_lua_layout_and_flags(tmp_path):
    modules = generate(tmp_path / 'modules', 'Lua')
    for flags in range(8):
        source = 0xABCDEF12 if flags & 2 else 42
        target = 0xFEDCBA98 if flags & 2 else 7
        wire = frame(flags, source=source, target=target)
        header_len = 10 + (3 if flags & 2 else 0) + (4 if flags & 4 else 0)
        crc = wire[header_len + 9:header_len + 11]
        for storage in [16, 256, 264]:
            data = crc + struct.pack('<BBBBBIB', 253, 9, flags, 0, 0, source, 11) + bytes(3)
            data += wire[header_len:header_len + 9].ljust(storage, b'\0') + bytes(15) + struct.pack('<I', target)
            (tmp_path / ('%d-%d.bin' % (flags, storage))).write_bytes(data)
    lua_run("package.path = %r .. '/?.lua;' .. package.path\nROOT = %r\n" % (str(modules), str(tmp_path)) + (RESOURCES / 'layout.lua').read_text())




@pytest.mark.parametrize('protocol', ['1.0', '2.0'])
def test_ada_rejects_extensions(tmp_path, streams, protocol):
    compiler = tool('gnatmake')
    generated = generate(tmp_path / 'ada', 'Ada', protocol)
    version = 'V1' if protocol == '1.0' else 'V2'
    source = (RESOURCES / 'reject_ada.adb').read_text().replace('@VERSION@', version)
    source = source.replace('@MESSAGE_ID@', 'Get_Msg_Id' if version == 'V1' else 'Get_Message_Id')
    source = source.replace('@SYSTEM_ID@', 'Get_Target_System_Id' if version == 'V1' else 'Get_Message_System_Id')
    (generated / 'reject_ada.adb').write_text(source)
    run([compiler, '-gnat2022', '-gnata', '-q', 'reject_ada.adb'], cwd=generated)
    for stream in sorted(streams.glob('*.' + version.lower())):
        run([generated / 'reject_ada', stream])


def test_typescript_rejects_extensions(tmp_path, streams):
    modules = Path(os.environ.get('MAVLINK_TYPESCRIPT_NODE_MODULES', RESOURCES / 'node_modules'))
    compiler = modules / 'typescript/bin/tsc'
    if not compiler.exists():
        pytest.skip('npm install in tests/sysid32 to enable TypeScript runtime tests')
    generated = generate(tmp_path / 'typescript', 'TypeScript')
    (generated / 'node_modules').symlink_to(modules, target_is_directory=True)
    run([tool('node'), compiler, '--skipLibCheck', '--target', 'es2017', '--module', 'commonjs',
         '--strict', '--outDir', generated / 'compiled', generated / 'message-registry.ts'], cwd=tmp_path)
    run([tool('node'), RESOURCES / 'reject-typescript.js', generated / 'compiled/message-registry.js', streams])


def test_swift_rejects_extensions(tmp_path, streams):
    compiler = tool('swiftc')
    generated = tmp_path / 'swift'
    assert mavgen.mavgen(mavgen.Opts(output=str(generated), language='Swift', wire_protocol='1.0', validate=False),
                         [str(XML.with_name('minimal.xml'))])
    (generated / 'main.swift').write_text((RESOURCES / 'reject.swift').read_text())
    exe = tmp_path / 'reject'
    run([compiler, '-module-cache-path', tmp_path / 'swift-cache', *generated.glob('*.swift'), '-o', exe])
    run([exe, streams])


def objc_sources(tmp_path):
    generated = generate(tmp_path / 'objc', 'ObjC', xml=XML.with_name('minimal.xml'))
    headers = generate(tmp_path / 'c', 'C', xml=XML.with_name('minimal.xml'))
    # The minimal dialect does not define the MAV_BOOL enum used by this API.
    includes = ['-DMAV_BOOL=BOOL'] + ['-I' + str(p) for p in [generated, headers / 'minimal',
                generated / 'minimal']]
    return generated, includes


def test_objc_rejects_extensions(tmp_path, streams):
    import sys
    if sys.platform != 'darwin':
        pytest.skip('Objective-C runtime test requires Apple Foundation and ARC')
    generated, includes = objc_sources(tmp_path)
    exe = tmp_path / 'reject'
    run([tool('clang'), '-fobjc-arc', '-framework', 'Foundation', '-include', 'Foundation/Foundation.h',
         *includes, *generated.rglob('*.m'), RESOURCES / 'reject.m', '-o', exe])
    run([exe, streams])


def test_objc_compiles_with_gnustep(tmp_path):
    root = os.environ.get('MAVLINK_GNUSTEP_ROOT')
    objc_include = os.environ.get('MAVLINK_OBJC_INCLUDE')
    if not root or not objc_include:
        pytest.skip('Set MAVLINK_GNUSTEP_ROOT and MAVLINK_OBJC_INCLUDE for GNUstep syntax check')
    generated, includes = objc_sources(tmp_path)
    run([tool('clang'), '-fsyntax-only', '-fobjc-runtime=gnustep-2.0', '-fobjc-weak',
         '-DGNUSTEP', '-DGNUSTEP_BASE_LIBRARY=1', '-I' + root, '-I' + objc_include,
         '-include', 'Foundation/Foundation.h', *includes, *generated.rglob('*.m'), RESOURCES / 'reject.m'])


@pytest.fixture
def spin2_harness(tmp_path, streams):
    compiler = tool('flexspin')
    assert mavgen.mavgen(mavgen.Opts(output=str(tmp_path / 'mavlink'), language='Spin2',
                                  wire_protocol='2.0', validate=False), [str(XML.with_name('minimal.xml'))])
    data = bytearray()
    for stream in sorted(streams.glob('*.v2')):
        packet = stream.read_bytes()
        data += struct.pack('<H', len(packet)) + packet
    (tmp_path / 'streams.bin').write_bytes(data)
    (tmp_path / 'reject.spin2').write_text((RESOURCES / 'reject.spin2').read_text())
    binary = tmp_path / 'reject.binary'
    # Disable cached-code calls for simulator compatibility.
    run([compiler, '-2', '--fcache=0', '-O1', '-o', binary, 'reject.spin2'], cwd=tmp_path)
    return binary


def test_spin2_compiles(spin2_harness):
    assert spin2_harness.stat().st_size > 0


def test_spin2_rejects_extensions(spin2_harness):
    simulator = os.environ.get('MAVLINK_SPINSIM')
    if not simulator:
        pytest.skip('Set MAVLINK_SPINSIM to a simulator supporting FlexSpin 7.7 Spin2 structures')
    binary = spin2_harness
    output = run([simulator, '-t', '-b115200', '-q', '-10000000', binary], timeout=30)
    assert output == 'OK'  # simulator instruction limit alone is not success
