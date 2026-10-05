'''Cross-language coverage for every message, using generated C as the wire oracle.'''
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest
from pymavlink.generator import mavgen, mavgen_rust, mavparse

HERE = Path(__file__).parent / 'rust'
pytestmark = pytest.mark.skipif(not shutil.which('cargo') or not shutil.which('cc'),
                                reason='Rust/C interoperability requires cargo and cc')


def run(args, **kwargs):
    result = subprocess.run([str(a) for a in args], text=True, capture_output=True, **kwargs)
    if result.returncode != 0:
        pytest.fail(result.stdout + result.stderr, pytrace=False)
    return result.stdout


def definitions():
    candidates = [Path(os.environ.get('MDEF', '../message_definitions')),
                  Path(__file__).parents[1] / 'mavlink/message_definitions']
    for path in candidates:
        if (path / 'v1.0/all.xml').exists():
            return (path / 'v1.0/all.xml').resolve()
    pytest.fail('all.xml is required; set MDEF to MAVLink message_definitions')


def dialects(path):
    result, seen = [], set()
    pending = [path]
    while pending:
        path = pending.pop(0).resolve()
        if path in seen:
            continue
        seen.add(path)
        xml = mavparse.MAVXML(str(path), '2.0')
        result.append(xml)
        pending.extend(path.parent / include for include in xml.include)
    assert not mavparse.check_duplicates(result)
    return result


def values(field, enums, pattern):
    if field.enum:
        enum = enums[field.enum]
        entries = [e for e in enum.entry if not e.end_marker]
        # Keep enum values valid even when an enum contains wider values than a field.
        entries = [e for e in entries if e.value < 2 ** (field.type_length * 8)]
        value = entries[0 if pattern == 0 else -1].value
    elif field.const_value is not None:
        value = field.const_value
    elif pattern == 0:
        value = 0
    elif field.type in ('float', 'double'):
        value = -123.25 - field.wire_offset
    elif field.type == 'char':
        value = 200
    elif field.type.startswith('int'):
        value = -(2 ** (field.type_length * 8 - 2)) + field.wire_offset % (2 ** (field.type_length * 8 - 2))
    else:
        value = (2 ** (field.type_length * 8) - 1) - field.wire_offset
    result = [value] * (field.array_length or 1)
    if pattern and field.array_length and field.enum:
        result = [entries[i % len(entries)].value for i in range(field.array_length)]
    if pattern and field.array_length and not field.enum:
        for i in range(field.array_length):
            if field.type in ('float', 'double'):
                result[i] = value - i * 0.25
            elif field.type.startswith('int'):
                bits = field.type_length * 8
                result[i] = ((value + i + 2 ** (bits - 1)) % 2 ** bits) - 2 ** (bits - 1)
            else:
                result[i] = (value - i) % 2 ** (field.type_length * 8)
    return result


def fixture_sources(xml):
    enums = {e.name: e for x in xml for e in x.enum}
    messages = sorted((m for x in xml for m in x.message), key=lambda m: m.id)
    c = '#include <assert.h>\n#include <stdio.h>\n#include <stdlib.h>\n#include <string.h>\n#include "all/mavlink.h"\n'
    c += (HERE / 'oracle.c').read_text()
    rust = 'use mavlink_generated::{DefaultDialect as All, messages, enums};\n'
    rust += 'fn fixture(id: u32, pattern: usize) -> All { match (id, pattern) {\n'
    for m in messages:
        c += 'static void message_%d(unsigned pattern, unsigned version, uint32_t source, uint32_t target, unsigned signed_frame) {\n' % m.id
        c += 'mavlink_message_t msg = {0}; mavlink_status_t status = {0}; mavlink_signing_t signing = {0};\n'
        c += 'status.current_tx_seq = 239; if(version == 1) status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;\n'
        c += 'if(signed_frame) { memset(signing.secret_key, 42, 32); signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING; signing.link_id = 3; signing.timestamp = 1000; status.signing = &signing; }\n'
        for pattern in (0, 1):
            c += 'if (pattern == %d) {\n' % pattern
            arguments = []
            rust += '(%d, %d) => messages::%s {\n' % (m.id, pattern, mavgen_rust.camel(m.name))
            for f in m.fields:
                vals = values(f, enums, pattern)
                cv = [str(v) for v in vals]
                if f.type.startswith('uint'):
                    cv = [v + 'ULL' for v in cv]
                if f.array_length:
                    c += '%s f_%s[%d] = {%s};\n' % (f.type, f.name, f.array_length, ','.join(cv))
                    arguments.append('f_' + f.name)
                elif f.is_target_system:
                    arguments.append('target')
                elif not f.omit_arg:
                    arguments.append(cv[0])
                rust_values = []
                for val in vals:
                    value = str(float(val)) if f.type in ('float', 'double') else str(val)
                    if f.enum:
                        enum = enums[f.enum]
                        typ = 'enums::' + mavgen_rust.camel(f.enum)
                        if enum.bitmask:
                            value = '%s::from_bits_retain(%s)' % (typ, value)
                        else:
                            entry = next(e for e in enum.entry if e.value == val)
                            value = '%s::%s' % (typ, mavgen_rust.entry_name(enum, entry))
                    rust_values.append(value)
                value = '[' + ','.join(rust_values) + ']' if f.array_length else rust_values[0]
                rust += '%s: %s,\n' % (mavgen_rust.snake(f.name), value)
            rust += '}.into(),\n'
            c += 'assert(mavlink_msg_%s_pack_status(source, 11, &status, &msg, %s));\n' % (m.name_lower, ', '.join(arguments))
            c += 'emit(&msg, pattern, version, source, target, signed_frame);\n}\n'
        c += '}\n'
    rust += '_ => panic!("unknown fixture {id}/{pattern}"),\n} }\n'
    c += 'int main(int argc, char **argv) { if(argc > 1) return verify_file(argv[1]);\n'
    c += 'const uint32_t sources[] = {42, 255, 256, 0x80000001U, 0xffffffffU};\n'
    c += 'const uint32_t targets[] = {0, 7, 255, 256, 0xffffffffU};\n'
    for m in messages:
        c += 'for(unsigned p=0;p<2;p++) for(unsigned v=1;v<=2;v++) for(unsigned s=0;s<5;s++) for(unsigned t=0;t<%d;t++) for(unsigned sig=0;sig<2;sig++) {\n' % (5 if m.target_system_fieldname else 1)
        c += 'if(v==1 && (%d>255 || sources[s]>255 || targets[t]>255 || sig)) continue;\n' % m.id
        if m.target_system_fieldname and m.target_system_ofs >= m.wire_min_length:
            c += 'if(v==1 && t>0) continue;\n'
        c += 'message_%d(p,v,sources[s],targets[t],sig);\n}\n' % m.id
    c += 'return 0; }\n'
    return c, rust, len(messages)


@pytest.fixture(scope='module')
def generated(tmp_path_factory):
    for executable in ('cargo', 'cc'):
        if not shutil.which(executable):
            pytest.skip(executable + ' is required')
    directory = tmp_path_factory.mktemp('rust-generator')
    xml = definitions()
    for language, output in [('Rust', directory / 'rust'), ('C', directory / 'c')]:
        assert mavgen.mavgen(mavgen.Opts(str(output), wire_protocol='2.0', language=language), [str(xml)])
    c, rust, count = fixture_sources(dialects(xml))
    (directory / 'oracle.c').write_text(c)
    run(['cc', '-std=c99', '-O0', '-Wno-address-of-packed-member', '-I' + str(directory / 'c'),
         directory / 'oracle.c', '-o', directory / 'oracle'])
    fixtures = run([directory / 'oracle'])
    tests = directory / 'rust/tests'
    tests.mkdir()
    (tests / 'c.frames').write_text(fixtures)
    (tests / 'interop.rs').write_text(rust + (HERE / 'interop.rs').read_text())
    shutil.copy(HERE / 'wire_tests.rs', tests / 'wire_tests.rs')
    # Exercise the actual consumer's traits and framing for ordinary IDs.
    with (directory / 'rust/Cargo.toml').open('a') as f:
        f.write('\n[dev-dependencies]\nmavio = { version = "0.5.10", default-features = false, features = ["sha2"] }\n')
    return directory, count


def test_all_messages_against_c(generated):
    directory, count = generated
    result = run(['cargo', 'test', '--manifest-path', directory / 'rust/Cargo.toml', '--', '--nocapture'],
                 env=dict(os.environ, RUST_FRAMES=str(directory / 'rust.frames'), EXPECT_MESSAGES=str(count)))
    assert 'test result: ok' in result
    run([directory / 'oracle', directory / 'rust.frames'])


def test_feature_builds(generated):
    directory, _ = generated
    run(['cargo', 'check', '--manifest-path', directory / 'rust/Cargo.toml', '--no-default-features'])
    run(['cargo', 'check', '--manifest-path', directory / 'rust/Cargo.toml', '--features', 'serde'])
    run(['cargo', 'check', '--manifest-path', directory / 'rust/Cargo.toml', '--features', 'std,serde'])


def test_v1_generator(tmp_path):
    assert mavgen.mavgen(mavgen.Opts(str(tmp_path), language='Rust', wire_protocol='1.0'), [str(definitions())])
    run(['cargo', 'check', '--manifest-path', tmp_path / 'Cargo.toml'])


def test_dialect_inheritance(tmp_path):
    parent = tmp_path / 'parent.xml'
    parent.write_text('''<mavlink><version>3</version><enums><enum name="MODE">
<entry name="MODE_FIRST" value="1"/></enum></enums><messages>
<message id="1" name="ENUM_MESSAGE"><description>Enum inheritance.</description><field type="uint8_t" name="mode" enum="MODE"/></message>
<message id="2" name="PLAIN_MESSAGE"><description>Shared message type.</description><field type="uint8_t" name="value"/></message>
</messages></mavlink>''')
    child = tmp_path / 'child.xml'
    child.write_text('''<mavlink><include>parent.xml</include><enums><enum name="MODE">
<entry name="MODE_SECOND" value="2"/></enum></enums><messages/></mavlink>''')
    output = tmp_path / 'rust'
    assert mavgen.mavgen(mavgen.Opts(str(output), language='Rust', wire_protocol='2.0'), [str(child)])
    (output / 'tests').mkdir()
    (output / 'tests/inheritance.rs').write_text('''
use mavlink_generated::dialects::{parent, child};
use mavlink_generated::spec::{Dialect, IntoPayload, MavLinkVersion};
#[test]
fn scoped_types() {
    assert_eq!(child::Child::version(), Some(3));
    // This exhaustive match must remain exhaustive when the child extends MODE.
    let parent = parent::enums::Mode::First;
    match parent { parent::enums::Mode::First => () }
    assert!(parent::enums::Mode::try_from(2u8).is_err());
    assert_eq!(child::enums::Mode::try_from(2u8).unwrap(), child::enums::Mode::Second);
    let child = child::messages::EnumMessage { mode: child::enums::Mode::Second };
    let payload = child.encode(MavLinkVersion::V2).unwrap();
    assert!(parent::messages::EnumMessage::try_from(&payload).is_err());
    assert!(child::messages::EnumMessage::try_from(&payload).is_ok());
    let plain = parent::messages::PlainMessage { value: 3 };
    let _: child::messages::PlainMessage = plain;
}
''')
    run(['cargo', 'test', '--manifest-path', output / 'Cargo.toml'])


@pytest.mark.parametrize('entries,expected', [([], 'no entries'),
                                               ([("MODE_FIRST", 1), ("MODE_ALIAS", 1)], 'duplicate discriminants'),
                                               ([("MODE_FOO_BAR", 1), ("MODE_FooBar", 2)], 'name collision')])
def test_invalid_rust_enums(entries, expected):
    enum = mavparse.MAVEnum('MODE', 1)
    enum.entry = [mavparse.MAVEnumEntry(name, value) for name, value in entries]
    with pytest.raises(ValueError, match=expected):
        mavgen_rust.generate_enum(enum)


@pytest.mark.parametrize('own,parents,expected', [
    ('<version>5</version><dialect>9</dialect>', ['<version>3</version><dialect>1</dialect>'], ('9', '5')),
    ('', ['<version>3</version><dialect>1</dialect>', '<version>4</version><dialect>2</dialect>'], ('2', '4')),
    ('', ['<version>3</version><dialect>1</dialect>', '<version>4</version>'], (None, '4')),
    ('', ['<dialect>2</dialect>'], (None, None)),
    ('', ['<include>grandparent.xml</include>'], (None, None)),
])
def test_mavinspect_metadata_compatibility(tmp_path, own, parents, expected):
    # Preserve the existing MAVInspect generator's metadata rules, including
    # version-gated dialect IDs and non-transitive, last-include precedence.
    (tmp_path / 'grandparent.xml').write_text('<mavlink><version>7</version><dialect>8</dialect></mavlink>')
    includes = []
    for i, content in enumerate(parents):
        name = 'parent%d.xml' % i
        (tmp_path / name).write_text('<mavlink>' + content + '</mavlink>')
        includes.append(name)
    filename = tmp_path / 'child.xml'
    filename.write_text('<mavlink>' + own + '</mavlink>')
    assert mavgen_rust.dialect_metadata(SimpleNamespace(filename=filename, include=includes)) == expected


def test_missing_enum_origin(tmp_path):
    filename = tmp_path / 'missing.xml'
    filename.write_text('''<mavlink><enums><enum name="MODE">
<entry name="MODE_FIRST" value="1"/></enum></enums><messages/></mavlink>''')
    xml = mavparse.MAVXML(str(filename), '2.0')
    xml.enum[0].entry[0].origin_file = ''
    with pytest.raises(ValueError, match='Unknown Rust enum entry origin'):
        mavgen_rust.generate(str(tmp_path / 'rust'), [xml])
