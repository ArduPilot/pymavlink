"""Check which wip/deprecated/superseded attribute macros the C and C++11
generators put on each enum and enum entry of the
wip_deprecated_superseded.xml fixture.

The compile tests under generator/C/test/posix and
generator/CPP11/test/posix show that a diagnostic fires, but a compiler
reports only one diagnostic for an entry carrying several attributes, so
they cannot show that every applicable macro is present.  This test checks the generated text instead.
"""
from pathlib import Path
import re

from pymavlink.generator import mavgen

XML = Path(__file__).parent / 'wip_deprecated_superseded.xml'

WIP = 'MAVLINK_ENUM_WIP'
DEP = 'MAVLINK_ENUM_DEPRECATED'
SUP = 'MAVLINK_ENUM_SUPERSEDED'

# enum name -> (attributes on the type, {entry name: attributes})
EXPECTED = {
    'WIP_ENUM': ([WIP], {
        'WIP_ENUM_INHERITS_WIP': [WIP],
        # an entry's own flag adds to its enum's rather than replacing it
        'WIP_ENUM_OWN_DEPRECATED': [WIP, DEP],
    }),
    'DEPRECATED_ENUM': ([DEP], {
        'DEPRECATED_ENUM_INHERITS_DEPRECATED': [DEP],
    }),
    'SUPERSEDED_ENUM': ([SUP], {
        'SUPERSEDED_ENUM_INHERITS_SUPERSEDED': [SUP],
    }),
    # extended by the fixture, without a flag, from a deprecated enum in
    # wip_deprecated_superseded_base.xml
    'EXTENDED_DEPRECATED_ENUM': ([DEP], {
        'EXTENDED_DEPRECATED_ENUM_FROM_FLAGGED_BASE': [DEP],
        'EXTENDED_DEPRECATED_ENUM_FROM_UNFLAGGED_DIALECT': [],
    }),
    # extended by the fixture, flagged WIP, from an unflagged enum in
    # wip_deprecated_superseded_base.xml
    'EXTENDED_WIP_ENUM': ([WIP], {
        'EXTENDED_WIP_ENUM_FROM_UNFLAGGED_BASE': [],
        'EXTENDED_WIP_ENUM_FROM_FLAGGED_DIALECT': [WIP],
    }),
    # flagged deprecated in both files
    'EXTENDED_BOTH_DEPRECATED_ENUM': ([DEP], {
        'EXTENDED_BOTH_DEPRECATED_ENUM_FROM_BASE': [DEP],
        'EXTENDED_BOTH_DEPRECATED_ENUM_FROM_DIALECT': [DEP],
    }),
    'ENTRY_FLAGS_ENUM': ([], {
        'ENTRY_FLAGS_ENUM_WIP': [WIP],
        'ENTRY_FLAGS_ENUM_DEPRECATED': [DEP],
        'ENTRY_FLAGS_ENUM_SUPERSEDED': [SUP],
        'ENTRY_FLAGS_ENUM_PLAIN': [],
    }),
}

# C++11 names an entry through its enum class (E::ENTRY), so the enum class
# carries only the flags every declaration has: a flag on it would be
# reported for the entries of an unflagged declaration too
CPP11_TYPE_ATTRIBUTES = {
    'EXTENDED_DEPRECATED_ENUM': [],
    'EXTENDED_WIP_ENUM': [],
}


def generate(directory, language):
    assert mavgen.mavgen(mavgen.Opts(output=str(directory), language=language,
                                     wire_protocol='2.0', validate=False), [str(XML)])
    return (directory / 'wip_deprecated_superseded' / 'wip_deprecated_superseded.h'
            if language == 'C' else
            directory / 'wip_deprecated_superseded' / 'wip_deprecated_superseded.hpp').read_text()


def attributes(text):
    return re.findall(r'MAVLINK_ENUM_\w+', text or '')


def test_c(tmp_path):
    header = generate(tmp_path, 'C')
    for enum, (type_attributes, entries) in EXPECTED.items():
        body = re.search(r'typedef enum %s\n\{\n(.*?)\n\} %s((?: \w+)*);' % (enum, enum),
                         header, re.S)
        assert body is not None, enum
        assert attributes(body.group(2)) == type_attributes, enum
        found = dict(re.findall(r'^   (\w+)((?: MAVLINK_ENUM_\w+)*)=', body.group(1), re.M))
        # the end marker is generated, not declared, and is never flagged
        assert attributes(found.pop('%s_ENUM_END' % enum)) == []
        assert {name: attributes(a) for name, a in found.items()} == entries, enum


def test_cpp11(tmp_path):
    header = generate(tmp_path, 'C++11')
    for enum, (type_attributes, entries) in EXPECTED.items():
        body = re.search(r'^enum class((?: MAVLINK_ENUM_\w+)*) %s\b.*?\n\{\n(.*?)\n\};' % enum,
                         header, re.S | re.M)
        assert body is not None, enum
        assert attributes(body.group(1)) == CPP11_TYPE_ATTRIBUTES.get(enum, type_attributes), enum
        found = dict(re.findall(r'^    (\w+)((?: MAVLINK_ENUM_\w+)*)=', body.group(2), re.M))
        expected = {name[len(enum) + 1:]: a for name, a in entries.items()}
        assert {name: attributes(a) for name, a in found.items()} == expected, enum
