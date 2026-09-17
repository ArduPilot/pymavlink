#!/bin/bash

set -e
set -x

test -z "$MDEF" && MDEF="../message_definitions"

# MAVLINK_DIALECT=ardupilotmega python setup.py clean build install --user

tools/mavgen.py --lang C $MDEF/v1.0/all.xml -o generator/C/include_v1.0 --wire-protocol=1.0
tools/mavgen.py --lang C $MDEF/v1.0/all.xml -o generator/C/include_v2.0 --wire-protocol=2.0
tools/mavgen.py --lang C++11 $MDEF/v1.0/all.xml -o generator/CPP11/include_v2.0 --wire-protocol=2.0

# stable, dedicated fixture used by test_diagnostics_positive.c (not schema-valid
# standalone, so it needs --no-validate) rather than a real dialect message whose
# <wip>/<deprecated>/<superseded> tags could be edited away from under the test
tools/mavgen.py --lang C tests/wip_deprecated_superseded.xml -o generator/C/include_v2.0 --wire-protocol=2.0 --no-validate

pushd generator/C/test/posix
make clean testmav1.0_ardupilotmega testmav2.0_ardupilotmega test_issues test_diagnostics test_diagnostics_be test_diagnostics_positive

# these test tools emit the test packet as hexadecimal and human-readable,
# other tools consume it as a cross-reference, we ignore the hex here.
./testmav1.0_ardupilotmega | egrep -v '(^fe|^fd)'
./testmav2.0_ardupilotmega | egrep -v '(^fe|^fd)'
./test_issues
./test_diagnostics
./test_diagnostics_be
popd

pushd generator/CPP11/test/posix
make clean all
popd
