#!/bin/bash

case "$(uname -s)" in

   Darwin)
     PYTHON=python
     ;;

   Linux)
     PYTHON=python
     ;;

   CYGWIN*|MINGW32*|MSYS*)
     # This assumes Python is installed in C:\Python27
     PYTHON=C:\Python27\python.exe
     ;;

   *)
     echo 'other OS'
     ;;
esac

set -e
set -x

test -z "$MDEF" && MDEF="../message_definitions"

MAVLINK_DIALECT=ardupilotmega $PYTHON setup.py clean build install --user
$PYTHON tools/mavgen.py --lang C $MDEF/v1.0/ardupilotmega.xml -o generator/C/include_v1.0 --wire-protocol=1.0
$PYTHON tools/mavgen.py --lang C $MDEF/v1.0/ardupilotmega.xml -o generator/C/include_v2.0 --wire-protocol=2.0
pushd generator/C/test/posix
make clean testmav1.0_common testmav2.0_common testmav1.0_ardupilotmega testmav2.0_ardupilotmega
./testmav1.0_common
./testmav2.0_common
./testmav1.0_ardupilotmega
./testmav2.0_ardupilotmega

