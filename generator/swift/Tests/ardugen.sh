#!/bin/sh

export PYTHONPATH="$PYTHONPATH:../../../../"

python -m pymavlink.tools.mavgen ./MAVLinkTests/Testdata/ardupilotmega.xml -o ./MAVLink/MAVLink/ --wire-protocol 1.0 --lang Swift