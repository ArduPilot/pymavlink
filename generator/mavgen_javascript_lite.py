#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a javascript file to get CRC extra and name from message ID and MAV_COMPONENT enum. No payload fields are considered.
'''
from __future__ import print_function

from builtins import range

import os

def generate(basename, xml):
    msgs = []
    enums = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        enums.extend(x.enum)
        filelist.append(os.path.basename(x.filename))

    print("Generating %s/mavlink_msgs.js" % basename)
    # create the output directory if needed
    if not os.path.exists(basename):
        os.makedirs(basename)
    outf = open("%s/mavlink_msgs.js" % basename, "w")

    outf.write("// Auto generated MAVLink parsing script\n\nmavlink_msgs = []\n")

    for msg in msgs:
        outf.write("mavlink_msgs[%i] = { name: \"%s\", CRC: %i }\n" % (msg.id, msg.name, msg.crc_extra))
    outf.write("\n")

    for enum in enums:
        if enum.name != "MAV_COMPONENT":
            # Only need component ID
            continue
        outf.write("%s = []\n" % enum.name)
        for entry in enum.entry:
            outf.write("%s[%i] = \"%s\"\n" % (enum.name, entry.value, entry.name))
        outf.write("\n")


    outf.close()
    print("Generated %s OK" % basename)

