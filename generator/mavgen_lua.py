#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a Wireshark LUA dissector
'''
from __future__ import print_function

from builtins import range

import os

def generate(basename, xml):
    '''generate complete python implemenation'''
    msgs = []
    enums = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        enums.extend(x.enum)
        filelist.append(os.path.basename(x.filename))

    unpack_types = {
        'float'    : 'f',
        'double'   : 'd',
        'char'     : 'b',
        'int8_t'   : 'b',
        'uint8_t'  : 'B',
        'uint8_t_mavlink_version'  : 'B',
        'int16_t'  : 'i2',
        'uint16_t' : 'I2',
        'int32_t'  : 'i4',
        'uint32_t' : 'I4',
        'int64_t'  : 'i8',
        'uint64_t' : 'I8',
        }


    print("Generating %s/mavlink_msgs.lua" % basename)
    outf = open("%s/mavlink_msgs.lua" % basename, "w")
    # dump the actual symbol table
    outf.write("local mavlink_msgs = {}\n")
    for m in msgs:
        # outf.write("mavlink_msg.msgs[%u] = \"%s\"\n" % (m.id, m.name))
        mavlink_msg_file = open("%s/mavlink_msg_%s.lua" % (basename, m.name), "w")
        mavlink_msg_file.write("local %s = {}\n" % m.name)
        mavlink_msg_file.write("%s.id = %u\n" % (m.name, m.id))
        mavlink_msg_file.write("%s.fields = {\n" % m.name)
        for i in range(0, len(m.ordered_fields)):
            field = m.ordered_fields[i]
            if (field.array_length > 0):
                mavlink_msg_file.write("             { \"%s\", \"<%s\", %s },\n" % (field.name, unpack_types.get(field.type), field.array_length))
            else:
                mavlink_msg_file.write("             { \"%s\", \"<%s\" },\n" % (field.name, unpack_types.get(field.type)))
        mavlink_msg_file.write("             }\n")
        mavlink_msg_file.write("return %s\n" % m.name)
        mavlink_msg_file.close()
    outf.write(
"""
local mavlink_msgs = {}

function mavlink_msgs.get_msgid(msgname)
  local message_map = require("mavlink_msg_" .. msgname)
  if not message_map then
    error("Unknown MAVLink message " .. msgname)
  end
  return message_map.id
end

function mavlink_msgs.decode_header(message)
  -- build up a map of the result
  local result = {}

  local read_marker = 3

  -- id the MAVLink version
  result.protocol_version, read_marker = string.unpack("<B", message, read_marker)
  if (result.protocol_version == 0xFE) then -- mavlink 1
    result.protocol_version = 1
  elseif (result.protocol_version == 0XFD) then --mavlink 2
    result.protocol_version = 2
  else
    error("Invalid magic byte")
  end

  local payload_len, read_marker = string.unpack("<B", message, read_marker) -- payload is always the second byte

  -- strip the incompat/compat flags
  result.incompat_flags, result.compat_flags, read_marker = string.unpack("<BB", message, read_marker)

  -- fetch seq/sysid/compid
  result.seq, result.sysid, result.compid, read_marker = string.unpack("<BBB", message, read_marker)

  -- fetch the message id
  result.msgid, read_marker = string.unpack("<I3", message, read_marker)

  return result, read_marker
end

function mavlink_msgs.decode(message, msg_map)
  local result, offset = mavlink_msgs.decode_header(message)
  local message_map = require("mavlink_msg_" .. msg_map[result.msgid])
  if not message_map then
    -- we don't know how to decode this message, bail on it
    return nil
  end

  -- map all the fields out
  for i,v in ipairs(message_map.fields) do
    if v[3] then
      result[v[1]] = {}
      for j=1,v[3] do
        result[v[1]][j], offset = string.unpack(v[2], message, offset)
      end
    else
      result[v[1]], offset = string.unpack(v[2], message, offset)
    end
  end

  -- ignore the idea of a checksum

  return result;
end

function mavlink_msgs.encode(msgname, message)
  local message_map = require("mavlink_msg_" .. msgname)
  if not message_map then                 
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgname)
  end

  local packString = "<"
  local packedTable = {}                  
  local packedIndex = 1
  for i,v in ipairs(message_map.fields) do
    if v[3] then
      packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
      for j = 1, v[3] do
        packedTable[packedIndex] = message[message_map.fields[i][1]][j]
        if packedTable[packedIndex] == nil then
          packedTable[packedIndex] = 0
        end
        packedIndex = packedIndex + 1
      end
    else
      packString = (packString .. string.sub(v[2], 2))
      packedTable[packedIndex] = message[message_map.fields[i][1]]
      packedIndex = packedIndex + 1
    end
  end
  return message_map.id, string.pack(packString, table.unpack(packedTable))
end

return mavlink_msgs
""")
    outf.close()
    print("Generated %s OK" % basename)

