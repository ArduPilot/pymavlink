#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a Ardupilot LUA mavlink module
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
    # create the output directory if needed
    if not os.path.exists(basename):
        os.makedirs(basename)
    outf = open("%s/mavlink_msgs.lua" % basename, "w")
    if 'modules' not in basename:
        print("ERROR: mavlink_msgs.lua must be generated in the modules directory or subdirectory")
        return
    # find the path relative to module directory
    module_root_rel = basename.split('modules')[1]
    module_root_rel = module_root_rel.strip('/')
    if module_root_rel != '':
        module_root_rel += '/'

    # dump the actual symbol table
    for m in msgs:
        mavlink_msg_file = open("%s/mavlink_msg_%s.lua" % (basename, m.name), "w")
        mavlink_msg_file.write("local %s = {}\n" % m.name)
        mavlink_msg_file.write("%s.id = %u\n" % (m.name, m.id))
        mavlink_msg_file.write("%s.fields = {\n" % m.name)
        for i in range(0, len(m.ordered_fields)):
            field = m.ordered_fields[i]
            if ((field.type == 'char') and (field.array_length > 0)):
                # Convert char array to lua string
                mavlink_msg_file.write("             { \"%s\", \"<c%s\" },\n" % (field.name, field.array_length))
            elif (field.array_length > 0):
                mavlink_msg_file.write("             { \"%s\", \"<%s\", %s },\n" % (field.name, unpack_types.get(field.type), field.array_length))
            else:
                mavlink_msg_file.write("             { \"%s\", \"<%s\" },\n" % (field.name, unpack_types.get(field.type)))
        mavlink_msg_file.write("             }\n")
        mavlink_msg_file.write("return %s\n" % m.name)
        mavlink_msg_file.close()
    outf.write(
"""-- Auto generated MAVLink parsing script
local mavlink_msgs = {{}}

function mavlink_msgs.get_msgid(msgname)
  local message_map = require("{module_root_rel}mavlink_msg_" .. msgname)
  if not message_map then
    error("Unknown MAVLink message " .. msgname)
  end
  return message_map.id
end

function mavlink_msgs.decode_header(message)
  -- build up a map of the result
  local result = {{}}

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

  _, read_marker = string.unpack("<B", message, read_marker) -- payload is always the second byte

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
  local message_map = require("{module_root_rel}mavlink_msg_" .. msg_map[result.msgid])
  if not message_map then
    -- we don't know how to decode this message, bail on it
    return nil
  end

  -- map all the fields out
  for _,v in ipairs(message_map.fields) do
    if v[3] then
      result[v[1]] = {{}}
      for j=1,v[3] do
        result[v[1]][j], offset = string.unpack(v[2], message, offset)
      end
    else
      result[v[1]], offset = string.unpack(v[2], message, offset)
      if string.sub(v[2],2,2) == 'c' then
        -- Got string, unpack includes 0 values to the set length
        -- this is annoying, so remove them
        result[v[1]] = string.gsub(result[v[1]], "\0", "")
      end
    end
  end

  -- ignore the idea of a checksum

  return result;
end

function mavlink_msgs.encode(msgname, message)
  local message_map = require("{module_root_rel}mavlink_msg_" .. msgname)
  if not message_map then
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgname)
  end

  local packString = "<"
  local packedTable = {{}}
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
""".format(module_root_rel=module_root_rel))
    outf.close()
    print("Generated %s OK" % basename)

