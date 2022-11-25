#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrii Fil root.fi36@gmail.com 2022
Based on mavgen_python.py
Released under GNU GPL version 3 or later
'''
from __future__ import print_function

import os
import math
import shutil
from operator import attrgetter
from . import mavparse

reserved_words = (
    'abort', 'else', 'new', 'return', 'abs', 'elsif', 'not', 'reverse', 'abstract', 'end', 'null',
    'accept', 'entry', 'select', 'access', 'exception', 'of', 'separate', 'aliased', 'exit', 'or',
    'subtype', 'all', 'others', 'synchronized', 'and', 'for', 'out', 'array', 'function', 'overriding',
    'tagged', 'at', 'task', 'generic', 'package', 'terminate', 'begin', 'goto', 'pragma', 'then',
    'body', 'private', 'type', 'if', 'procedure', 'case', 'in', 'protected', 'until', 'constant',
    'interface', 'use', 'is', 'raise', 'declare', 'range', 'when', 'delay', 'limited', 'record',
    'while', 'delta', 'loop', 'rem', 'with', 'digits', 'renames', 'do', 'mod', 'requeue', 'xor')

reserved_fields = ('sequence', 'system_id', 'component_id', 'message_id')

# mavlink developers cannot understand that there are not only c-like languages :(
try:
    math.log2(2)

    def is_position(val):
        return math.log2(val).is_integer()
except:
    def is_position(val):
        return 2 ** int(math.log(val, 2)) == val

def normalize_name(name, suffix):
    if name.lower() in reserved_words:
        return name + "_" + suffix
    return name

def normalize_field_name(name):
    n = normalize_name(name, "Field")
    if n.lower() in reserved_fields:
        return n + "_Field"
    return n

def normalize_message_name(name):
    return normalize_name(name, "Message")

def normalize_enum_name(name):
    return normalize_name(name, "Enum")

def normalize_entry_name(name):
    return normalize_name(name, "Entry")

field_types = {
    'uint8_t' : 'Interfaces.Unsigned_8',
    'uint16_t' : 'Interfaces.Unsigned_16',
    'uint32_t' : 'Interfaces.Unsigned_32',
    'uint64_t' : 'Interfaces.Unsigned_64',
    'int8_t' : 'Interfaces.Integer_8',
    'int16_t' : 'Interfaces.Integer_16',
    'int32_t' : 'Interfaces.Integer_32',
    'int64_t' : 'Interfaces.Integer_64',
    'float' : 'Short_Float',
    'double' : 'Long_Float'}

field_array_types = {
    'uint8_t' : 'Unsigned_8_Array',
    'uint16_t' : 'Unsigned_16_Array',
    'uint32_t' : 'Unsigned_32_Array',
    'uint64_t' : 'Unsigned_64_Array',
    'int8_t' : 'Integer_8_Array',
    'int16_t' : 'Integer_16_Array',
    'int32_t' : 'Integer_32_Array',
    'int64_t' : 'Integer_64_Array',
    'float' : 'Short_Float_Array',
    'double' : 'Long_Float_Array'}

def generate_mavlink_messages(outf, msgs):
    outf.write("""-- Defines Mavlink messages
-- Copyright Fil Andrii root.fi36@gmail.com 2022

with Interfaces;
with Mavlink.Types; use Mavlink.Types;

package Mavlink.Messages is

   pragma Pure (Mavlink.Messages);

   type Message
     (Message_Id : Msg_Id;
      Payload_Length : Interfaces.Unsigned_8) is abstract tagged record
      Sequence     : Interfaces.Unsigned_8;
      System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8;
   end record with Alignment => Message_Alignment, Size => Message_Size * 8;
   for Message use record
      Payload_Length at Tag_Length + 0 range 0 .. 7;
      Sequence       at Tag_Length + 1 range 0 .. 7;
      System_Id      at Tag_Length + 2 range 0 .. 7;
      Component_Id   at Tag_Length + 3 range 0 .. 7;
      Message_Id     at Tag_Length + 4 range 0 .. 7;
   end record;\n\n""")

    max_len = 0
    for m in msgs:
        name_len = len(normalize_message_name(m.name))
        if name_len > max_len:
            max_len = name_len
    max_len += 3

    s = ""
    for m in msgs:
        name = normalize_message_name(m.name).title()
        s += "   %s : constant Msg_Id := %i;\n" % ((name + "_Id").ljust(max_len), m.id)
    outf.write(s)
    outf.write("\n")

    s = ""
    for m in msgs:
        message_name = normalize_message_name(m.name).title()
        max_len = 0
        payload_length = 0
        for field in m.fields:
            payload_length += (field.array_length or 1) * field.type_length
            field_name_len = len(normalize_field_name(field.name))
            if field_name_len > max_len:
                max_len = field_name_len
        s += "   type %s is new Message\n" % message_name
        s += "     (Message_Id => %s_Id,\n      Payload_Length => %i) with record\n" % (message_name, payload_length)
        for field in m.fields:
            field_name = normalize_field_name(field.name).title()
            s += "      %s : " % field_name.ljust(max_len)
            if field.type == 'char':
                s += "String (1..%i)" % field.array_length
            else:
                if field.array_length:
                    s += "%s (1..%i)" % (field_array_types[field.type], field.array_length)
                elif field.enum:
                    s += "Types." + normalize_enum_name(field.enum).title()
                else:
                    s += field_types[field.type]
            s += ";\n"
        s += "   end record;\n"

        offset = 0
        s += "   for %s use record\n" % message_name
        for field in m.ordered_fields:
            field_name = normalize_field_name(field.name).title()
            field_size = (field.array_length or 1) * field.type_length * 8
            s += "      %s at Message_Size + %i range 0 .. %i;\n" % (field_name.ljust(max_len), offset, field_size - 1)
            offset += field_size / 8
        s += "   end record;\n\n"
    outf.write(s)

    s = """   CRC_Extras : constant array (Interfaces.Unsigned_8'Range) of
     Interfaces.Unsigned_8 := \n     (""";
    
    item_len = 0
    l = 6
    for m in msgs:
        item = "%s => %s," % (str(m.id).ljust(3), str(m.crc_extra))
        item = item.ljust(12)
        item_len = len(item)
        if l + item_len > 79:
            s += "\n      "
            l = 6
        l += item_len
        s += item
    s += "\n      others => 0) with Size => 2048;\n"
    outf.write(s)

    outf.write("\nend MAVLink.Messages;")

def repr_bitmask(enum, size):
    enum_name = normalize_enum_name(enum.name).title()
    s = "   type %s is record\n" % enum_name

    max_len = 0
    for i in enum.entry:
        if i.end_marker:
            break
        elif i.value == 0 or not is_position(i.value):
            continue
        name_len = len(normalize_entry_name(i.name))
        if name_len > max_len:
            max_len = name_len

    for i in enum.entry:
        if i.end_marker:
            break
        elif i.value == 0:
            continue
        elif not is_position(i.value):
            print("%s ignored because the composite value!" % i.name)
            continue
        name = normalize_entry_name(i.name).title()
        s += "      %s : Boolean := False;\n" % name.ljust(max_len)

    s += "   end record with Size => %i;\n" % (size * 8)

    s += "   for %s use record\n" % enum_name
    for i in enum.entry:
        if i.end_marker:
            break
        elif i.value == 0 or not is_position(i.value):
            continue
        name = normalize_entry_name(i.name).title()
        pos = int(math.log(i.value, 2))
        s += "      %s at 0 range %i .. %i;\n" % (name.ljust(max_len), pos, pos)
    s += "   end record;\n"

    return s

def repr_enum(enum, size):
    enum_name = normalize_enum_name(enum.name).title()
    s = "   type %s is\n     (" % enum_name
    l = 6
    max_len = 0
    for i in enum.entry:
        if i.end_marker:
            break
        if max_len > 0:
            s += ", "
            l += 2

        name = normalize_entry_name(i.name).title()
        name_len = len(name)
        if name_len > max_len:
            max_len = name_len
        l += name_len + 1
        if l > 80:
            l = name_len + 6
            s += "\n      "
        s += "%s" % name

    s += ")"
    aspect = " with Size => %i;\n" % (size * 8)
    if l + len(aspect) > 79:
        s += "\n    "
    s += aspect;

    s += "   for %s use\n     (" % enum_name
    l = 0
    for i in enum.entry:
        if i.end_marker:
            break
        if l > 0:
            s += ",\n      "
        else:
            l = 1
        s += "%s => %i" % (normalize_entry_name(i.name).title().ljust(max_len), i.value)
            
    s += ");\n"

    return s

def generate_mavlink_types(outf, types, types_size):
    outf.write("""-- Defines Mavlink types
-- Copyright Fil Andrii root.fi36@gmail.com 2022

package MAVLink.Types is

   pragma Pure (Mavlink.Types);\n\n""")

    for t in types:
        size = types_size[t.name]
        if size is not None:
            if t.bitmask:
                outf.write(repr_bitmask(t, size))
            else:
                outf.write(repr_enum(t, size))
            outf.write("\n")
    outf.write("end MAVLink.Types;")

def generate(directory, xml):
    '''generate complete Ada implementation'''
    mavparse.mkdir_p(directory)

    msgs = []
    types = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        types.extend(x.enum)
        filelist.append(os.path.basename(x.filename))
    msgs.sort(key=attrgetter('id'))

    types_size = {t.name: None for t in types}
    for m in msgs:
        for f in m.fields:
            if f.enum:
                if types_size[f.enum] is None:
                    types_size[f.enum] = f.type_length
                else:
                    assert types_size[f.enum] == f.type_length, "Different size for one enum"

    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'Ada')
    shutil.copy(os.path.join(srcpath, "x25crc.ads"), directory)
    shutil.copy(os.path.join(srcpath, "x25crc.adb"), directory)
    shutil.copy(os.path.join(srcpath, "mavlink.ads"), directory)
    shutil.copy(os.path.join(srcpath, "mavlink-connection.ads"), directory)
    shutil.copy(os.path.join(srcpath, "mavlink-connection.adb"), directory)
    shutil.copytree(os.path.join(srcpath, "examples"), os.path.join(directory, "examples"), dirs_exist_ok=True)

    print("Generate MAVLink.Types")
    with open(os.path.join(directory, "mavlink-types.ads"), "w") as f:
        generate_mavlink_types(f, types, types_size)
    print("Generate MAVLink.Messages")
    with open(os.path.join(directory, "mavlink-messages.ads"), "w") as f:
        generate_mavlink_messages(f, msgs)
