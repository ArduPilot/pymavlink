#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrii Fil root.fi36@gmail.com 2022
Based on mavgen_python.py
Released under GNU GPL version 3 or later
'''
from __future__ import print_function

import os
import os.path
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
    if not name[0].isalpha():
        return "A_" + name
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

field_values = {
    'uint8_t' : '1',
    'uint16_t' : '2',
    'uint32_t' : '3',
    'uint64_t' : '4',
    'int8_t' : '5',
    'int16_t' : '6',
    'int32_t' : '7',
    'int64_t' : '8',
    'float' : '9.0',
    'double' : '10.0'}

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

field_array_values = {
    'uint8_t' : '(others => 1)',
    'uint16_t' : '(others => 2)',
    'uint32_t' : '(others => 3)',
    'uint64_t' : '(others => 4)',
    'int8_t' : '(others => 5)',
    'int16_t' : '(others => 6)',
    'int32_t' : '(others => 7)',
    'int64_t' : '(others => 8)',
    'float' : '(others => 9.0)',
    'double' : '(others => 10.0)'}

# Default invalid values for fields
invalid = {
    'UINT8_MAX'  : "Interfaces.Unsigned_8'Last",
    'UINT16_MAX' : "Interfaces.Unsigned_16'Last",
    'UINT32_MAX' : "Interfaces.Unsigned_32'Last",
    'INT8_MAX'   : "Interfaces.Integer_8'Last",
    'INT16_MAX'  : "Interfaces.Integer_16'Last",
    'INT32_MAX'  : "Interfaces.Integer_32'Last"
}

GEN = """-------------------------------------------
--  DO NOT EDIT. This file is generated. --
-------------------------------------------\n\n"""

# format text as a comment with lines < 79
def format_comment(text, tab):
    sp = text.split()
    if len(sp) < 1: return ""

    s = "".ljust(tab) + "--  "
    start = len (s)
    l = start
    for i in sp:
        if l + len(i) + 1 > 79:
            s += "\n".ljust(tab + 1) + "--  "
            l = start
      
        s += i + " "
        l = l + len(i) + 1
    s += "\n"
    return s

#return information about deprication as a comment
def get_deprecated(deprecated, tab):
    s  = "".ljust(tab) + "------------\n"
    s += "".ljust(tab) + "--  DEPRECATED SINCE: %s REPLACED BY: %s\n" % (deprecated.since, deprecated.replaced_by)
    s += format_comment (deprecated.explanation, tab)
    s += "".ljust(tab) + "------------\n"
    return s

def generate_mavlink_messages(outf, msgs):
    outf.write("""--  Defines MAVLink messages
--  Copyright Fil Andrii root.fi36@gmail.com 2022

with Interfaces;
with MAVLink.Types; use MAVLink.Types;

package MAVLink.Messages is

   pragma Pure;

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
    names = [i.name for i in enum.entry if not i.end_marker]
    common_prefix = os.path.commonprefix(names) if len(names) > 1 else ""

    max_len = 0
    for i in enum.entry:
        if i.end_marker:
            name = "Reserved_%i" % (size * 8 - 1)
        elif i.value == 0 or not is_position(i.value):
            continue
        else:
            name = i.name[len(common_prefix):] # strip common prefix
        name_len = len(normalize_entry_name(name))
        if name_len > max_len:
            max_len = name_len

    for i in enum.entry:
        if i.end_marker:
            for j in range(math.ceil(math.log(i.value, 2)), size * 8):
                name = "Reserved_%i" % j
                s += "      %s : Boolean := False;\n" % name.ljust(max_len)
            break
        elif i.value == 0:
            continue
        elif not is_position(i.value):
            print("%s ignored because the composite value!" % i.name)
            continue
        name = i.name[len(common_prefix):] # strip common prefix
        name = normalize_entry_name(name).title()
        s += "      %s : Boolean := False;\n" % name.ljust(max_len)

    s += "   end record with Size => %i;\n" % (size * 8)
    s += format_comment (enum.description, 3) + '\n'

    parts = ""
    s += "   for %s use record\n" % enum_name
    for i in enum.entry:
        if i.end_marker:
            for j in range(math.ceil(math.log(i.value, 2)), size * 8):
                name = "Reserved_%i" % j
                s += "      %s at 0 range %i .. %i;\n" % (name.ljust(max_len), j, j)
            break
        elif i.value == 0 or not is_position(i.value):
            continue
        name = i.name[len(common_prefix):] # strip common prefix
        image = name
        name = normalize_entry_name(name).title()
        pos = int(math.log(i.value, 2))
        s += "      %s at 0 range %i .. %i;\n" % (name.ljust(max_len), pos, pos)
        parts += '\n      & (if V.%s then "%s " else "")' % (name, image)
    s += "   end record;\n"
    s += """
   function Image (V : {}) return String is
     ("["{}
      & "]");
""".format (enum_name, parts)
    return s

def repr_enum(enum, size):
    enum_name = normalize_enum_name(enum.name).title()
    s = "   type %s is new Interfaces.Unsigned_%i;\n" % (enum_name, size * 8)
    if enum.deprecated:
        s += "   pragma Obsolescent (%s);\n" % enum_name
        s += get_deprecated(enum.deprecated, 3)
    s += format_comment (enum.description, 3) + '\n'

    names = [i.name for i in enum.entry if not i.end_marker]
    common_prefix = os.path.commonprefix(names) if len(names) > 1 else ""
    first = ""
    last = ""
    last_value = 0
    predicate = ""
    choises = ""

    for i in enum.entry:
        if i.end_marker:
            break
        name = i.name[len(common_prefix):] # strip common prefix
        name = normalize_entry_name(name).title()

        if first == "":
            first = name
            last = name
            last_value = i.value
        elif last_value + 1 == i.value:
            last = name
            last_value = i.value
        else:
            predicate += ("\n       | " if predicate else "")
            predicate += (first if first == last else first + " .. " + last)
            first = name
            last = name
            last_value = i.value

        choises += ("," if choises else "")
        choises += '\n        when %s => "%s"' % (name, name)
        fun = "   function %s return %s is (%i)\n     with Static;\n" % (name, enum_name, i.value)
        s += fun
        if i.deprecated:
            s += "   pragma Obsolescent (%s);\n" % name
            s += get_deprecated(i.deprecated, 3)
        s += format_comment (i.description, 3) + '\n'

    predicate += ("\n       | " if predicate else "")
    predicate += (first if first == last else first + " .. " + last)
    subtype = """   subtype {0}_Well_Known is {0}
     with Static_Predicate => {0}_Well_Known in
       {1};

   function Well_Known_Image
     (Value : {0}_Well_Known) return String is
       (case Value is{2});

   function Image (Value : {0}) return String is
     (if Value in {0}_Well_Known
      then Well_Known_Image (Value) else "Unknown:" & Value'Image);
"""
    s += subtype.format(enum_name, predicate, choises)

    return s

def generate_mavlink_types(outf, types, types_size):
    outf.write("""--  Defines MAVLink types
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

pragma Ada_2022;

package MAVLink.Types is

   pragma Pure;\n\n""")

    for t in types:
        size = types_size[t.name]
        if size is not None:
            if t.bitmask:
                outf.write(repr_bitmask(t, size))
            else:
                outf.write(repr_enum(t, size))
            outf.write("\n")
    outf.write("end MAVLink.Types;")

msgs = []
types = []
filelist = []
types_size = {}
types_files = {}

def calculate_types_size(xml):
    global msgs
    global types
    global filelist
    global types_size
    global types_files

    for x in xml:
        msgs.extend(x.message)
        types.extend(x.enum)
        filelist.append(os.path.basename(x.filename))
        for enum in x.enum:
            types_files |= {enum.name: x}

    msgs.sort(key=attrgetter('id'))

    types_size = {t.name: None for t in types}
    for m in msgs:
        for f in m.fields:
            if f.enum:
                if types_size[f.enum] is None:
                    types_size[f.enum] = f.type_length
                else:
                    assert types_size[f.enum] == f.type_length, "Different size for one enum"

def generate(directory, xml):
    '''generate complete Ada implementation'''
    mavparse.mkdir_p(directory)

    global msgs
    global types
    global filelist
    global types_size
    calculate_types_size(xml)

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

#
# Generate V2
#

v2 = "Mavlink_v2"

#Add with/use for includes
def generate_includes(x, f):
    n_len = 0
    for i in x.include:
        n_len = max (len(os.path.splitext(i)[0]), n_len);

    for i in x.include:
        n = os.path.splitext(i)[0].title() + ";"
        f.write("with " + v2 + "." + n.ljust(n_len) + " use " + v2 + "." + n + "\n")

    if n_len > 0: f.write("\n")

# calculate size of enum type based on the max value
def calculate_enum_size(enum):
    max_value = 0
    for i in enum.entry:
        if i.end_marker: break
        max_value = max(max_value, i.value)

    size = math.ceil(math.ceil(math.log2(max_value)) / 8)
    if size < 1: size = 1
    return size

# calculate size of bitmask type based on the fields count
def calculate_bitmask_size(bitmask):
    c = 0
    for i in bitmask.entry:
        if i.end_marker:
            c += math.ceil(math.log2(i.value))
            break
        elif i.value == 0:
            continue
        elif not is_position(i.value):
            print("%s ignored because the composite value!" % i.name)
            continue
        c += 1
    return math.ceil(c / 8)

# return default value
def get_default(name, type_name):
    global types

    if name[0] == '[':
        return "        (others => %s)" % get_default(name[1:-1], type_name)
    elif name[:2] == '0x':
        if type_name.find("Float") != -1:
            return type_name + "\n        (16#" + name[2:] + "#)"
        return "16#%s#" % name[2:]
    else:
        if name in invalid:
            if type_name.find("Float") != -1:
                return type_name + "\n        (" + invalid[name] + ")"
            return invalid[name]
        elif name[:3].upper() == "NAN":
            return get_default("0", type_name)
        else:
            if type_name.find("Float") != -1 and name.find(".") == -1:
                return name + ".0"
            elif type_name == "String":
                return "Character'Val (%s)" % name
            else:
                for t in types:
                    if normalize_enum_name(t.name).title() == type_name and not t.bitmask:
                        names = [i.name for i in t.entry if not i.end_marker]
                        if name in names:
                            common_prefix = os.path.commonprefix(names) if len(names) > 1 else ""
                            return normalize_entry_name(name[len(common_prefix):]).title()

                return name

#Generate message
def generate_message(msg, spec, body):
    name = normalize_message_name(msg.name).title()

    spec.write("   %s : constant Msg_Id := %i;\n\n" % (name + "_Id", msg.id))

    max_len = 0
    for field in msg.fields:
        max_len = max(max_len, len(normalize_field_name(field.name)))

    spec.write("   type %s is record\n" % name)
    for field in msg.fields:
        field_name = normalize_field_name(field.name).title()
        spec.write("      %s : " % field_name.ljust(max_len))
        tp = ""
        if field.type == 'char':
            tp = "String"
            spec.write("String (1 .. %i)" % field.array_length)
        else:
            if field.array_length:
                tp = field_array_types[field.type]
                spec.write("%s (1 .. %i)" % (tp, field.array_length))
            elif field.enum:
                tp = normalize_enum_name(field.enum).title()
                if field_name == tp:
                    tp = "Common." + tp
                spec.write(tp)
            else:
                tp = field_types[field.type]
                spec.write(tp)

        if field.invalid:
            s = get_default(field.invalid, tp)
            if s != "":
                spec.write(" :=\n        %s" % s)

        spec.write(";\n")
        if field.units: spec.write("      --  Units: %s\n" % field.units)
        spec.write(format_comment(field.description, 6))
    spec.write("   end record;\n\n")
    if msg.deprecated:
        spec.write("   pragma Obsolescent (%s);\n\n" % name)

    offset = 0
    max_offset = 0
    for field in msg.ordered_fields:
        field_size = (field.array_length or 1) * field.type_length * 8
        pp = "%i" % offset
        max_offset = max(max_offset, len(pp))
        offset += field_size / 8

    offset = 0
    spec.write("   for %s use record\n" % name)
    for field in msg.ordered_fields:
        field_name = normalize_field_name(field.name).title()
        field_size = (field.array_length or 1) * field.type_length * 8
        off = "%i" % offset
        spec.write("      %s at %s range 0 .. %i;\n" % (field_name.ljust(max_len), off.ljust(max_offset), field_size - 1))
        offset += field_size / 8
    spec.write("   end record;\n\n")

    spec.write("""   procedure Encode
     (Message : %s;
      Connect : in out %s.Connection;
      Buffer  : out Data_Buffer;
      Last    : out Positive);
   --  Put the message in the buffer ready for send

   procedure Decode
     (Message   : out %s;
      Connect   : in out %s.Connection;
      CRC_Valid : out Boolean);
   --  Get the message from the Connect and delete it
   --  from the Connect's buffer. CRC_Valid is set to
   --  True if x25crc is valid for the message.

   procedure Decode
     (Message : out %s;
      Connect : in out %s.Connection);
   --  Same as Above but does not check CRC

   function Check_CRC (Connect : in out %s.Connection) return Boolean;
   --  Returns True if CRC is valid

""" % (name, v2, name, v2, name, v2, v2))

    # Body
    body.write("""   procedure Encode
     (Message : %s;
      Connect : in out %s.Connection;
      Buffer  : out Data_Buffer;
      Last    : out Positive)
   is
      Local : Data_Buffer (1 .. %s'Value_Size / 8)
        with Import, Address => Message'Address,
        Convention => Ada;

   begin
      Last  := Buffer'First +
        Message_Data_Position_In_Buffer +
        (%s'Value_Size / 8) - 1;
      Buffer (Buffer'First + Message_Data_Position_In_Buffer .. Last) := Local;
      Encode (Connect, %s_Id, %s, Buffer, Last);
   end Encode;

   procedure Decode
     (Message   : out %s;
      Connect   : in out %s.Connection;
      CRC_Valid : out Boolean)
   is
      Data  : constant Data_Buffer := Get_Message_Data (Connect);
      Buf   : Data_Buffer
        (1 .. %s'Size / 8) := (others => 0)
        with Address => Message'Address,
        Convention   => Ada;
   begin
      Buf (1 .. Data'Length) := Data;
      
      CRC_Valid := Check_CRC (Connect);

      Drop_Message (Connect);
   end Decode;

   function Check_CRC
     (Connect : in out %s.Connection) return Boolean is
   begin
      return Is_CRC_Valid (Connect, %s);
   end Check_CRC;

   procedure Decode
     (Message : out %s;
      Connect : in out %s.Connection)
   is
      Data  : constant Data_Buffer := Get_Message_Data (Connect);
      Buf   : Data_Buffer
        (1 .. %s'Size / 8) := (others => 0)
        with Address => Message'Address,
        Convention   => Ada;
   begin
      Buf (1 .. Data'Length) := Data;
      Drop_Message (Connect);
   end Decode;

""" % (name, v2, name, name, name, str(msg.crc_extra), name, v2, name, v2, str(msg.crc_extra), name, v2, name))

def get_pakage_name(x, m, directory):
    name = os.path.splitext(os.path.basename(x.filename))[0]
    pkg_name = normalize_message_name(m.name).title()
    if pkg_name[-1] == 's':
        pkg_name += 'es'
    else:
        pkg_name += 's'
    f_name = os.path.join(directory, "mavlink_v2-%s-message-%s" % (name, pkg_name.lower()))
    p_name = "%s.%s.Message.%s" % (v2, name.title(), pkg_name)
    return f_name, p_name

def find_package(xml, name):
    for x in xml:
        for t in x.enum:
            if not t.bitmask and t.name == name:
                return os.path.splitext(os.path.basename(x.filename))[0].title()
    return ""

# Generate test project
def generate_test(directory, xml, bitmasks):
    dir = os.path.join(directory, 'tests')
    mavparse.mkdir_p(dir)

    f = open(os.path.join(dir, "test.gpr"), "w")
    f.write("""with "../mavlink_v2.gpr";
project Test is

   for Source_Dirs use (".");
   for Object_Dir use "./obj/";
   for Main use ("test.adb");

   package Compiler is
      for Switches ("ada") use ("-gnat2022", "-gnata", "-g");
   end Compiler;

   package Builder is
      for Switches ("ada") use ("-g");
   end Builder;

   package Linker is
      for Switches ("ada") use ("-g");
   end Linker;

end Test;""")
    f = open(os.path.join(dir, "test.adb"), "w")
    for x in xml:
        for m in x.message:
            f_name, p_name = get_pakage_name(x, m, directory)
            f.write("with %s;\n" % p_name)
    f.write("""with SHA_256;
with Interfaces;
with Ada.Text_IO;
use Mavlink_v2;
use Interfaces;

procedure Test
is
   D1     : constant SHA_256.Data  := [16#61#, 16#62#, 16#63#];
   R1     : constant SHA_256.State :=
     [16#ba7816bf#, 16#8f01cfea#, 16#414140de#, 16#5dae2223#,
      16#b00361a3#, 16#96177a9c#, 16#b410ff61#, 16#f20015ad#];

   D2     : constant SHA_256.Data :=
     [16#61#, 16#62#, 16#63#, 16#64#, 16#62#, 16#63#, 16#64#, 16#65#, 16#63#,
      16#64#, 16#65#, 16#66#, 16#64#, 16#65#, 16#66#, 16#67#, 16#65#, 16#66#,
      16#67#, 16#68#, 16#66#, 16#67#, 16#68#, 16#69#, 16#67#, 16#68#, 16#69#,
      16#6a#, 16#68#, 16#69#, 16#6a#, 16#6b#, 16#69#, 16#6a#, 16#6b#, 16#6c#,
      16#6a#, 16#6b#, 16#6c#, 16#6d#, 16#6b#, 16#6c#, 16#6d#, 16#6e#, 16#6c#,
      16#6d#, 16#6e#, 16#6f#, 16#6d#, 16#6e#, 16#6f#, 16#70#, 16#6e#, 16#6f#,
      16#70#, 16#71#];
   R2     : constant SHA_256.State :=
     [16#248d6a61#, 16#d20638b8#, 16#e5c02693#, 16#0c3e6039#,
      16#a33ce459#, 16#64ff2167#, 16#f6ecedd4#, 16#19db06c1#];

   D3 : constant SHA_256.Data :=
     [16#6C#, 16#6F#, 16#6E#, 16#67#, 16#5F#, 16#70#, 16#61#, 16#73#, 16#73#,
      16#77#, 16#6F#, 16#72#, 16#64#, 16#fd#, 16#05#, 16#01#, 16#00#, 16#00#,
      16#01#, 16#01#, 16#78#, 16#32#, 16#00#, 16#01#, 16#00#, 16#00#, 16#00#,
      16#01#, 16#08#, 16#98#, 16#01#, 16#c8#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#];
   R3     : constant SHA_256.State :=
     [16#48c298bd#, 16#a123637a#, 16#f1103486#, 16#180a716a#,
      16#9c41e4b1#, 16#42293472#, 16#ea587ff5#, 16#247d5943#];
   T : Data_Buffer (1 .. 32) with Import, Address => R3'Address;

   procedure Do_SHA_256_Test (D : SHA_256.Data; R : SHA_256.State);
   procedure Do_SHA_256_Test (D : SHA_256.Data; R : SHA_256.State)
   is
      use type SHA_256.State;

      Checksum : SHA_256.Context;
      Result   : SHA_256.Digest_Type;
      Res      : SHA_256.State (1 .. 8) with Import, Address => Result'Address;
   begin
      SHA_256.Update (Checksum, D);
      Result := SHA_256.Digest (Checksum);
      pragma Assert (Res = R);
   end Do_SHA_256_Test;

   Hygrometer_Sensors_Data : constant Data_Buffer :=
     [253, 5, 1, 0, 0, 1, 1, 120, 50,  0, --  Header
      1, 0, 0, 0, 1, --  Message
      8, 152, --  CRC
      1, -- Link_Id
      200, 0, 0, 0, 0, 0, --  Timestamp
      189, 152, 194, 72,  122, 99]; --  SHA

   In_Connect  : Mavlink_v2.Connection (1, 1);
   Out_Connect : Mavlink_v2.Connection (1, 1);
   Pass        : constant String := "long_password";
   Pass_Data   : Signature_Key (1 .. Pass'Length) with Import,
     Address => Pass'Address;
   Res         : Boolean;

   Seq         : Interfaces.Unsigned_8;
   Sys_Id      : Interfaces.Unsigned_8;
   Comp_Id     : Interfaces.Unsigned_8;
   Id          : Msg_Id;
   Link_Id     : Interfaces.Unsigned_8;
   Timestamp   : Interfaces.Unsigned_64;
   Signature   : Three_Boolean;
   Buffer      : Data_Buffer (1 .. Mavlink_v2.Maximum_Buffer_Len);
   Last        : Positive;

begin
   Do_SHA_256_Test (D1, R1);
   Do_SHA_256_Test (D2, R2);
   Do_SHA_256_Test (D3, R3);

   Initialize_Signature (In_Connect,  1, Pass_Data, 200);
   Initialize_Signature (Out_Connect, 1, Pass_Data, 200);

   -- Income
   for Index in Hygrometer_Sensors_Data'First ..
     Hygrometer_Sensors_Data'Last
   loop
      Res := Parse_Byte (In_Connect, Hygrometer_Sensors_Data (Index));
   end loop;

   pragma Assert (Res);

   Get_Message_Information
     (In_Connect, Seq, Sys_Id, Comp_Id, Id, Link_Id, Timestamp, Signature);

   pragma Assert (Seq = 0);
   pragma Assert (Sys_Id = 1);
   pragma Assert (Comp_Id = 1);
   pragma Assert
     (Id =
        Mavlink_v2.Common.Message.Hygrometer_Sensors.Hygrometer_Sensor_Id);
   pragma Assert (Link_Id = 1);
   pragma Assert (Timestamp = 200);
   pragma Assert (Signature = True);
   Drop_Message (In_Connect);

   -- In / Out
   declare
      use Mavlink_v2.Common.Message.Hygrometer_Sensors;
      M : constant Hygrometer_Sensor :=
        (Id => 1, Temperature => 1, Humidity => 0);
      O : Hygrometer_Sensor;
   begin
      Encode (M, Out_Connect, Buffer, Last);
      pragma Assert (Last = 30);

      for Index in Buffer'First .. Last loop
         Res := Parse_Byte (In_Connect, Buffer (Index));
      end loop;
      pragma Assert (Res);

      Get_Message_Information
        (In_Connect, Seq, Sys_Id, Comp_Id, Id, Link_Id, Timestamp, Signature);

      pragma Assert (Seq = 0);
      pragma Assert (Sys_Id = 1);
      pragma Assert (Comp_Id = 1);
      pragma Assert (Id = Hygrometer_Sensor_Id);
      pragma Assert (Link_Id = 1);
      pragma Assert (Timestamp = 200);
      pragma Assert (Signature = True);

      Decode (O, In_Connect, Res);
      pragma Assert (M = O);
   end;

""")
    for x in xml:
        for m in x.message:
            msg = normalize_message_name(m.name).title()
            f_name, p_name = get_pakage_name(x, m, directory)
            f.write("""   declare
      use %s;
      I : %s;
      O : %s :=
         (""" % (p_name, msg, msg))
            first = True
            for field in m.fields:
                field_name = normalize_field_name(field.name).title()
                if field.invalid:
                    value = "<>"
                else:
                    if field.type == 'char':
                        value = "(others => 'A')"
                    else:
                        if field.array_length:
                            value = field_array_values[field.type]
                        elif field.enum:
                            if field.enum in bitmasks:
                                value = "<>"
                            else:
                                value = "Mavlink_v2.%s.%s'First" % (find_package(xml, field.enum), normalize_enum_name(field.enum).title())
                        else:
                            value = field_values[field.type]

                if first: 
                    first = False
                    f.write("%s => %s" % (field_name, value))
                else:
                    f.write(",\n          %s => %s" % (field_name, value))

            f.write(""");
   begin
      Encode (O, Out_Connect, Buffer, Last);

      for Index in Buffer'First .. Last loop
         Res := Parse_Byte (In_Connect, Buffer (Index));
      end loop;
      pragma Assert (Res);

      Decode (I, In_Connect, Res);
      pragma Assert (Res);
      pragma Assert (I = O);
   end;

""")
    f.write("end Test;")

# Generate V2
def generate_v2(directory, xml):
    '''generate complete Ada implementation for v2'''
    mavparse.mkdir_p(directory)

    global msgs
    global types
    global filelist
    global types_size
    global types_files
    bitmasks = []
    calculate_types_size(xml)

    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'Ada')
    shutil.copy(os.path.join(srcpath, "x25crc.ads"), directory)
    shutil.copy(os.path.join(srcpath, "x25crc.adb"), directory)
    srcpath = os.path.join(basepath, 'Ada', 'v2')
    shutil.copy(os.path.join(srcpath, "mavlink_v2.gpr"), directory)
    shutil.copy(os.path.join(srcpath, "mavlink_v2.ads"), directory)
    shutil.copy(os.path.join(srcpath, "mavlink_v2.adb"), directory)
    shutil.copy(os.path.join(srcpath, "sha_256.ads"), directory)
    shutil.copy(os.path.join(srcpath, "sha_256.adb"), directory)

    for x in xml:
        name = os.path.splitext(os.path.basename(x.filename))[0]
        print("Generate " + name.title())

        # Create spec file
        spec = open(os.path.join(directory, "mavlink_v2-" + name + ".ads"), "w")
        spec.write(GEN)

        generate_includes (x, spec)

        spec.write("package %s.%s is\n\n" % (v2, name.title()))

        # Generate types
        for t in x.enum:
            size = types_size[t.name]
            if t.bitmask:
                bitmasks.append(t.name)
                if size is None:
                    size = calculate_bitmask_size(t)
                spec.write(repr_bitmask(t, size))
            else:
                if size is None:
                    size = calculate_enum_size(t)
                spec.write(repr_enum(t, size))
            spec.write("\n")

        spec.write("end %s.%s;\n" % (v2, name.title()))

        # Mavlink.Common.Message
        f_name = os.path.join(directory, "mavlink_v2-%s-message" % name)
        p_name = "%s.%s.Message" % (v2, name.title())
        spec = open(f_name + ".ads", "w")
        spec.write(GEN)
        spec.write("\npackage %s is\n\nend %s;\n" % (p_name, p_name))

        #Generate messages
        for m in x.message:
            f_name, p_name = get_pakage_name(x, m, directory)

            spec = open(f_name + ".ads", "w")
            spec.write(GEN)
            if m.deprecated:
                spec.write(get_deprecated(m.deprecated, 0))
            spec.write(format_comment (m.description, 0))

            # not direct includes
            included = [x]
            field_types = [field.enum for field in m.fields if field.enum]
            for t in field_types:
                if t in types_files and types_files[t] not in included and types_files[t].filename not in x.include:
                    if len(included) == 1: spec.write("\n")
                    included.append(types_files[t])
                    inc = os.path.splitext(os.path.basename(types_files[t].filename))[0].title()
                    spec.write("with %s.%s; use %s.%s;\n" % (v2, inc, v2, inc))

            spec.write("\npackage %s is\n\n" % p_name)

            body = open(f_name + ".adb", "w")
            body.write(GEN)
            body.write("pragma Ada_2022;\n\npackage body %s is\n\n" % p_name)

            generate_message(m, spec, body)

            spec.write("end %s;\n" % p_name)
            body.write("end %s;\n" % p_name)

    generate_test(directory, xml, bitmasks)