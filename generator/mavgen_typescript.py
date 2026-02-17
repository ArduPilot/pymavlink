#!/usr/bin/env python3
"""
parse a MAVLink protocol XML file and generate a Node.js typescript module implementation

Based on original work Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
"""
import os
import re
from . import mavtemplate

t = mavtemplate.MAVTemplate()


def camelcase(str):
    parts = str.split('_')
    result = ''
    for part in parts:
        result += part.lower().capitalize()
    return result


def generate_enums(dir, enums):
    print("Generating enums")

    if not os.path.isdir(dir):
        os.mkdir(dir)
        
    for e in enums:
        filename = e.name.replace('_', '-')
        filename = filename.lower()
        with open('{}/{}.ts'.format(dir, filename), "w") as f:
            f.write("export enum {} {{\n".format(camelcase(e.name)))
            for entry in e.entry:
                f.write(
                    "\t{} = {}, /* {} */\n".format(entry.name, entry.value, entry.description.rstrip("\r").rstrip("\n")))
            f.write("}")


def generate_classes(dir, registry, msgs, xml):
    print("Generating class definitions")

    ts_types = {"uint8_t": "number", "uint16_t": "number", "uint32_t": "number", "uint64_t": "number",
                "int8_t": "number", "int16_t": "number", "int32_t": "number", "int64_t": "number",
                "float": "number", "double": "number", "char": "string"}

    if not os.path.isdir(dir):
        os.mkdir(dir)

    with open(registry, "w") as registry_f:
        registry_f.write("import {MavLinkData, MavLinkDataConstructor, MavLinkPacketRegistry} from 'node-mavlink';\n")
        for m in msgs:
            filename = m.name.replace('_', '-')
            filename = filename.lower()

            m.order_map = [0] * len(m.fieldnames)
            for i in range(0, len(m.fieldnames)):
                m.order_map[i] = m.ordered_fieldnames.index(m.fieldnames[i])

            with open('{}/{}.ts'.format(dir, filename), "w") as f:
                if xml.wire_protocol_version == '1.0':
                    raise Exception('WireProtocolException', 'Please use WireProtocol = 2.0 only.')

                f.write("import {MavLinkData, MavLinkPacketField} from 'node-mavlink';\n")
                #f.write("import {readInt64LE, readUInt64LE} from 'node-mavlink';\n")
                registry_f.write("import {{{}}} from './messages/{}';\n".format(camelcase(m.name), filename))
                imported_enums = []
                for enum in [field.enum for field in m.fields if field.enum != '']:
                    if enum not in imported_enums:
                        f.write("import {{{}}} from '../enums/{}';\n".format(camelcase(enum),
                                                                             enum.replace('_', '-').lower()))
                        imported_enums.append(enum)

                f.write("/*\n{}\n*/\n".format(m.description.strip()))

                f.write("export class {} extends MavLinkData {{\n".format(camelcase(m.name)))

                for field in m.fields:
                    if field.enum:
                        f.write("\t/* {} {} */\n".format(field.type, field.description.strip()))
                        f.write("\tpublic {}!: {};\n".format(field.name, camelcase(field.enum)))
                    else:
                        if field.array_length > 1 and field.type != "char":
                            f.write("\t/* {}[] {} */\n".format(field.type, field.description.strip()))
                            f.write("\tpublic {}!: {};\n".format(field.name, ts_types[field.type] + "[]"))
                        else:
                            f.write("\t/* {} {} */\n".format(field.type, field.description.strip()))
                            f.write("\tpublic {}!: {};\n".format(field.name, ts_types[field.type]))
                f.write("\tstatic MSG_ID: number = {};\n".format(m.id))
                f.write("\tstatic MSG_NAME: string = '{}';\n".format(m.name))
                f.write("\tstatic MAGIC_NUMBER: number = {};\n".format(m.crc_extra))

                i = 0
                f.write("\tstatic FIELDS: MavLinkPacketField[] = [\n")
                offset = 0
                for fieldname in m.ordered_fieldnames:
                    field = next(field for field in m.fields if field.name == fieldname)
                    if m.extensions_start is not None and i >= m.extensions_start:
                        extension = 'true'
                    else:
                        extension = 'false'
                    """
                    @param source original name of the field
                    @param name name of the field
                    @param offset field offset in the payload
                    @param extension true if it is an extension field, false otherwise
                    @param size size of either the field or the size of an element in the array if it is an array field
                    @param type type of the field (ends with [] if it is an array field)
                    @param units unit of the field
                    @param length for array fields this is the length of the array
                    """
                    f.write("\t\t/* {} */\n".format(field.description.strip()))
                    if field.array_length > 1:
                        f.write("\t\tnew MavLinkPacketField('{}', '{}', {}, {}, {}, '{}', '{}',{}),\n".format(field.name, field.name, field.wire_offset, extension, field.type_length, field.type +"[]", field.units, field.array_length))
                        offset += field.type_length * field.array_length
                    else:
                        f.write("\t\tnew MavLinkPacketField('{}', '{}', {}, {}, {}, '{}', '{}'),\n".format(field.name, field.name, field.wire_offset, extension, field.type_length, field.type, field.units))
                        offset += field.type_length
                    i += 1
                f.write("\t];\n".format("', '".join(m.ordered_fieldnames)))
                f.write("\tstatic PAYLOAD_LENGTH: number = {};\n".format(offset))
                f.write("}")

        registry_f.write(
            "export const messageRegistry = new Map<number, new (system_id: number, component_id: number) => MavLinkData>([\n")
        for m in msgs:
            registry_f.write("\t[{}, {}],\n".format(m.id, camelcase(m.name)))
        registry_f.write("]);\n\n")
        registry_f.write("export function getCustomMagicNumbers() {\n")
        registry_f.write("\tconst record: Record<string, number> = {};\n")
        registry_f.write("\tArray.from(messageRegistry.entries())\n")
        registry_f.write("\t\t.forEach(([msgId, constructor]) => {\n")
        registry_f.write("\t\t\trecord[msgId.toString()] = (constructor as MavLinkDataConstructor<any>).MAGIC_NUMBER;\n")
        registry_f.write("\t\t});\n")
        registry_f.write("\treturn record;\n")
        registry_f.write("}\n")


def generate_tsconfig(basename):
    with open('{}/tsconfig.json'.format(basename), "w") as f:
        f.write(
            "{\n  \"compilerOptions\": {\n    \"target\": \"es5\",\n    \"module\": \"commonjs\","
            "\n    \"declaration\": true,\n    \"declarationMap\": true,\n    \"sourceMap\": true,"
            "\n    \"outDir\": \"./\",\n    \"strict\": true,\n    \"esModuleInterop\": true\n  },"
            "\n  \"include\": [\n    \"./\"\n  ],\n  \"exclude\": [\n    \"**/*.d.ts\","
            "\n    \"**/*.d.ts.map\",\n    \"**/*.js\",\n    \"**/*.js.map\"\n  ]\n}")


def generate(basename, xml):
    enums_dir = basename + '/enums'
    messages_dir = basename + '/messages'
    message_registry = basename + '/message-registry.ts'

    msgs = []
    enums = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        enums.extend(x.enum)
        filelist.append(os.path.basename(x.filename))

    generate_enums(enums_dir, enums)
    generate_classes(messages_dir, message_registry, msgs, xml[0])
    generate_tsconfig(basename)
