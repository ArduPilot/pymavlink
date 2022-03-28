#!/usr/bin/env python

'''parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

General process:
 - each filename passed in:
    - may be validated, based on --validate
    - is parsed using mavparse.MAVXML into an xml document and appended to a list, "xml"

 - expand_includes is called to do a breadth-first search of the xml
    documents structure inferred by the <include> tags in each
    document, expanding the xml list from its base (just the ones on
    the commandline) to the entire structure

 - update_includes works on the xml list created by expand_includes
    - any xml document with no includes is added to the "done" list (there must be at least one of these)
    - it repeatedly calls update_one_iteration
    - each iteration is intended to include the crcs and other information from includes into the xml document doing the include

'''

from __future__ import print_function
from future import standard_library
standard_library.install_aliases()
from builtins import object
import copy
import operator
import os
import sys
from . import mavparse

# Set defaults for generating MAVLink code
DEFAULT_WIRE_PROTOCOL = mavparse.PROTOCOL_1_0
DEFAULT_LANGUAGE = 'Python'
DEFAULT_ERROR_LIMIT = 200
DEFAULT_VALIDATE = True
DEFAULT_STRICT_UNITS = False

MAXIMUM_INCLUDE_FILE_NESTING = 5

# List the supported languages. This is done globally because it's used by the GUI wrapper too
# Right now, 'JavaScript' ~= 'JavaScript_Stable', in the future it may be made equivalent to 'JavaScript_NextGen'
supportedLanguages = ["C", "CS", "JavaScript", "JavaScript_Stable","JavaScript_NextGen", "TypeScript", "Python", "Lua", "WLua", "ObjC", "Swift", "Java", "C++11"]


class MAVXMLs(object):
    '''a collection of parsed MAVXML objects, rooted at filename'''
    def __init__(self, filename, wire_protocol=None, validate=True, validate_strict_units=True):

        self.filepath = os.path.abspath(filename)

        # dictionary of filepath -> MAVXML (parsed XML object)
        self.MAVXML = {}

        # parse all reachable include files:
        self.process_filepath(
            self.filepath,
            all_files=set(),
            wire_protocol=wire_protocol,
            validate=validate,
            validate_strict_units=validate_strict_units
        )

        # populate the traditional aggregate values (e.g. aggregate
        # list of messages)
        self.create_aggregate_values()

    def process_filepath(self, filepath, all_files, wire_protocol=None, validate=True, validate_strict_units=True):
        '''turn filepath into a MAVXML object - and then do the same for all
        its includes.  Populates self.MAVXML with those objects, keyed
        by filepath
        '''
        # Only parse new include files
        if filepath in self.MAVXML:
            return
        self.MAVXML[filepath] = mavparse.MAVXML(
            filepath,
            wire_protocol,
            validate=validate,
            validate_strict_units=validate_strict_units)

        # recurse into our own includes
        for include in self.MAVXML[filepath].include:
            if os.path.isabs(include):
                include_filepath = include
            else:
                include_filepath = os.path.join(os.path.dirname(filepath), include)
            self.process_filepath(
                include_filepath,
                all_files,
                wire_protocol=wire_protocol,
                validate=validate,
                validate_strict_units=validate_strict_units)

    def traverse_filepath(self, filepath, function, seen=None):
        if seen is None:
            seen = set()
        '''calls function on document structure inferred by <include>, returns list of results'''
        ret = []
        for include in self.MAVXML[filepath].include:
            if os.path.isabs(include):
                include_fpath = include
            else:
                include_fpath = os.path.join(os.path.dirname(filepath), include)
            if include_fpath in seen:
                continue
            seen.add(include_fpath)
            ret.extend(self.traverse_filepath(include_fpath, function, seen=seen))
        ret.append(function(self, filepath))
        return ret

    def traverse(self, function, seen=None):
        return self.traverse_filepath(self.filepath, function=function, seen=seen)

    def total_msgs(self):
        '''returns a count of messages in this document and all documents in
        its include_MAVXML list'''

        def msg_count(self, filepath):
            return len(self.MAVXML[filepath].message)

        counts = self.traverse(msg_count)
        return reduce(lambda a, b : a + b, counts)

    def total_filecount(self):
        '''returns number of files parsed to create this MAVXMLs object'''
        def file_count(self, filepath):
            return 1
        counts = self.traverse(file_count)
        return len(counts)

    def aggregate_value_keybits(self):
        return [
            "crcs",
            "lengths",
            "min_lengths",
            "flags",
            "target_system_ofs",
            "target_component_ofs",
            "names",
        ]

    def create_aggregate_values(self):
        # hashes by message-id:
        class Aggregates(object):
            pass
        self.aggregates = Aggregates()
        for keybit in self.aggregate_value_keybits():
            mykey = "message_" + keybit
            setattr(self.aggregates, mykey, dict())

        self.largest_payload = 0

        def update_aggregate_values(self, filepath):
            print("Merging %s" % os.path.basename(filepath))
            mavxml = self.MAVXML[filepath]

            for keybit in self.aggregate_value_keybits():
                mykey = "message_" + keybit
                theirkey = "message_" + keybit
                mine = getattr(self.aggregates, mykey)
                theirs = getattr(mavxml, theirkey)
                mine.update(theirs)

            if mavxml.largest_payload > self.largest_payload:
                self.largest_payload = mavxml.largest_payload

        self.traverse(update_aggregate_values)

        # now sort aggregate values:


    def filelist(self):
        '''returns a list of files used to create this MAVXMLs object'''

        def filepath(self, filepath):
            return os.path.basename(filepath)

        return sorted(self.traverse(filepath))

    def all_messages(self):
        '''returns messages from all files used to create this MAVXML object'''

        def message(self, filepath):
            return self.MAVXML[filepath].message

        ret = []
        for l in self.traverse(message):
            ret.extend(l)

        return sorted(ret, key=operator.attrgetter('name'))

    def aggregate(self, filepath, attribute_name):
        '''walks filepath and its includes, returns mapping from message id to
        attribute value for supplied attribute.  So if attribute_name
        is "message_crc", the return hash will have a mapping from id
        to crc for each message in the XML corresponding to filepath
        and all its includes.
        '''
        ret = {}

        def getter(self, filepath2):
            ret.update(getattr(self.MAVXML[filepath2], attribute_name))

        self.traverse_filepath(filepath, getter)

        return ret

    def aggregate_enums_collect(self, filepath, seen):
        seen.add(filepath)

        def do_merge_enum(enum1, enum2):
            '''merge entries from enum2 back into enum1'''
            enum1_entry_by_name = {}
            enum1_entry_by_value = {}
            for entry in enum1.entry:
                enum1_entry_by_name[entry.name] = enum
                enum1_entry_by_value[entry.value] = enum
            for entry in enum2.entry:
                if entry.name in enum1_entry_by_name:
                    print("%s already has a %s" % (enum1.name, entry.name))
                    sys.exit(1)
                if entry.value in enum1_entry_by_value:
                    print("%s already has value %u" % (enum1.name, entry.value))
                    sys.exit(1)
                enum1.entry.append(copy.deepcopy(entry))

        enums_by_name = {}

        must_emit_enum = {}

        mavxml = self.MAVXML[filepath]

        ret = []  # order must be preserved

        for include in mavxml.include:
            if os.path.isabs(include):
                include_fpath = include
            else:
                include_fpath = os.path.join(os.path.dirname(filepath), include)
            if include_fpath in seen:
                continue
            seen.add(include_fpath)
            for enum in self.aggregate_enums_collect(include_fpath, seen):
                if enum.name not in enums_by_name:
                    enums_by_name[enum.name] = copy.deepcopy(enum)
                    ret.append(enums_by_name[enum.name])
                    continue

                must_emit_enum[enum.name] = True
                do_merge_enum(enums_by_name[enum.name], enum)

        for enum in mavxml.enum:
            must_emit_enum[enum.name] = True
            if enum.name not in enums_by_name:
                enums_by_name[enum.name] = copy.deepcopy(enum)
                ret.append(enums_by_name[enum.name])
                continue

            do_merge_enum(enums_by_name[enum.name], enum)

        return filter(lambda x : x.name in must_emit_enum, ret)


    def aggregate_enums(self, filepath):
        '''returns enumerations from all files used to create this MAVXML
        object.  Enumeration values will be merged in each enum'''

        ret = self.aggregate_enums_collect(filepath, set())

        # sort values numerically, add end marker
        for enum in ret:
            enum.entry = sorted(enum.entry, key=lambda x : x.value)
            # add on the "enum end" bit:
            enum.entry.append(mavparse.MAVEnumEntry("%s_ENUM_END" % enum.name, enum.entry[-1].value+1, end_marker=True))

        return ret

class MAVGen(object):
    '''the MAVGen script'''

    def __init__(self, filepath, opts):
        self.filepath = filepath
        self.wire_protocol = opts.wire_protocol
        self.validate = opts.validate
        self.validate_strict_units = opts.strict_units
        self.output = opts.output

        # for emit:
        self.language = opts.language.lower()
        print('self.language=%s' % self.language)

    def parse(self):
        # construct MAVXML objects and container for same:
        self.xml = MAVXMLs(
            self.filepath,
            wire_protocol=self.wire_protocol,
            validate=self.validate,
            validate_strict_units=self.validate_strict_units
        )

    def run(self):
        '''replacement mavgen XML wrangling'''

        self.parse()

        print("Found %u MAVLink message types in %u XML files" %
              (self.xml.total_msgs(), self.xml.total_filecount()))

        if self.language == 'c':
            from . import mavgen_c
            mavgen_c.generate(
                self.output,
                self.xml,
                wire_protocol=self.wire_protocol
            )
        else:
            print("Unsupported language %s" % self.language)
            return False

        return True

def mavgen(opts, args):
    """Generate mavlink message formatters and parsers (C and Python ) using options
    and args where args are a list of xml files. This function allows python
    scripts under Windows to control mavgen using the same interface as
    shell scripts under Unix"""

    if opts.language.lower() == 'c':
        if len(args) != 1:
            raise ValueError("Expected a single XML file to process")
        mg = MAVGen(args[0], opts)
        return mg.run()

    if len(args) != 1:
        raise ValueError("Expected a single XML file to process")

    xml = []
    all_files = set()

    def expand_includes():
        """Expand includes. Root files already parsed objects in the xml list."""

        def expand_oneiteration():
            '''takes the list of xml files to process and finds includes which
            have not already been turned into xml documents added to
            xml files to process, turns them into xml documents and
            adds them to the xml files list.  Returns false if no more
            documents were added.
            '''
            includeadded = False
            for x in xml[:]:
                for i in x.include:
                    fname = os.path.abspath(os.path.join(os.path.dirname(x.filename), i))
                    # Only parse new include files
                    if fname in all_files:
                        continue
                    # Parsing
                    print("Parsing %s" % fname)
                    xml.append(mavparse.MAVXML(fname, opts.wire_protocol, validate=opts.validate, validate_strict_units=opts.strict_units))
                    all_files.add(fname)
                    includeadded = True
            return includeadded

        for i in range(MAXIMUM_INCLUDE_FILE_NESTING):
            if not expand_oneiteration():
                break

        if mavparse.check_duplicates(xml):
            sys.exit(1)

    def update_includes():
        """Update dialects with crcs etc of included files.  Included files
        were already found and parsed into xml list in
        expand_includes().
        """

        # 1: Mark files that don't have includes as "done"
        done = []
        for x in xml:
            #print("\n",x)
            if len(x.include) == 0:
                done.append(x)
                #print("\nFile with no includes found (ENDPOINT): %s" % x.filename )
        if len(done) == 0:
            print("\nERROR in includes tree, no base found!")
            exit(1)

        #print("\n",done)

        # 2: Update all 'not done' files for which all includes have
        # been done.  Returns True if any updates were made
        def update_oneiteration():
            initial_done_length = len(done)
            for x in xml:
                #print("\nCHECK %s" % x.filename)
                if x in done:
                    #print("  already done, skip")
                    continue
                #check if all its includes were already done
                all_includes_done = True
                for i in x.include:
                    fname = os.path.abspath(os.path.join(os.path.dirname(x.filename), i))
                    if fname not in [d.filename for d in done]:
                        all_includes_done = False
                        break
                if not all_includes_done:
                    #print("  not all includes ready, skip")
                    continue
                #Found file where all includes are done
                done.append(x)
                #print("  all includes ready, add" )
                #now update it with the facts from all it's includes
                for i in x.include:
                    fname = os.path.abspath(os.path.join(os.path.dirname(x.filename), i))
                    #print("  include file %s" % i )
                    #Find the corresponding x
                    for ix in xml:
                        if ix.filename != fname:
                            continue
                        #print("    add %s" % ix.filename )
                        x.message_crcs.update(ix.message_crcs)
                        x.message_lengths.update(ix.message_lengths)
                        x.message_min_lengths.update(ix.message_min_lengths)
                        x.message_flags.update(ix.message_flags)
                        x.message_target_system_ofs.update(ix.message_target_system_ofs)
                        x.message_target_component_ofs.update(ix.message_target_component_ofs)
                        x.message_names.update(ix.message_names)
                        x.largest_payload = max(x.largest_payload, ix.largest_payload)
                        break

            if len(done) == len(xml):
                return False  # finished
            if len(done) == initial_done_length:
                # we've made no progress
                print("ERROR include tree can't be resolved, no base found!")
                exit(1)
            return True

        for i in range(MAXIMUM_INCLUDE_FILE_NESTING):
            #print("\nITERATION "+str(i))
            if not update_oneiteration():
                break

    # Process all XML files, validating them as necessary.
    for fname in args:
        # only add each dialect file argument once.
        if fname in all_files:
            continue
        all_files.add(fname)
        xml.append(mavparse.MAVXML(fname, opts.wire_protocol, validate=opts.validate, validate_strict_units=opts.strict_units))

    # expand includes
    expand_includes()
    update_includes()

    print("Found %u MAVLink message types in %u XML files" % (
        mavparse.total_msgs(xml), len(xml)))

    # convert language option to lowercase and validate
    opts.language = opts.language.lower()
    if opts.language == 'python':
        from . import mavgen_python
        mavgen_python.generate(opts.output, xml)
    elif opts.language == 'lua':
        from . import mavgen_lua
        mavgen_lua.generate(opts.output, xml)
    elif opts.language == 'wlua':
        from . import mavgen_wlua
        mavgen_wlua.generate(opts.output, xml)
    elif opts.language == 'cs':
        from . import mavgen_cs
        mavgen_cs.generate(opts.output, xml)
    elif (opts.language == 'javascript' ) or ( opts.language == 'javascript_stable' ):
        from . import mavgen_javascript_stable as mavgen_javascript
        mavgen_javascript.generate(opts.output, xml)
    elif opts.language == 'javascript_nextgen':
        from . import mavgen_javascript
        mavgen_javascript.generate(opts.output, xml)
    elif opts.language == 'typescript':
        from . import mavgen_typescript
        mavgen_typescript.generate(opts.output, xml)
    elif opts.language == 'objc':
        from . import mavgen_objc
        mavgen_objc.generate(opts.output, xml)
    elif opts.language == 'swift':
        from . import mavgen_swift
        mavgen_swift.generate(opts.output, xml)
    elif opts.language == 'java':
        from . import mavgen_java
        mavgen_java.generate(opts.output, xml)
    elif opts.language == 'c++11':
        from . import mavgen_cpp11
        mavgen_cpp11.generate(opts.output, xml)
    else:
        print("Unsupported language %s" % opts.language)

    return True


# build all the dialects in the dialects subpackage
class Opts(object):
    def __init__(self, output, wire_protocol=DEFAULT_WIRE_PROTOCOL, language=DEFAULT_LANGUAGE, validate=DEFAULT_VALIDATE, error_limit=DEFAULT_ERROR_LIMIT, strict_units=DEFAULT_STRICT_UNITS):
        self.wire_protocol = wire_protocol
        self.error_limit = error_limit
        self.language = language
        self.output = output
        self.validate = validate
        self.strict_units = strict_units


def mavgen_python_dialect(dialect, wire_protocol):
    '''generate the python code on the fly for a MAVLink dialect'''
    dialects = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'dialects')
    mdef = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'message_definitions')
    if wire_protocol == mavparse.PROTOCOL_0_9:
        py = os.path.join(dialects, 'v09', dialect + '.py')
        xml = os.path.join(dialects, 'v09', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v0.9', dialect + '.xml')
    elif wire_protocol == mavparse.PROTOCOL_1_0:
        py = os.path.join(dialects, 'v10', dialect + '.py')
        xml = os.path.join(dialects, 'v10', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v1.0', dialect + '.xml')
    else:
        py = os.path.join(dialects, 'v20', dialect + '.py')
        xml = os.path.join(dialects, 'v20', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v1.0', dialect + '.xml')
    opts = Opts(py, wire_protocol)

    # Python 2 to 3 compatibility
    try:
        import StringIO as io
    except ImportError:
        import io

    # throw away stdout while generating
    stdout_saved = sys.stdout
    sys.stdout = io.StringIO()
    try:
        xml = os.path.relpath(xml)
        if not mavgen(opts, [xml]):
            sys.stdout = stdout_saved
            return False
    except Exception:
        sys.stdout = stdout_saved
        raise
    sys.stdout = stdout_saved
    return True

if __name__ == "__main__":
    raise DeprecationWarning("Executable was moved to pymavlink.tools.mavgen")
