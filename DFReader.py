#!/usr/bin/env python
'''
APM DataFlash log file reader

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

Partly based on SDLog2Parser by Anton Babushkin
'''
from __future__ import print_function
from builtins import range
from builtins import object

import array
import math
import sys
import os
import mmap
import platform
import time

import struct
import gzip
import io

from . import mavutil

try:
    long        # Python 2 has long
except NameError:
    long = int  # But Python 3 does not

FORMAT_TO_STRUCT = {
    "a": ("64s", None, str),
    "b": ("b", None, int),
    "B": ("B", None, int),
    "h": ("h", None, int),
    "H": ("H", None, int),
    "i": ("i", None, int),
    "I": ("I", None, int),
    "f": ("f", None, float),
    "n": ("4s", None, str),
    "N": ("16s", None, str),
    "Z": ("64s", None, str),
    "c": ("h", 0.01, float),
    "C": ("H", 0.01, float),
    "e": ("i", 0.01, float),
    "E": ("I", 0.01, float),
    "L": ("i", 1.0e-7, float),
    "d": ("d", None, float),
    "M": ("b", None, int),
    "q": ("q", None, long),  # Backward compat
    "Q": ("Q", None, long),  # Backward compat
    }

MULT_TO_PREFIX = {
    0: "",
    1: "",
    1.0e-1: "d", # deci
    1.0e-2: "c", # centi
    1.0e-3: "m", # milli
    1.0e-6: "µ", # micro
    1.0e-9: "n"  # nano
}

def u_ord(c):
	return ord(c) if sys.version_info.major < 3 else c

def is_quiet_nan(val):
    '''determine if the argument is a quiet nan'''
    # Is this a float, and some sort of nan?
    if isinstance(val, float) and math.isnan(val):
        # quiet nans have more non-zero values:
        if sys.version_info.major >= 3:
            noisy_nan = bytearray([0x7f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        else:
            noisy_nan = "\x7f\xf8\x00\x00\x00\x00\x00\x00"
        return struct.pack(">d", val) != noisy_nan
    else:
        return False

class DFFormat(object):
    def __init__(self, type, name, flen, format, columns, oldfmt=None):
        self.type = type
        self.name = null_term(name)
        self.len = flen
        self.format = format
        self.columns = columns.split(',')
        self.instance_field = None
        self.units = None

        if self.columns == ['']:
            self.columns = []

        msg_struct = "<"
        msg_mults = []
        msg_types = []
        msg_fmts = []
        for c in format:
            if u_ord(c) == 0:
                break
            try:
                msg_fmts.append(c)
                (s, mul, type) = FORMAT_TO_STRUCT[c]
                msg_struct += s
                msg_mults.append(mul)
                if c == "a":
                    msg_types.append(array.array)
                else:
                    msg_types.append(type)
            except KeyError as e:
                print("DFFormat: Unsupported format char: '%s' in message %s" %
                      (c, name))
                raise Exception("Unsupported format char: '%s' in message %s" %
                                (c, name))

        self.msg_struct = msg_struct
        self.msg_types = msg_types
        self.msg_mults = msg_mults
        self.msg_fmts = msg_fmts
        self.colhash = {}
        for i in range(len(self.columns)):
            self.colhash[self.columns[i]] = i
        self.a_indexes = []
        for i in range(0, len(self.msg_fmts)):
            if self.msg_fmts[i] == 'a':
                self.a_indexes.append(i)

        # If this format was alrady defined, copy over units and instance info
        if oldfmt is not None:
            self.units = oldfmt.units
            if oldfmt.instance_field is not None:
                self.set_instance_field(self.colhash[oldfmt.instance_field])

    def set_instance_field(self, instance_idx):
        '''set up the instance field for this format'''
        self.instance_field = self.columns[instance_idx]
        # work out offset and length of instance field in message
        pre_fmt = self.format[:instance_idx]
        pre_sfmt = ""
        for c in pre_fmt:
            (s, mul, type) = FORMAT_TO_STRUCT[c]
            pre_sfmt += s
        self.instance_ofs = struct.calcsize(pre_sfmt)
        (ifmt,) = self.format[instance_idx]
        self.instance_len = struct.calcsize(ifmt)

    def set_unit_ids(self, unit_ids, unit_lookup):
        '''set unit IDs string from FMTU'''
        if unit_ids is None:
            return
        # Does this unit string define an instance field?
        instance_idx = unit_ids.find('#')
        if instance_idx != -1:
            self.set_instance_field(instance_idx)
        # Build the units array from the IDs
        self.units = [""]*len(self.columns)
        for i in range(len(self.columns)):
            if i < len(unit_ids):
                if unit_ids[i] in unit_lookup:
                    self.units[i] = unit_lookup[unit_ids[i]]

    def set_mult_ids(self, mult_ids, mult_lookup):
        '''set mult IDs string from FMTU'''
        # Update the units based on the multiplier
        for i in range(len(self.units)):
            # If the format has its own multiplier, do not adjust the unit,
            # and if no unit is specified there is nothing to adjust
            if self.msg_mults[i] is not None or self.units[i] == "":
                continue
            # Get the unit multiplier from the lookup table
            if mult_ids[i] in mult_lookup:
                unitmult = mult_lookup[mult_ids[i]]
                # Combine the multipler and unit to derive the real unit
                if unitmult in MULT_TO_PREFIX:
                    self.units[i] = MULT_TO_PREFIX[unitmult]+self.units[i]
                else:
                    self.units[i] = "%.4g %s" % (unitmult, self.units[i])

    def get_unit(self, col):
        '''Return the unit for the specified field'''
        if self.units is None:
            return ""
        else:
            idx = self.colhash[col]
            return self.units[idx]

    def __str__(self):
        return ("DFFormat(%s,%s,%s,%s)" %
                (self.type, self.name, self.format, self.columns))

# Swiped into mavgen_python.py
def to_string(s):
    '''desperate attempt to convert a string regardless of what garbage we get'''
    if isinstance(s, str):
        return s
    if sys.version_info[0] == 2:
        # In python2 we want to return unicode for passed in unicode
        return s
    return s.decode(errors="backslashreplace")

def null_term(string):
    '''null terminate a string'''
    if isinstance(string, bytes):
        string = to_string(string)
    idx = string.find("\0")
    if idx != -1:
        string = string[:idx]
    return string


class DFMessage(object):
    def __init__(self, fmt, elements, apply_multiplier, parent):
        self.fmt = fmt
        self._elements = elements
        self._apply_multiplier = apply_multiplier
        self._fieldnames = fmt.columns
        self._parent = parent

    def to_dict(self):
        d = {'mavpackettype': self.fmt.name}

        for field in self._fieldnames:
            d[field] = self.__getattr__(field)

        return d

    def __getattr__(self, field):
        '''override field getter'''
        try:
            i = self.fmt.colhash[field]
        except Exception:
            raise AttributeError(field)
        if self.fmt.msg_fmts[i] == 'Z' and self.fmt.name == 'FILE':
            # special case for FILE contens as bytes
            return self._elements[i]
        if isinstance(self._elements[i], bytes):
            try:
                v = self._elements[i].decode("utf-8")
            except UnicodeDecodeError:
                # try western europe
                v = self._elements[i].decode("ISO-8859-1")
        else:
            v = self._elements[i]
        if self.fmt.format[i] == 'a':
            pass
        elif self.fmt.format[i] != 'M' or self._apply_multiplier:
            v = self.fmt.msg_types[i](v)
        if self.fmt.msg_types[i] == str:
            v = null_term(v)
        if self.fmt.msg_mults[i] is not None and self._apply_multiplier:
            # For reasons relating to floating point accuracy, you get a more
            # accurate result by dividing by 1e2 or 1e7 than multiplying by
            # 1e-2 or 1e-7
            if self.fmt.msg_mults[i] > 0.0 and self.fmt.msg_mults[i] < 1.0:
                divisor = 1/self.fmt.msg_mults[i]
                v /= divisor
            else:
                v *= self.fmt.msg_mults[i]
        return v

    def __setattr__(self, field, value):
        '''override field setter'''
        if not field[0].isupper() or not field in self.fmt.colhash:
            super(DFMessage,self).__setattr__(field, value)
        else:
            i = self.fmt.colhash[field]
            if self.fmt.msg_mults[i] is not None and self._apply_multiplier:
                value /= self.fmt.msg_mults[i]
            self._elements[i] = value

    def get_type(self):
        return self.fmt.name

    def __str__(self):
        is_py3 = sys.version_info >= (3,0)
        ret = "%s {" % self.fmt.name
        col_count = 0
        for c in self.fmt.columns:
            val = self.__getattr__(c)
            if is_quiet_nan(val):
                val = "qnan"
            # Add the value to the return string
            if is_py3:
                ret += "%s : %s, " % (c, val)
            else:
                try:
                    ret += "%s : %s, " % (c, val)
                except UnicodeDecodeError:
                    ret += "%s : %s, " % (c, to_string(val))
            col_count += 1
        if col_count != 0:
            ret = ret[:-2]
        return ret + '}'

    def dump_verbose_bitmask(self, f, c, val, field_metadata):
        try:
            try:
                bitmask = field_metadata["bitmask"]
            except Exception:
                return

            # work out how many bits to show:
            t = field_metadata.get("type")
            bit_count = None
            if t == "uint8_t":
                bit_count = 8
            elif t == "uint16_t":
                bit_count = 16
            elif t == "uint32_t":
                bit_count = 32

            if bit_count is None:
                return

            highest = -1

            # we show bit values at least up to the highest bit set:
            for i in range(bit_count):
                if val & (1<<i):
                    highest = i

            # we show bit values at least up until the highest
            # bit we have a description for:
            for bit in bitmask.bit:
                bit_offset = int(math.log(bit["value"], 2))
                if bit_offset > highest:
                    highest = bit_offset

            for i in range(highest+1):
                bit_value = 1 << i
                if val & bit_value:
                    bang = " "
                else:
                    bang = "!"
                done = False
                for bit in bitmask.bit:
                    if bit["value"] != bit_value:
                        continue
                    bit_name = bit.get('name')
                    f.write("      %s %s" % (bang, bit_name,))
                    if hasattr(bit, 'description'):
                        f.write(" (%s)\n" % bit["description"])
                    else:
                        f.write("\n")
                    done = True
                    break
                if not done:
                    f.write("      %s UNKNOWN_BIT%d\n" % (bang, i))
        except Exception as e:
            # print(e)
            pass

    def dump_verbose(self, f):
        is_py3 = sys.version_info >= (3,0)
        timestamp = "%s.%03u" % (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self._timestamp)),
            int(self._timestamp*1000.0)%1000)
        f.write("%s: %s\n" % (timestamp, self.fmt.name))

        field_metadata_by_name = {}
        try:
            metadata_tree = self._parent.metadata.metadata_tree()
            metadata = metadata_tree[self.fmt.name]
            for fm in metadata["fields"].field:
                field_metadata_by_name[fm.get("name")] = fm
        except Exception as e:
            # print(e)
            pass

        for c in self.fmt.columns:
            # Get the value
            val = self.__getattr__(c)
            # Handle quiet nan
            if is_quiet_nan(val):
                val = "qnan"
            # Output the field label and value
            if is_py3:
                f.write("    %s: %s" % (c, val))
            else:
                try:
                    f.write("    %s: %s" % (c, val))
                except UnicodeDecodeError:
                    f.write("    %s: %s" % (c, to_string(val)))

            # see if this is an enumeration entry, emit enumeration
            # entry name if it is
            if c in field_metadata_by_name:
                fm = field_metadata_by_name[c]
                fm_enum = getattr(fm, "enum", None)
                if fm_enum is not None:
                    enum_entry_name = "?????"  # default, "not found" value
                    for entry in fm_enum.iterchildren():
                        if int(entry.value) == int(val):
                            enum_entry_name = entry.get('name')
                            break

                    f.write(f" ({enum_entry_name})")

            # Append the unit to the output
            unit = self.fmt.get_unit(c)
            if unit.startswith("rad"):
                # For rad or rad/s, add the degrees conversion too
                f.write(" %s (%s %s)" % (unit, math.degrees(val), unit.replace("rad","deg")))
            else:
                # Append the unit
                f.write(" %s" % (unit))

            # output the newline
            f.write("\n")

            # if this is a bitmask then print out all bits set:
            if c in field_metadata_by_name:
                self.dump_verbose_bitmask(f, c, val, field_metadata_by_name[c])

    def get_msgbuf(self):
        '''create a binary message buffer for a message'''
        values = []
        is_py2 = sys.version_info < (3,0)
        for i in range(len(self.fmt.columns)):
            if i >= len(self.fmt.msg_mults):
                continue
            mul = self.fmt.msg_mults[i]
            name = self.fmt.columns[i]
            if name == 'Mode' and 'ModeNum' in self.fmt.columns:
                name = 'ModeNum'
            v = self.__getattr__(name)
            if is_py2:
                if isinstance(v,unicode): # NOQA
                    v = str(v)
                elif isinstance(v, array.array):
                    v = v.tostring()
            else:
                if isinstance(v,str):
                    try:
                        v = bytes(v,'ascii')
                    except UnicodeEncodeError:
                        v = v.encode()
                elif isinstance(v, array.array):
                    v = v.tobytes()
            if mul is not None:
                v /= mul
                v = int(round(v))
            values.append(v)

        ret1 = struct.pack("BBB", 0xA3, 0x95, self.fmt.type)
        try:
            ret2 = struct.pack(self.fmt.msg_struct, *values)
        except Exception as ex:
            return None
        return ret1 + ret2

    def get_fieldnames(self):
        return self._fieldnames

    def __getitem__(self, key):
        '''support indexing, allowing for multi-instance sensors in one message'''
        if self.fmt.instance_field is None:
            raise IndexError()
        k = '%s[%s]' % (self.fmt.name, str(key))
        if not k in self._parent.messages:
            raise IndexError()
        return self._parent.messages[k]


class DFReaderClock(object):
    '''base class for all the different ways we count time in logs'''

    def __init__(self):
        self.set_timebase(0)
        self.timestamp = 0

    def _gpsTimeToTime(self, week, msec):
        '''convert GPS week and TOW to a time in seconds since 1970'''
        epoch = 86400*(10*365 + int((1980-1969)/4) + 1 + 6 - 2)
        return epoch + 86400*7*week + msec*0.001 - 18

    def set_timebase(self, base):
        self.timebase = base

    def message_arrived(self, m):
        pass

    def rewind_event(self):
        pass


class DFReaderClock_usec(DFReaderClock):
    '''DFReaderClock_usec - use microsecond timestamps from messages'''
    def __init__(self):
        DFReaderClock.__init__(self)

    def find_time_base(self, gps, first_us_stamp):
        '''work out time basis for the log - even newer style'''
        t = self._gpsTimeToTime(gps.GWk, gps.GMS)
        self.set_timebase(t - gps.TimeUS*0.000001)
        # this ensures FMT messages get appropriate timestamp:
        self.timestamp = self.timebase + first_us_stamp*0.000001

    def type_has_good_TimeMS(self, type):
        '''The TimeMS in some messages is not from *our* clock!'''
        if type.startswith('ACC'):
            return False
        if type.startswith('GYR'):
            return False
        return True

    def should_use_msec_field0(self, m):
        if not self.type_has_good_TimeMS(m.get_type()):
            return False
        if 'TimeMS' != m._fieldnames[0]:
            return False
        if self.timebase + m.TimeMS*0.001 < self.timestamp:
            return False
        return True

    def set_message_timestamp(self, m):
        if 'TimeUS' == m._fieldnames[0]:
            # only format messages don't have a TimeUS in them...
            m._timestamp = self.timebase + m.TimeUS*0.000001
        elif self.should_use_msec_field0(m):
            # ... in theory. I expect there to be some logs which are not
            # "pure":
            m._timestamp = self.timebase + m.TimeMS*0.001
        else:
            m._timestamp = self.timestamp
        self.timestamp = m._timestamp


class DFReaderClock_msec(DFReaderClock):
    '''DFReaderClock_msec - a format where many messages have TimeMS in
    their formats, and GPS messages have a "T" field giving msecs'''
    def find_time_base(self, gps, first_ms_stamp):
        '''work out time basis for the log - new style'''
        t = self._gpsTimeToTime(gps.Week, gps.TimeMS)
        self.set_timebase(t - gps.T*0.001)
        self.timestamp = self.timebase + first_ms_stamp*0.001

    def set_message_timestamp(self, m):
        if 'TimeMS' == m._fieldnames[0]:
            m._timestamp = self.timebase + m.TimeMS*0.001
        elif m.get_type() in ['GPS', 'GPS2']:
            m._timestamp = self.timebase + m.T*0.001
        else:
            m._timestamp = self.timestamp
        self.timestamp = m._timestamp


class DFReaderClock_px4(DFReaderClock):
    '''DFReaderClock_px4 - a format where a starting time is explicitly
    given in a message'''
    def __init__(self):
        DFReaderClock.__init__(self)
        self.px4_timebase = 0

    def find_time_base(self, gps):
        '''work out time basis for the log - PX4 native'''
        t = gps.GPSTime * 1.0e-6
        self.timebase = t - self.px4_timebase

    def set_px4_timebase(self, time_msg):
        self.px4_timebase = time_msg.StartTime * 1.0e-6

    def set_message_timestamp(self, m):
        m._timestamp = self.timebase + self.px4_timebase

    def message_arrived(self, m):
        type = m.get_type()
        if type == 'TIME' and 'StartTime' in m._fieldnames:
            self.set_px4_timebase(m)


class DFReaderClock_gps_interpolated(DFReaderClock):
    '''DFReaderClock_gps_interpolated - for when the only real references
    in a message are GPS timestamps'''
    def __init__(self):
        DFReaderClock.__init__(self)
        self.msg_rate = {}
        self.counts = {}
        self.counts_since_gps = {}

    def rewind_event(self):
        '''reset counters on rewind'''
        self.counts = {}
        self.counts_since_gps = {}

    def message_arrived(self, m):
        type = m.get_type()
        if type not in self.counts:
            self.counts[type] = 1
        else:
            self.counts[type] += 1
        # this preserves existing behaviour - but should we be doing this
        # if type == 'GPS'?
        if type not in self.counts_since_gps:
            self.counts_since_gps[type] = 1
        else:
            self.counts_since_gps[type] += 1

        if type == 'GPS' or type == 'GPS2':
            self.gps_message_arrived(m)

    def gps_message_arrived(self, m):
        '''adjust time base from GPS message'''
        # msec-style GPS message?
        gps_week = getattr(m, 'Week', None)
        gps_timems = getattr(m, 'TimeMS', None)
        if gps_week is None:
            # usec-style GPS message?
            gps_week = getattr(m, 'GWk', None)
            gps_timems = getattr(m, 'GMS', None)
            if gps_week is None:
                if getattr(m, 'GPSTime', None) is not None:
                    # PX4-style timestamp; we've only been called
                    # because we were speculatively created in case no
                    # better clock was found.
                    return

        if gps_week is None and hasattr(m,'Wk'):
            # AvA-style logs
            gps_week = getattr(m, 'Wk')
            gps_timems = getattr(m, 'TWk')
            if gps_week is None or gps_timems is None:
                return

        t = self._gpsTimeToTime(gps_week, gps_timems)

        deltat = t - self.timebase
        if deltat <= 0:
            return

        for type in self.counts_since_gps:
            rate = self.counts_since_gps[type] / deltat
            if rate > self.msg_rate.get(type, 0):
                self.msg_rate[type] = rate
        self.msg_rate['IMU'] = 50.0
        self.timebase = t
        self.counts_since_gps = {}

    def set_message_timestamp(self, m):
        rate = self.msg_rate.get(m.fmt.name, 50.0)
        if int(rate) == 0:
            rate = 50
        count = self.counts_since_gps.get(m.fmt.name, 0)
        m._timestamp = self.timebase + count/rate


class DFMetaData(object):
    '''handle dataflash messages metadata'''
    def __init__(self, parent):
        self.parent = parent
        self.data = None
        self.metadata_load_attempted = False

    def reset(self):
        '''clear cached data'''
        self.data = None
        self.metadata_load_attempted = False

    @staticmethod
    def dot_pymavlink(*args):
        '''return a path to store pymavlink data'''
        if 'HOME' not in os.environ:
            dir = os.path.join(os.environ['LOCALAPPDATA'], '.pymavlink')
        else:
            dir = os.path.join(os.environ['HOME'], '.pymavlink')
        if len(args) == 0:
            return dir
        return os.path.join(dir, *args)

    @staticmethod
    def download_url(url):
        '''download a URL and return the content'''
        if sys.version_info.major < 3:
            from urllib2 import urlopen as url_open
            from urllib2 import URLError as url_error
        else:
            from urllib.request import urlopen as url_open
            from urllib.error import URLError as url_error
        try:
            resp = url_open(url)
        except url_error as e:
            print('Error downloading %s : %s' % (url, e))
            return None
        return resp.read()

    @staticmethod
    def download():
        # Make sure the folder to store XML in has been created
        os.makedirs(DFMetaData.dot_pymavlink('LogMessages'), exist_ok=True)
        # Loop through vehicles to download
        for vehicle in ['Rover', 'Copter', 'Plane', 'Tracker', 'Blimp', 'Sub']:
            url = 'http://autotest.ardupilot.org/LogMessages/%s/LogMessages.xml.gz' % vehicle
            file = DFMetaData.dot_pymavlink('LogMessages', "%s.xml" % vehicle)
            print("Downloading %s as %s" % (url, file))
            data = DFMetaData.download_url(url)
            if data is None:
                continue
            # decompress it...
            with gzip.GzipFile(fileobj=io.BytesIO(data)) as gz:
                data = gz.read()
            try:
                open(file, mode='wb').write(data)
            except Exception as e:
                print("Failed to save to %s : %s" % (file, e))

    def metadata_tree(self, verbose=False):
        ''' return a map between a log message and its metadata. May return
        None if data is not available '''
        # If we've already tried loading data, use it if we have it
        # This avoid repeated attempts, when the file is not there
        if self.metadata_load_attempted:
            return self.data
        self.metadata_load_attempted = True
        # Get file name, based on vehicle type
        mapping = {mavutil.mavlink.MAV_TYPE_GROUND_ROVER : "Rover",
                   mavutil.mavlink.MAV_TYPE_FIXED_WING : "Plane",
                   mavutil.mavlink.MAV_TYPE_QUADROTOR : "Copter",
                   mavutil.mavlink.MAV_TYPE_HELICOPTER : "Copter",
                   mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER : "Tracker",
                   mavutil.mavlink.MAV_TYPE_SUBMARINE : "Sub",
                   mavutil.mavlink.MAV_TYPE_AIRSHIP : "Blimp",
                   }
        if self.parent.mav_type not in mapping:
            return None
        path = DFMetaData.dot_pymavlink("LogMessages", "%s.xml" % mapping[self.parent.mav_type])
        # Does the file exist?
        if not os.path.exists(path):
            if verbose:
                print("Can't find '%s'" % path)
                print("Please run 'logmessage download' from MAVExplorer, or call")
                print("DFMetaData.download() from Python.")
            return None
        # Read in the XML
        xml = open(path, 'rb').read()
        from lxml import objectify
        objectify.enable_recursive_str()
        tree = objectify.fromstring(xml)
        data = {}
        for p in tree.logformat:
            n = p.get('name')
            data[n] = p
        # Cache and return data
        self.data = data
        return self.data

    def print_help(self, msg):
        '''print help for a log message'''
        data = self.metadata_tree(verbose=True)
        if data is None:
            return
        if msg not in data:
            print("No help found for message: %s" % msg)
            return
        node = data[msg]
        # Message name and description
        print("Log Message: %s\n%s\n" % (msg, node.description.text))
        # Protect against replay messages which dont list their fields
        if not hasattr(node.fields, 'field'):
            return
        namelist = []
        unitlist = []
        # Loop through fields to build list of name/units
        for f in node.fields.field:
            namelist.append(f.get('name'))
            units = f.get('units')
            dtype = f.get('type')
            if units:
                unitlist.append("[%s] " % units)
            elif 'char' in dtype:
                unitlist.append("[%s] " % dtype)
            elif hasattr(f, 'enum'):
                unitlist.append("[enum] ")
            elif hasattr(f, 'bitmask'):
                unitlist.append("[bitmask] ")
            else:
                unitlist.append("")
        # Now get the max string length from each list
        namelen = len(max(namelist, key=len))
        unitlen = len(max(unitlist, key=len))
        # Loop through fields again to do the actual printing
        for i in range(0, len(namelist)):
            desc = node.fields.field[i].description.text
            print("%-*s %-*s: %s" % (namelen, namelist[i], unitlen, unitlist[i], desc))

    def get_description(self, msg):
        '''get the description of a log message'''
        data = self.metadata_tree()
        if data is None:
            return None
        if msg in data:
            return data[msg].description.text
        return ""


class DFReader(object):
    '''parse a generic dataflash file'''
    def __init__(self):
        # read the whole file into memory for simplicity
        self.clock = None
        self.timestamp = 0
        self.mav_type = mavutil.mavlink.MAV_TYPE_FIXED_WING
        self.verbose = False
        self.params = {}
        self._flightmodes = None
        self.messages = {
            'MAV': self,
            '__MAV__': self,  # avoids conflicts with messages actually called "MAV"
        }
        self.percent = 0
        self.unit_lookup = {}  # lookup table of units defined by UNIT messages
        self.mult_lookup = {}  # lookup table of multipliers defined by MULT messages
        self.metadata = DFMetaData(self)

    def _rewind(self):
        '''reset state on rewind'''
        # be careful not to replace self.messages with a new hash;
        # some people have taken a reference to self.messages and we
        # need their messages to disappear to.  If they want their own
        # copy they can copy.copy it!
        self.messages.clear()
        self.messages = {
            'MAV': self,
            '__MAV__': self,  # avoids conflicts with messages actually called "MAV"
        }
        if self._flightmodes is not None and len(self._flightmodes) > 0:
            self.flightmode = self._flightmodes[0][0]
        else:
            self.flightmode = "UNKNOWN"
        self.percent = 0
        if self.clock:
            self.clock.rewind_event()

    def init_clock_px4(self, px4_msg_time, px4_msg_gps):
        self.clock = DFReaderClock_px4()
        if not self._zero_time_base:
            self.clock.set_px4_timebase(px4_msg_time)
            self.clock.find_time_base(px4_msg_gps)
        return True

    def init_clock_msec(self):
        # it is a new style flash log with full timestamps
        self.clock = DFReaderClock_msec()

    def init_clock_usec(self):
        self.clock = DFReaderClock_usec()

    def init_clock_gps_interpolated(self, clock):
        self.clock = clock

    def init_clock(self):
        '''work out time basis for the log'''

        self._rewind()

        # speculatively create a gps clock in case we don't find anything
        # better
        gps_clock = DFReaderClock_gps_interpolated()
        self.clock = gps_clock

        px4_msg_time = None
        px4_msg_gps = None
        gps_interp_msg_gps1 = None
        first_us_stamp = None
        first_ms_stamp = None

        have_good_clock = False
        while True:
            m = self.recv_msg()
            if m is None:
                break

            type = m.get_type()

            if first_us_stamp is None:
                first_us_stamp = getattr(m, "TimeUS", None)

            if first_ms_stamp is None and (type != 'GPS' and type != 'GPS2'):
                # Older GPS messages use TimeMS for msecs past start
                # of gps week
                first_ms_stamp = getattr(m, "TimeMS", None)

            if type == 'GPS' or type == 'GPS2':
                if getattr(m, "TimeUS", 0) != 0 and \
                   getattr(m, "GWk", 0) != 0:  # everything-usec-timestamped
                    self.init_clock_usec()
                    if not self._zero_time_base:
                        self.clock.find_time_base(m, first_us_stamp)
                    have_good_clock = True
                    break
                if getattr(m, "T", 0) != 0 and \
                   getattr(m, "Week", 0) != 0:  # GPS is msec-timestamped
                    if first_ms_stamp is None:
                        first_ms_stamp = m.T
                    self.init_clock_msec()
                    if not self._zero_time_base:
                        self.clock.find_time_base(m, first_ms_stamp)
                    have_good_clock = True
                    break
                if getattr(m, "GPSTime", 0) != 0:  # px4-style-only
                    px4_msg_gps = m
                if getattr(m, "Week", 0) != 0:
                    if (gps_interp_msg_gps1 is not None and
                        (gps_interp_msg_gps1.TimeMS != m.TimeMS or
                         gps_interp_msg_gps1.Week != m.Week)):
                        # we've received two distinct, non-zero GPS
                        # packets without finding a decent clock to
                        # use; fall back to interpolation. Q: should
                        # we wait a few more messages befoe doing
                        # this?
                        self.init_clock_gps_interpolated(gps_clock)
                        have_good_clock = True
                        break
                    gps_interp_msg_gps1 = m

            elif type == 'TIME':
                '''only px4-style logs use TIME'''
                if getattr(m, "StartTime", None) is not None:
                    px4_msg_time = m

            if px4_msg_time is not None and px4_msg_gps is not None:
                self.init_clock_px4(px4_msg_time, px4_msg_gps)
                have_good_clock = True
                break

#        print("clock is " + str(self.clock))
        if not have_good_clock:
            # we failed to find any GPS messages to set a time
            # base for usec and msec clocks.  Also, not a
            # PX4-style log
            if first_us_stamp is not None:
                self.init_clock_usec()
            elif first_ms_stamp is not None:
                self.init_clock_msec()

        self._rewind()

        return

    def _set_time(self, m):
        '''set time for a message'''
        # really just left here for profiling
        m._timestamp = self.timestamp
        if len(m._fieldnames) > 0 and self.clock is not None:
            self.clock.set_message_timestamp(m)

    def recv_msg(self):
        return self._parse_next()

    def _add_msg(self, m):
        '''add a new message'''
        type = m.get_type()
        self.messages[type] = m
        if m.fmt.instance_field is not None:
            i = m.__getattr__(m.fmt.instance_field)
            self.messages["%s[%s]" % (type, str(i))] = m

        if self.clock:
            self.clock.message_arrived(m)

        if type == 'MSG' and hasattr(m,'Message'):
            if m.Message.find("Rover") != -1:
                self.mav_type = mavutil.mavlink.MAV_TYPE_GROUND_ROVER
            elif m.Message.find("Plane") != -1:
                self.mav_type = mavutil.mavlink.MAV_TYPE_FIXED_WING
            elif m.Message.find("Copter") != -1:
                self.mav_type = mavutil.mavlink.MAV_TYPE_QUADROTOR
            elif m.Message.startswith("Antenna"):
                self.mav_type = mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER
            elif m.Message.find("ArduSub") != -1:
                self.mav_type = mavutil.mavlink.MAV_TYPE_SUBMARINE
            elif m.Message.find("Blimp") != -1:
                self.mav_type = mavutil.mavlink.MAV_TYPE_AIRSHIP
        if type == 'VER' and hasattr(m,'BU'):
            build_types = { 1: mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                            2: mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            3: mavutil.mavlink.MAV_TYPE_FIXED_WING,
                            4: mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER,
                            7: mavutil.mavlink.MAV_TYPE_SUBMARINE,
                            13: mavutil.mavlink.MAV_TYPE_HELICOPTER,
                            12: mavutil.mavlink.MAV_TYPE_AIRSHIP,
                            }
            mavtype = build_types.get(m.BU,None)
            if mavtype is not None:
                self.mav_type = mavtype
        if type == 'MODE':
            if hasattr(m,'Mode') and isinstance(m.Mode, str):
                self.flightmode = m.Mode.upper()
            elif 'ModeNum' in m._fieldnames:
                mapping = mavutil.mode_mapping_bynumber(self.mav_type)
                if mapping is not None and m.ModeNum in mapping:
                    self.flightmode = mapping[m.ModeNum]
                else:
                    self.flightmode = 'UNKNOWN'
            elif hasattr(m,'Mode'):
                self.flightmode = mavutil.mode_string_acm(m.Mode)
        if type == 'STAT' and 'MainState' in m._fieldnames:
            self.flightmode = mavutil.mode_string_px4(m.MainState)
        if type == 'PARM' and getattr(m, 'Name', None) is not None:
            self.params[m.Name] = m.Value
            if hasattr(m,'Default') and not math.isnan(m.Default):
                if not hasattr(self,'param_defaults'):
                    self.param_defaults = {}
                self.param_defaults[m.Name] = m.Default
        self._set_time(m)

    def recv_match(self, condition=None, type=None, blocking=False):
        '''recv the next message that matches the given condition
        type can be a string or a list of strings'''
        if type is not None:
            if isinstance(type, str):
                type = set([type])
            elif isinstance(type, list):
                type = set(type)
        while True:
            if type is not None:
                self.skip_to_type(type)
            m = self.recv_msg()
            if m is None:
                return None
            if type is not None and not m.get_type() in type:
                continue
            if not mavutil.evaluate_condition(condition, self.messages):
                continue
            return m

    def check_condition(self, condition):
        '''check if a condition is true'''
        return mavutil.evaluate_condition(condition, self.messages)

    def param(self, name, default=None):
        '''convenient function for returning an arbitrary MAVLink
           parameter with a default'''
        if name not in self.params:
            return default
        return self.params[name]

    def flightmode_list(self):
        '''return an array of tuples for all flightmodes in log. Tuple is (modestring, t0, t1)'''
        tstamp = None
        fmode = None
        if self._flightmodes is None:
            self._rewind()
            self._flightmodes = []
            types = set(['MODE'])
            while True:
                m = self.recv_match(type=types)
                if m is None:
                    break
                tstamp = m._timestamp
                if self.flightmode == fmode:
                    continue
                if len(self._flightmodes) > 0:
                    (mode, t0, t1) = self._flightmodes[-1]
                    self._flightmodes[-1] = (mode, t0, tstamp)
                self._flightmodes.append((self.flightmode, tstamp, None))
                fmode = self.flightmode
            if tstamp is not None:
                (mode, t0, t1) = self._flightmodes[-1]
                self._flightmodes[-1] = (mode, t0, self.last_timestamp())

        self._rewind()
        return self._flightmodes
    
    def close(self):
        '''close the log file'''
        self.data_map.close()
        self.filehandle.close()
    

class DFReader_binary(DFReader):
    '''parse a binary dataflash file'''
    def __init__(self, filename, zero_time_base=False, progress_callback=None):
        DFReader.__init__(self)
        # read the whole file into memory for simplicity
        self.filehandle = open(filename, 'r')
        self.filehandle.seek(0, 2)
        self.data_len = self.filehandle.tell()
        self.filehandle.seek(0)
        if platform.system() == "Windows":
            self.data_map = mmap.mmap(self.filehandle.fileno(), self.data_len, None, mmap.ACCESS_READ)
        else:
            self.data_map = mmap.mmap(self.filehandle.fileno(), self.data_len, mmap.MAP_PRIVATE, mmap.PROT_READ)

        self.HEAD1 = 0xA3
        self.HEAD2 = 0x95
        self.unpackers = {}
        if sys.version_info.major < 3:
            self.HEAD1 = chr(self.HEAD1)
            self.HEAD2 = chr(self.HEAD2)
        self.formats = {
            0x80: DFFormat(0x80,
                           'FMT',
                           89,
                           'BBnNZ',
                           "Type,Length,Name,Format,Columns")
        }
        self._zero_time_base = zero_time_base
        self.prev_type = None
        self.init_clock()
        self.prev_type = None
        self._rewind()
        self.init_arrays(progress_callback)

    def _rewind(self):
        '''rewind to start of log'''
        DFReader._rewind(self)
        self.offset = 0
        self.remaining = self.data_len
        self.type_nums = None
        self.timestamp = 0

    def rewind(self):
        '''rewind to start of log'''
        self._rewind()

    def init_arrays(self, progress_callback=None):
        '''initialise arrays for fast recv_match()'''
        self.offsets = []
        self.counts = []
        self._count = 0
        self.name_to_id = {}
        self.id_to_name = {}
        type_instances = {}
        for i in range(256):
            self.offsets.append([])
            self.counts.append(0)
        fmt_type = 0x80
        fmtu_type = None
        unit_type = None
        mult_type = None
        ofs = 0
        pct = 0
        HEAD1 = self.HEAD1
        HEAD2 = self.HEAD2
        lengths = [-1] * 256

        while ofs+3 < self.data_len:
            hdr = self.data_map[ofs:ofs+3]
            if hdr[0] != HEAD1 or hdr[1] != HEAD2:
                # avoid end of file garbage, 528 bytes has been use consistently throughout this implementation
                # but it needs to be at least 249 bytes which is the block based logging page size (256) less a 6 byte header and
                # one byte of data. Block based logs are sized in pages which means they can have up to 249 bytes of trailing space.
                if self.data_len - ofs >= 528 or self.data_len < 528:
                    print("bad header 0x%02x 0x%02x at %d" % (u_ord(hdr[0]), u_ord(hdr[1]), ofs), file=sys.stderr)
                ofs += 1
                continue
            mtype = u_ord(hdr[2])
            self.offsets[mtype].append(ofs)

            if lengths[mtype] == -1:
                if not mtype in self.formats:
                    if self.data_len - ofs >= 528 or self.data_len < 528:
                        print("unknown msg type 0x%02x (%u) at %d" % (mtype, mtype, ofs),
                              file=sys.stderr)
                    break
                self.offset = ofs
                self._parse_next()
                fmt = self.formats[mtype]
                lengths[mtype] = fmt.len
            elif self.formats[mtype].instance_field is not None:
                fmt = self.formats[mtype]
                # see if we've has this instance value before
                idata = self.data_map[ofs+3+fmt.instance_ofs:ofs+3+fmt.instance_ofs+fmt.instance_len]
                if not mtype in type_instances:
                    type_instances[mtype] = set()
                if not idata in type_instances[mtype]:
                    # its a new one, need to parse it so we have the complete set of instances
                    type_instances[mtype].add(idata)
                    self.offset = ofs
                    self._parse_next()

            self.counts[mtype] += 1
            mlen = lengths[mtype]

            if mtype == fmt_type:
                body = self.data_map[ofs+3:ofs+mlen]
                if len(body)+3 < mlen:
                    break
                fmt = self.formats[mtype]
                elements = list(struct.unpack(fmt.msg_struct, body))
                ftype = elements[0]
                mfmt = DFFormat(
                    ftype,
                    null_term(elements[2]), elements[1],
                    null_term(elements[3]), null_term(elements[4]),
                    oldfmt=self.formats.get(ftype,None))
                self.formats[ftype] = mfmt
                self.name_to_id[mfmt.name] = mfmt.type
                self.id_to_name[mfmt.type] = mfmt.name
                if mfmt.name == 'FMTU':
                    fmtu_type = mfmt.type
                if mfmt.name == 'UNIT':
                    unit_type = mfmt.type
                if mfmt.name == 'MULT':
                    mult_type = mfmt.type

            # Handle FMTU messages by updating the DFFormat class with the
            # unit/multiplier information
            if fmtu_type is not None and mtype == fmtu_type:
                fmt = self.formats[mtype]
                body = self.data_map[ofs+3:ofs+mlen]
                if len(body)+3 < mlen:
                    break
                elements = list(struct.unpack(fmt.msg_struct, body))
                ftype = int(elements[1])
                if ftype in self.formats:
                    fmt2 = self.formats[ftype]
                    if 'UnitIds' in fmt.colhash:
                        fmt2.set_unit_ids(null_term(elements[fmt.colhash['UnitIds']]), self.unit_lookup)
                    if 'MultIds' in fmt.colhash:
                        fmt2.set_mult_ids(null_term(elements[fmt.colhash['MultIds']]), self.mult_lookup)

            # Handle UNIT messages by updating the unit_lookup dictionary
            if unit_type is not None and mtype == unit_type:
                fmt = self.formats[mtype]
                body = self.data_map[ofs+3:ofs+mlen]
                if len(body)+3 < mlen:
                    break
                elements = list(struct.unpack(fmt.msg_struct, body))
                self.unit_lookup[chr(elements[1])] = null_term(elements[2])

            # Handle MULT messages by updating the mult_lookup dictionary
            if mult_type is not None and mtype == mult_type:
                fmt = self.formats[mtype]
                body = self.data_map[ofs+3:ofs+mlen]
                if len(body)+3 < mlen:
                    break
                elements = list(struct.unpack(fmt.msg_struct, body))
                # Even though the multiplier value is logged as a double, the
                # values in log files look to be single-precision values that have
                # been cast to a double.
                # To ensure that the values saved here can be used to index the
                # MULT_TO_PREFIX table, we round them to 7 significant decimal digits
                mult = float("%.7g" % (elements[2]))
                self.mult_lookup[chr(elements[1])] = mult

            ofs += mlen
            if progress_callback is not None:
                new_pct = (100 * ofs) // self.data_len
                if new_pct != pct:
                    progress_callback(new_pct)
                    pct = new_pct

        for i in range(256):
            self._count += self.counts[i]
        self.offset = 0

    def last_timestamp(self):
        '''get the last timestamp in the log'''
        highest_offset = 0
        second_highest_offset = 0
        for i in range(256):
            if self.counts[i] == -1:
                continue
            if len(self.offsets[i]) == 0:
                continue
            ofs = self.offsets[i][-1]
            if ofs > highest_offset:
                second_highest_offset = highest_offset
                highest_offset = ofs
            elif ofs > second_highest_offset:
                second_highest_offset = ofs
        self.offset = highest_offset
        m = self.recv_msg()
        if m is None:
            self.offset = second_highest_offset
            m = self.recv_msg()
        return m._timestamp


    def skip_to_type(self, type):
        '''skip fwd to next msg matching given type set'''

        if self.type_nums is None:
            # always add some key msg types so we can track flightmode, params etc
            type = type.copy()
            type.update(set(['MODE','MSG','PARM','STAT','ORGN','VER']))
            self.indexes = []
            self.type_nums = []
            for t in type:
                if not t in self.name_to_id:
                    continue
                self.type_nums.append(self.name_to_id[t])
                self.indexes.append(0)
        smallest_index = -1
        smallest_offset = self.data_len
        for i in range(len(self.type_nums)):
            mtype = self.type_nums[i]
            if self.indexes[i] >= self.counts[mtype]:
                continue
            ofs = self.offsets[mtype][self.indexes[i]]
            if ofs < smallest_offset:
                smallest_offset = ofs
                smallest_index = i
        if smallest_index >= 0:
            self.indexes[smallest_index] += 1
            self.offset = smallest_offset

    def _parse_next(self):
        '''read one message, returning it as an object'''

        # skip over bad messages; after this loop has run msg_type
        # indicates the message which starts at self.offset (including
        # signature bytes and msg_type itself)
        skip_type = None
        skip_start = 0
        while True:
            if self.data_len - self.offset < 3:
                return None

            hdr = self.data_map[self.offset:self.offset+3]
            if hdr[0] == self.HEAD1 and hdr[1] == self.HEAD2:
                # signature found
                if skip_type is not None:
                    # emit message about skipped bytes
                    if self.remaining >= 528:
                        # APM logs often contain garbage at end
                        skip_bytes = self.offset - skip_start
                        print("Skipped %u bad bytes in log at offset %u, type=%s (prev=%s)" %
                              (skip_bytes, skip_start, skip_type, self.prev_type),
                          file=sys.stderr)
                    skip_type = None
                # check we recognise this message type:
                msg_type = u_ord(hdr[2])
                if msg_type in self.formats:
                    # recognised message found
                    self.prev_type = msg_type
                    break;
                # message was not recognised; fall through so these
                # bytes are considered "skipped".  The signature bytes
                # are easily recognisable in the "Skipped bytes"
                # message.
            if skip_type is None:
                skip_type = (u_ord(hdr[0]), u_ord(hdr[1]), u_ord(hdr[2]))
                skip_start = self.offset
            self.offset += 1
            self.remaining -= 1

        self.offset += 3
        self.remaining = self.data_len - self.offset

        fmt = self.formats[msg_type]
        if self.remaining < fmt.len-3:
            # out of data - can often happen half way through a message
            if self.verbose:
                print("out of data", file=sys.stderr)
            return None
        body = self.data_map[self.offset:self.offset+fmt.len-3]
        elements = None
        try:
            if not msg_type in self.unpackers:
                self.unpackers[msg_type] = struct.Struct(fmt.msg_struct).unpack
            elements = list(self.unpackers[msg_type](body))
        except Exception as ex:
            print(ex)
            if self.remaining < 528:
                # we can have garbage at the end of an APM2 log
                return None
            # we should also cope with other corruption; logs
            # transferred via DataFlash_MAVLink may have blocks of 0s
            # in them, for example
            print("Failed to parse %s/%s with len %u (remaining %u)" %
                  (fmt.name, fmt.msg_struct, len(body), self.remaining),
                  file=sys.stderr)
        if elements is None:
            return self._parse_next()
        name = fmt.name
        # transform elements which can't be done at unpack time:
        for a_index in fmt.a_indexes:
            try:
                elements[a_index] = array.array('h', elements[a_index])
            except Exception as e:
                print("Failed to transform array: %s" % str(e),
                      file=sys.stderr)

        if name == 'FMT':
            # add to formats
            # name, len, format, headings
            try:
                ftype = elements[0]
                mfmt = DFFormat(
                    ftype,
                    null_term(elements[2]), elements[1],
                    null_term(elements[3]), null_term(elements[4]),
                    oldfmt=self.formats.get(ftype,None))
                self.formats[ftype] = mfmt
            except Exception:
                return self._parse_next()

        self.offset += fmt.len - 3
        self.remaining = self.data_len - self.offset
        m = DFMessage(fmt, elements, True, self)

        if m.fmt.name == 'FMTU':
            # add to units information
            FmtType = int(elements[0])
            UnitIds = elements[1]
            MultIds = elements[2]
            if FmtType in self.formats:
                fmt = self.formats[FmtType]
                fmt.set_unit_ids(UnitIds, self.unit_lookup)
                fmt.set_mult_ids(MultIds, self.mult_lookup)

        try:
            self._add_msg(m)
        except Exception as ex:
            print("bad msg at offset %u" % self.offset, ex)
            pass
        self.percent = 100.0 * (self.offset / float(self.data_len))

        return m

    def find_unused_format(self):
        '''find an unused format code'''
        for i in range(254, 1, -1):
            if not i in self.formats:
                return i
        return None

    def add_format(self, fmt):
        '''add a new format'''
        new_type = self.find_unused_format()
        if new_type is None:
            return None
        fmt.type = new_type
        self.formats[new_type] = fmt
        return fmt

    def make_msgbuf(self, fmt, values):
        '''make a message buffer from a list of values'''
        ret = struct.pack("BBB", 0xA3, 0x95, fmt.type)
        ret += struct.pack(fmt.msg_struct, *values)
        return ret

    def make_format_msgbuf(self, fmt):
        '''make a message buffer for a FMT message'''
        fmt_fmt = self.formats[0x80]
        ret = struct.pack("BBB", 0xA3, 0x95, 0x80)
        ret += struct.pack(fmt_fmt.msg_struct, *[fmt.type,struct.calcsize(fmt.msg_struct)+3,
                                                 fmt.name.encode('ascii'),
                                                 fmt.format.encode('ascii'),
                                                 ','.join(fmt.columns).encode('ascii')])
        return ret
    

def DFReader_is_text_log(filename):
    '''return True if a file appears to be a valid text log'''
    with open(filename, 'r') as f:
        ret = (f.read(8000).find('FMT,') != -1)

    return ret


class DFReader_text(DFReader):
    '''parse a text dataflash file'''
    def __init__(self, filename, zero_time_base=False, progress_callback=None):
        DFReader.__init__(self)
        self.name_to_id = {}
        # read the whole file into memory for simplicity
        self.filehandle = open(filename, 'r')
        self.filehandle.seek(0, 2)
        self.data_len = self.filehandle.tell()
        self.filehandle.seek(0, 0)
        if platform.system() == "Windows":
            self.data_map = mmap.mmap(self.filehandle.fileno(), self.data_len, None, mmap.ACCESS_READ)
        else:
            self.data_map = mmap.mmap(self.filehandle.fileno(), self.data_len, mmap.MAP_PRIVATE, mmap.PROT_READ)
        self.offset = 0
        self.delimiter = ", "

        self.formats = {
            'FMT': DFFormat(0x80,
                            'FMT',
                            89,
                            'BBnNZ',
                            "Type,Length,Name,Format,Columns")
        }
        self.id_to_name = { 0x80 : 'FMT' }
        self._rewind()
        self._zero_time_base = zero_time_base
        self.init_clock()
        self._rewind()
        self.init_arrays(progress_callback)

    def _rewind(self):
        '''rewind to start of log'''
        DFReader._rewind(self)
        # find the first valid line
        self.offset = self.data_map.find(b'FMT, ')
        if self.offset == -1:
            self.offset = self.data_map.find(b'FMT,')
            if self.offset != -1:
                self.delimiter = ","
        self.type_list = None

    def rewind(self):
        '''rewind to start of log'''
        self._rewind()

    def init_arrays(self, progress_callback=None):
        '''initialise arrays for fast recv_match()'''
        self.offsets = {}
        self.counts = {}
        self._count = 0
        ofs = self.offset
        pct = 0

        while ofs+16 < self.data_len:
            mtype = self.data_map[ofs:ofs+4]
            # convert to string and cut if there is a ','
            mtype = mtype.decode().split(',')[0]
            if not mtype in self.offsets:
                self.counts[mtype] = 0
                self.offsets[mtype] = []
                self.offset = ofs
                self._parse_next()
            self.offsets[mtype].append(ofs)

            self.counts[mtype] += 1

            if mtype == "FMT":
                self.offset = ofs
                self._parse_next()

            if mtype == "FMTU":
                self.offset = ofs
                self._parse_next()

            ofs = self.data_map.find(b"\n", ofs)
            if ofs == -1:
                break
            ofs += 1
            new_pct = (100 * ofs) // self.data_len
            if progress_callback is not None and new_pct != pct:
                progress_callback(new_pct)
                pct = new_pct

        for mtype in self.counts.keys():
            self._count += self.counts[mtype]
        self.offset = 0

    def skip_to_type(self, type):
        '''skip fwd to next msg matching given type set'''

        if self.type_list is None:
            # always add some key msg types so we can track flightmode, params etc
            self.type_list = type.copy()
            self.type_list.update(set(['MODE','MSG','PARM','STAT','ORGN','VER']))
            self.type_list = list(self.type_list)
            self.indexes = []
            self.type_nums = []
            for t in self.type_list:
                self.indexes.append(0)
        smallest_index = -1
        smallest_offset = self.data_len
        for i in range(len(self.type_list)):
            mtype = self.type_list[i]
            if not mtype in self.counts:
                continue
            if self.indexes[i] >= self.counts[mtype]:
                continue
            ofs = self.offsets[mtype][self.indexes[i]]
            if ofs < smallest_offset:
                smallest_offset = ofs
                smallest_index = i
        if smallest_index >= 0:
            self.indexes[smallest_index] += 1
            self.offset = smallest_offset

    def _parse_next(self):
        '''read one message, returning it as an object'''

        while True:
            endline = self.data_map.find(b'\n',self.offset)
            if endline == -1:
                endline = self.data_len
                if endline < self.offset:
                    break
            s = self.data_map[self.offset:endline].rstrip()
            if sys.version_info.major >= 3:
                s = s.decode('utf-8')
            elements = s.split(self.delimiter)
            self.offset = endline+1
            if len(elements) >= 2:
                # this_line is good
                break

        if self.offset > self.data_len:
            return None

        # cope with empty structures
        if len(elements) == 5 and elements[-1] == ',':
            elements[-1] = ''
            elements.append('')

        self.percent = 100.0 * (self.offset / float(self.data_len))

        msg_type = elements[0]

        if msg_type not in self.formats:
            return self._parse_next()

        fmt = self.formats[msg_type]

        if len(elements) < len(fmt.format)+1:
            # not enough columns
            return self._parse_next()

        elements = elements[1:]

        name = fmt.name.rstrip('\0')
        if name == 'FMT':
            # add to formats
            # name, len, format, headings
            ftype = int(elements[0])
            fname = elements[2]
            if self.delimiter == ",":
                elements = elements[0:4] + [",".join(elements[4:])]
            columns = elements[4]
            if fname == 'FMT' and columns == 'Type,Length,Name,Format':
                # some logs have the 'Columns' column missing from text logs
                columns = "Type,Length,Name,Format,Columns"
            new_fmt = DFFormat(ftype,
                               fname,
                               int(elements[1]),
                               elements[3],
                               columns,
                               oldfmt=self.formats.get(ftype,None))
            self.formats[fname] = new_fmt
            self.id_to_name[ftype] = fname
            self.name_to_id[fname] = ftype

        try:
            m = DFMessage(fmt, elements, False, self)
        except ValueError:
            return self._parse_next()

        if m.get_type() == 'FMTU':
            fmtid = getattr(m, 'FmtType', None)
            if fmtid is not None and fmtid in self.id_to_name:
                fmtu = self.formats[self.id_to_name[fmtid]]
                fmtu.set_unit_ids(getattr(m, 'UnitIds', None), self.unit_lookup)
                fmtu.set_mult_ids(getattr(m, 'MultIds', None), self.mult_lookup)

        if m.get_type() == 'UNIT':
            unitid = getattr(m, 'Id', None)
            label = getattr(m, 'Label', None)
            self.unit_lookup[chr(unitid)] = null_term(label)

        if m.get_type() == 'MULT':
            multid = getattr(m, 'Id', None)
            mult = getattr(m, 'Mult', None)
            # Even though the multiplier value is logged as a double, the
            # values in log files look to be single-precision values that have
            # been cast to a double.
            # To ensure that the values saved here can be used to index the
            # MULT_TO_PREFIX table, we round them to 7 significant decimal digits
            mult = float("%.7g" % (mult))
            self.mult_lookup[chr(multid)] = mult

        self._add_msg(m)

        return m

    def last_timestamp(self):
        '''get the last timestamp in the log'''
        highest_offset = 0
        for mtype in self.counts.keys():
            if len(self.offsets[mtype]) == 0:
                continue
            ofs = self.offsets[mtype][-1]
            if ofs > highest_offset:
                highest_offset = ofs
        self.offset = highest_offset
        m = self.recv_msg()
        return m._timestamp

if __name__ == "__main__":
    use_profiler = False
    if use_profiler:
        from line_profiler import LineProfiler
        profiler = LineProfiler()
        profiler.add_function(DFReader_binary._parse_next)
        profiler.add_function(DFReader_binary._add_msg)
        profiler.add_function(DFReader._set_time)
        profiler.enable_by_count()

    filename = sys.argv[1]
    if filename.endswith('.log'):
        log = DFReader_text(filename)
    else:
        log = DFReader_binary(filename)
    #bfile = filename + ".bin"
    #bout = open(bfile, 'wb')
    while True:
        m = log.recv_msg()
        if m is None:
            break
        #bout.write(m.get_msgbuf())
        #print(m)
    if use_profiler:
        profiler.print_stats()
