'''
module for loading/saving sets of mavlink parameters
'''
import fnmatch, math, time, struct
from pymavlink import mavutil

class MAVParmDict(dict):
    def __init__(self, *args):
        dict.__init__(self, args)
        # some parameters should not be loaded from files
        self.exclude_load = [
            'ARSPD_OFFSET',
            'CMD_INDEX',
            'CMD_TOTAL',
            'FENCE_TOTAL',
            'FORMAT_VERSION',
            'GND_ABS_PRESS',
            'GND_TEMP',
            'LOG_LASTFILE',
            'MIS_TOTAL',
            'SYSID_SW_MREV',
            'SYS_NUM_RESETS',
        ]
        self.mindelta = 0.000001
        # Logical storage types, independent of the selected wire encoding.
        self.param_types = {}
        self.target_supports_bytewise = False

    def mavset(self, mav, name, value, retries=3, parm_type=None, extended_type=None):
        """Set a parameter and verify exact acknowledgements for bytewise values."""
        raw_value = None
        extended_data = None
        integer_value = None
        link_is_mavlink2 = getattr(mav, 'mavlink20', lambda: True)()
        logical_type = self.param_types.get(str(name).upper())
        if parm_type is None and self.target_supports_bytewise and link_is_mavlink2:
            if logical_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                parm_type = 12  # MAV_PARAM_TYPE_BYTEWISE_INT32
            elif logical_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
                parm_type = 13  # MAV_PARAM_TYPE_BYTEWISE_UINT32
            elif logical_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT64, mavutil.mavlink.MAV_PARAM_TYPE_UINT64):
                parm_type = 11  # MAV_PARAM_TYPE_EXTENDED
                extended_type = 1 if logical_type == mavutil.mavlink.MAV_PARAM_TYPE_INT64 else 2

        if parm_type in (11, 12, 13):
            try:
                integer_value = mavutil.param_integer_value(value)
                if parm_type == 11:
                    if extended_type not in (1, 2) or not link_is_mavlink2:
                        raise ValueError('64-bit parameter requires an extended type and MAVLink2')
                    extended_data = struct.pack('<q' if extended_type == 1 else '<Q', integer_value)
                    numeric_value = float('nan')
                else:
                    raw_value = struct.pack('<i' if parm_type == 12 else '<I', integer_value)
                    numeric_value = 0.0
            except (TypeError, ValueError, OverflowError, ArithmeticError, struct.error) as e:
                print("can't send %s: %s" % (name, e))
                return False
        elif parm_type is not None and parm_type != mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            # Legacy bytewise encoding, used by PX4 and older implementations.
            formats = {
                mavutil.mavlink.MAV_PARAM_TYPE_UINT8: '>xxxB',
                mavutil.mavlink.MAV_PARAM_TYPE_INT8: '>xxxb',
                mavutil.mavlink.MAV_PARAM_TYPE_UINT16: '>xxH',
                mavutil.mavlink.MAV_PARAM_TYPE_INT16: '>xxh',
                mavutil.mavlink.MAV_PARAM_TYPE_UINT32: '>I',
                mavutil.mavlink.MAV_PARAM_TYPE_INT32: '>i',
            }
            if parm_type not in formats:
                print("can't send %s of type %u" % (name, parm_type))
                return False
            numeric_value, = struct.unpack('>f', struct.pack(formats[parm_type], int(value)))
        else:
            if isinstance(value, str) and value.lower().startswith('0x'):
                numeric_value = int(value[2:], 16)
            else:
                numeric_value = float(value)

        if integer_value is not None:
            # Negotiate even when the parameter dictionary came from a file/FTP.
            mav.param_fetch_one(name.upper())
        while retries > 0:
            retries -= 1
            kwargs = {'parm_type': parm_type}
            if raw_value is not None:
                kwargs['parm_raw'] = raw_value
            if extended_data is not None:
                kwargs.update(extended_type=extended_type, extended_data=extended_data)
            mav.param_set_send(name.upper(), numeric_value, **kwargs)
            tstart = time.time()
            while time.time() - tstart < 1:
                ack = mav.recv_match(type='PARAM_VALUE', blocking=False)
                if ack is None:
                    time.sleep(0.1)
                    continue
                if str(name).upper() != str(ack.param_id).upper():
                    continue
                if integer_value is not None:
                    decoded = mavutil.decode_param_value(ack)
                    if decoded != integer_value:
                        print("%s: ack value %s does not match %d" % (name, decoded, integer_value))
                        continue
                    self[name] = decoded
                else:
                    self[name] = numeric_value
                return True
        print("timeout setting %s to %s" % (name, value))
        return False


    def save(self, filename, wildcard='*', verbose=False):
        '''save parameters to a file'''
        f = open(filename, mode='w')
        k = list(self.keys())
        k.sort()
        count = 0
        for p in k:
            if p and fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                value = self.__getitem__(p)
                if isinstance(value, float):
                    f.write("%-16.16s %f\n" % (p, value))
                else:
                    f.write("%-16.16s %s\n" % (p, str(value)))
                count += 1
        f.close()
        if verbose:
            print("Saved %u parameters to %s" % (count, filename))


    def load(self, filename, wildcard='*', mav=None, check=True, use_excludes=True):
        '''load parameters from a file'''
        try:
            f = open(filename, mode='r')
        except Exception as e:
            print("Failed to open file '%s': %s" % (filename, str(e)))
            return False
        count = 0
        changed = 0
        for line in f:
            # strip comments, which may be at the end of a line
            line = line.split('#')[0].strip()
            if not line:
                continue
            line = line.replace(',',' ')
            a = line.split()
            if len(a) != 2:
                print("Invalid line: %s" % line)
                continue
            # some parameters should not be loaded from files
            if use_excludes and a[0] in self.exclude_load:
                continue
            if not fnmatch.fnmatch(a[0].upper(), wildcard.upper()):
                continue
            value = a[1].strip()
            if isinstance(value, str) and value.lower().startswith('0x'):
                numeric_value = int(value[2:], 16)
            else:
                try:
                    numeric_value = mavutil.param_integer_value(value)
                except (ValueError, OverflowError, ArithmeticError):
                    numeric_value = float(value)

            if mav is not None:
                if check:
                    if a[0] not in list(self.keys()):
                        print("Unknown parameter %s" % a[0])
                        continue
                    old_value = self.__getitem__(a[0])
                    if math.fabs(old_value - numeric_value) <= self.mindelta:
                        count += 1
                        continue
                    if self.mavset(mav, a[0], value):
                        print("changed %s from %s to %s" % (a[0], old_value, numeric_value))
                else:
                    print("set %s to %s" % (a[0], numeric_value))
                    self.mavset(mav, a[0], value)
                changed += 1
            else:
                self.__setitem__(a[0], numeric_value)
            count += 1
        f.close()
        if mav is not None:
            print("Loaded %u parameters from %s (changed %u)" % (count, filename, changed))
        else:
            print("Loaded %u parameters from %s" % (count, filename))
        return True

    def show_param_value(self, name, value):
        print("%-16.16s %s" % (name, value))

    def show(self, wildcard='*'):
        '''show parameters'''
        k = sorted(self.keys())
        for p in k:
            if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                self.show_param_value(str(p), str(self.get(p)))

    def diff(self, filename, wildcard='*', use_excludes=True, use_tabs=False, show_only1=True, show_only2=True,
             header=False, value_info=None):
        '''show differences with another parameter file. The first value column
           comes from filename, the second from this parameter set.
           value_info, if given, is a callable(name, value) returning a string
           describing the meaning of the value, which is appended as a comment'''
        other = MAVParmDict()
        if not other.load(filename, use_excludes=use_excludes):
            return

        def info(k, label, value):
            '''return a comment fragment for the meaning of a value'''
            if value_info is None:
                return None
            s = value_info(k, value)
            if s is None:
                return None
            return "%s=%s" % (label, s)

        def comment(*parts):
            '''join value meanings into a trailing comment'''
            parts = [p for p in parts if p is not None]
            if not parts:
                return ""
            return " # %s" % " ".join(parts)

        def display(value):
            return str(value) if isinstance(value, int) else "%.4f" % value

        if header:
            if use_tabs:
                print("%s\t%s\t%s" % ("PARAMETER", "FILE1", "FILE2"))
            else:
                print("%-16.16s %12s %12s" % ("PARAMETER", "FILE1", "FILE2"))
        keys = sorted(list(set(self.keys()).union(set(other.keys()))))
        for k in keys:
            if not fnmatch.fnmatch(str(k).upper(), wildcard.upper()):
                continue
            if not k in other:
                value = self[k]
                if show_only2:
                    print("%-16.16s              %12s%s" % (k, display(value), comment(info(k, "FILE2", value))))
            elif not k in self:
                if show_only1:
                    value = other[k]
                    print("%-16.16s %12s%s" % (k, display(value), comment(info(k, "FILE1", value))))
            elif abs(self[k] - other[k]) > self.mindelta:
                value = self[k]
                c = comment(info(k, "FILE1", other[k]), info(k, "FILE2", value))
                if use_tabs:
                    print("%s\t%s\t%s%s" % (k, display(other[k]), display(value), c))
                else:
                    print("%-16.16s %12s %12s%s" % (k, display(other[k]), display(value), c))
