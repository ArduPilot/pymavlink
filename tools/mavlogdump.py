#!/usr/bin/env python3

'''
example program that dumps a Mavlink log file. The log file is
assumed to be in the format that qgroundcontrol uses, which consists
of a series of MAVLink packets, each with a 64 bit timestamp
header. The timestamp is in microseconds since 1970 (unix epoch)
'''
import array
import fnmatch
import json
import os
import struct
import sys
import time
import inspect
import math
from argparse import ArgumentParser
from pymavlink.DFReader import to_string
from pymavlink import mavutil

def reduce_msg(mtype, reduction_ratio, reduction_yes, reduction_no, reduction_count):
    '''return True if this msg should be discarded by reduction'''
    if mtype in reduction_no:
        return False
    if not mtype in reduction_yes:
        reduction_msgs = ['NKF*', 'XKF*', 'IMU*', 'AHR2', 'BAR*', 'ATT', 'BAT*', 'CTUN', 'NTUN', 'GP*', 'IMT*', 'MAG*', 'PL', 'POS', 'POW*', 'RATE', 'RC*', 'RFND', 'UBX*', 'VIBE', 'NKQ*', 'MOT*', 'CTRL', 'FTS*', 'DSF', 'CST*', 'LOS*', 'UWB*']
        for m in reduction_msgs:
            if fnmatch.fnmatch(mtype, m):
                reduction_yes.add(mtype)
                reduction_count[mtype] = 0
                break
        if not mtype in reduction_yes:
            reduction_no.add(mtype)
            return False
    reduction_count[mtype] += 1
    if reduction_count[mtype] == reduction_ratio:
        reduction_count[mtype] = 0
        return False
    return True

def reduce_rate_msg(m, reduction_rate, last_msg_rate_t):
    '''return True if this msg should be discarded by reduction'''
    mtype = m.get_type()
    if mtype in ['PARM','MSG','FMT','FMTU','MULT','MODE','EVT','UNIT', 'VER']:
        return False
    t = getattr(m,'_timestamp',None)
    if t is None:
        return False
    if not mtype in last_msg_rate_t:
        last_msg_rate_t[mtype] = t
    dt = t - last_msg_rate_t[mtype]

    if dt < 0 or dt >= 1.0/reduction_rate:
        last_msg_rate_t[mtype] = t
        return False
    return True

def match_type(mtype, patterns):
    '''return True if mtype matches pattern'''
    for p in patterns:
        if fnmatch.fnmatch(mtype, p):
            return True
    return False

def handle_json_output(m, m_type, timestamp, show_source, output_fh, count, newline_delim=False):
    '''Handle JSON format output'''
    # Format our message as a Python dict, which gets us almost to proper JSON format
    data = m.to_dict()

    # Remove the mavpackettype value as we specify that later.
    del data['mavpackettype']

    # Also, if it's a BAD_DATA message, make it JSON-compatible by removing array objects
    if 'data' in data and type(data['data']) is not dict:
        data['data'] = list(data['data'])

    # Prepare the message as a single object with 'meta' and 'data' keys holding
    # the message's metadata and actual data respectively.
    meta = {"type": m_type, "timestamp": timestamp}
    if show_source:
        meta["srcSystem"] = m.get_srcSystem()
        meta["srcComponent"] = m.get_srcComponent()

    # convert any array.array (e.g. packed-16-bit fft readings) into lists:
    for key in data.keys():
        if type(data[key]) == float:
            if math.isnan(data[key]):
                data[key] = None
        elif type(data[key]) == array.array:
            data[key] = list(data[key])
        # convert any byte-strings into utf-8 strings.  Don't die trying.
        elif type(data[key]) == bytes:
            str_repr = to_string(data[key])
            data[key] = str_repr
    outMsg = {"meta": meta, "data": data}

    # TODO: add file write
    # Now write this object stringified properly.
    if newline_delim:
        output_fh.write(f'{json.dumps(outMsg)}\n')
    else:
        output_fh.write(f'{"," if count>0 else ""}\n\t{json.dumps(outMsg)}')


def handle_csv_output(m, m_type, timestamp, csv_sep, fields, isbin, islog, output_fh):
    '''Handle CSV format output'''
    data = m.to_dict()
    if isbin or islog:
        csv_out = [str(data[y]) if y != "timestamp" else "" for y in fields]
    else:
        csv_out = [str(data[y.split('.')[-1]]) if y.split('.')[0] == m_type and y.split('.')[-1] in data else "" for y in fields]
    csv_out[0] = "{:.8f}".format(timestamp)
    output_fh.write(f'{csv_sep.join(csv_out)}\n')

def handle_mat_output(m, m_type, out_dict):
    '''Handle MAT format output'''
    # If this packet contains data (i.e. is not a FMT
    # packet), append the data in this packet to the
    # corresponding list
    if m_type == 'FMT':
        return

    # If this packet type has not yet been
    # seen, add a new entry to the big dict
    if m_type not in out_dict:
        out_dict[m_type] = {}

    md = m.to_dict()
    del md['mavpackettype']
    cols = md.keys()
    for col in cols:
        # If this column hasn't had data entered,
        # make a new key and list
        if col in out_dict[m_type]:
            out_dict[m_type][col].append(md[col])
        else:
            out_dict[m_type][col] = [md[col]]

def handle_standard_output(m, timestamp, show_source, show_seq, output_fh, isbin, islog, m_type, parms):
    '''Handle standard format output'''

    # Otherwise we output in a standard Python dict-style format
    if output_fh is not sys.stdout:
        if (isbin or islog) and (m_type in ["FMT", "FMTU", "MULT", "UNIT"] or (m_type == "PARM" and parms)):
            output_fh.write(m.get_msgbuf())
        elif m_type == 'PARAM_VALUE' and parms:
            timestamp = getattr(m, '_timestamp', None)
            output_fh.write(struct.pack('>Q', int(timestamp*1.0e6)) + m.get_msgbuf())
    else:
        s = "%s.%02u: %s" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                    time.localtime(timestamp)),
                            int(timestamp*100.0)%100, m)
        if show_source:
            s += " srcSystem=%u srcComponent=%u" % (m.get_srcSystem(), m.get_srcComponent())
        if show_seq:
            s += " seq=%u" % m.get_seq()
        output_fh.write(s)

def handle_pretty_output(m, istlog, output_fh):
    if istlog:
        mavutil.dump_message_verbose(output_fh, m)
        output_fh.write('')
    elif hasattr(m,"dump_verbose"):
        m.dump_verbose(output_fh)
        output_fh.write('')

def parse_args():
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--no-timestamps", action='store_true', help="Log doesn't have timestamps")
    parser.add_argument("--planner", action='store_true', help="use planner file format")
    parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
    parser.add_argument("-f", "--follow", action='store_true', help="keep waiting for more data at end of file (not implemented for .bin, .log, .csv")
    parser.add_argument("--condition", default=None, help="select packets by condition")
    parser.add_argument("-o", "--output_path", default=None, help="Output file path; if left undefined, writes to stdout.")
    parser.add_argument("-p", "--parms", action='store_true', help="preserve parameters in output with -o")
    parser.add_argument("--format", default='standard', help="Change the output format between 'standard', 'json', 'ndjson', 'csv', 'mat', 'types-only', and 'pretty'. For the CSV output, you must supply types that you want. For MAT output, specify output file with -o")
    parser.add_argument("--csv_sep", dest="csv_sep", default=",", help="Select the delimiter between columns for the output CSV file. Use 'tab' to specify tabs. Only applies when --format=csv")
    parser.add_argument("--types", default=None, help="types of messages (comma separated with wildcard)")
    parser.add_argument("--nottypes", default=None, help="types of messages not to include (comma separated with wildcard)")
    parser.add_argument("-c", "--compress", action='store_true', help="Compress .mat file data")
    parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
    parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
    parser.add_argument("--no-bad-data", action='store_true', help="Don't output corrupted messages")
    parser.add_argument("--show-source", action='store_true', help="Show source system ID and component ID")
    parser.add_argument("--show-seq", action='store_true', help="Show sequence numbers")
    parser.add_argument("--show-loss", action='store_true', help="Shows changes in lost messages")
    parser.add_argument("--source-system", type=int, default=None, help="filter by source system ID")
    parser.add_argument("--source-component", type=int, default=None, help="filter by source component ID")
    parser.add_argument("--link", type=int, default=None, help="filter by comms link ID")
    parser.add_argument("--mav10", action='store_true', help="parse as MAVLink1")
    parser.add_argument("--reduce", type=int, default=0, help="reduce streaming messages")
    parser.add_argument("--reduce-rate", type=float, default=0, help="reduce messages to maximum rate in Hz")
    parser.add_argument("--profile", action='store_true', help="run the Yappi python profiler")
    parser.add_argument("--meta", action='store_true', help="output meta-data msgs even if not matching condition")
    parser.add_argument("log", metavar="LOG")

    return parser.parse_args()

def dump_log(
    no_timestamps: bool = False,
    planner: bool = False,
    robust: bool = False,
    follow: bool = False,
    condition: str = None,
    output_path: str = None,
    parms: bool = False,
    format: str = None,
    csv_sep: str = ",",
    types: str = None,
    nottypes: str = None,
    compress: bool = False,
    dialect: str = "ardupilotmega",
    zero_time_base: bool = False,
    no_bad_data: bool = False,
    show_source: bool = False,
    show_seq: bool = False,
    show_loss: bool = False,
    source_system: int = None,
    source_component: int = None,
    link: int = None,
    mav10: bool = False,
    reduce: int = 0,
    reduce_rate: float = 0,
    log: str = None,
    profile: bool = False,
    meta: bool = False,
):
    
    # set up output file handler based on format and output_path
    with open(output_path, mode='wb' if format =='standard' else 'w') if output_path else sys.stdout as output_fh:

        if not mav10:
            os.environ['MAVLINK20'] = '1'

        if profile:
            import yappi    # We do the import here so that we won't barf if run normally and yappi not available
            yappi.start()

        if format == 'mat':
            # Scipy needed only for matlab format
            from scipy.io import savemat

            # Check that the output_path argument has been specified
            if output_path is None:
                print("output_path argument must be specified when mat format is selected")
                sys.exit(1)

        elif format =='json':
            output_fh.write('[')

        filename = log
        mlog = mavutil.mavlink_connection(filename, planner_format=planner,
                                        no_timestamps=no_timestamps,
                                        robust_parsing=robust,
                                        dialect=dialect,
                                        zero_time_base=zero_time_base)


        if csv_sep == "tab":
            csv_sep = "\t"

        types = types
        if types is not None:
            types = types.split(',')

        nottypes = nottypes
        if nottypes is not None:
            nottypes = nottypes.split(',')

        ext = os.path.splitext(filename)[1]
        isbin = ext in ['.bin', '.BIN', '.px4log']
        islog = ext in ['.log', '.LOG'] # NOTE: "islog" does not mean a tlog
        istlog = ext in ['.tlog', '.TLOG']

        reduction_yes = set()
        reduction_no = set()
        reduction_count = {}
        last_msg_rate_t = {}

        # Write out a header row as we're outputting in CSV format.
        fields = ['timestamp']
        offsets = {}
        if istlog and format == 'csv': # we know our fields from the get-go
            try:
                currentOffset = 1 # Store how many fields in we are for each message.
                for mtype in types:
                    try:
                        typeClass = "MAVLink_{0}_message".format(mtype.lower())
                        fields += [mtype + '.' + x for x in inspect.getfullargspec(getattr(mavutil.mavlink, typeClass).__init__).args[1:]]
                        offsets[mtype] = currentOffset
                        currentOffset += len(fields)
                    except IndexError:
                        sys.exit(1)
                    except AttributeError:
                        print("Message type '%s' not found" % (mtype))
                        sys.exit(1)
            except TypeError:
                print("You must specify a list of message types if outputting CSV format via the --types argument.")
                sys.exit(1)

            # The first line output are names for all columns
            output_fh.write(csv_sep.join(fields))

        if (isbin or islog) and format == 'csv': # need to accumulate columns from message
            if types is None or len(types) != 1:
                print("Need exactly one type when dumping CSV from bin file")
                sys.exit(1)

        # Track types found
        available_types = set()

        # for DF logs pre-calculate types list
        match_types=None
        if types is not None and hasattr(mlog, 'name_to_id'):
            for k in mlog.name_to_id.keys():
                if match_type(k, types):
                    if nottypes is not None and match_type(k, nottypes):
                        continue
                    if match_types is None:
                        match_types = []
                    match_types.append(k)

        if (isbin or islog) and format == 'csv':
            # Make sure the specified type was found
            if match_types is None:
                print("Specified type '%s' not found in log file" % (types[0]))
                sys.exit(1)
            # we need FMT messages for column headings
            match_types.append("FMT")

        last_loss = 0

        # Keep track of data from the current timestep. If the following timestep has the same data, it's stored in here as well. Output should therefore have entirely unique timesteps.
        out_dict = {}    # Dictionary to hold output data for 'mat' format options
        count = 0
        while True:
            m = mlog.recv_match(blocking=follow, type=match_types)
            if m is None:
                break
            m_type = m.get_type()
            available_types.add(m_type)
            if (isbin or islog) and m_type == "FMT" and format == 'csv':
                if m.Name == types[0]:
                    fields += m.Columns.split(',')
                    output_fh.write(csv_sep.join(fields))

            if reduce and reduce_msg(m_type, reduce, reduction_yes, reduction_no, reduction_count):
                continue

            if reduce_rate > 0 and reduce_rate_msg(m, reduce_rate, last_msg_rate_t):
                continue

            if not mavutil.evaluate_condition(condition, mlog.messages) and (
                    not (m_type in ['FMT', 'FMTU', 'MULT', 'PARM', 'MODE', 'UNIT', 'VER','CMD','MAVC','MSG','EV'] and meta)):
                continue
            if source_system is not None and source_system != m.get_srcSystem():
                continue
            if source_component is not None and source_component != m.get_srcComponent():
                continue
            if link is not None and link != m._link:
                continue

            if types is not None and m_type != 'BAD_DATA' and not match_type(m_type, types):
                continue

            if nottypes is not None and match_type(m_type, nottypes):
                continue

            # Ignore BAD_DATA messages is the user requested or if they're because of a bad prefix. The
            # latter case is normally because of a mismatched MAVLink version.
            if m_type == 'BAD_DATA' and (no_bad_data is True or m.reason == "Bad prefix"):
                continue

            # Grab the timestamp.
            timestamp = getattr(m, '_timestamp', 0.0)

            # Handle different output formats
            if format == 'json':
                handle_json_output(m, m_type, timestamp, show_source, output_fh, count)
            elif format == 'ndjson':
                handle_json_output(m, m_type, timestamp, show_source, output_fh, count, newline_delim=True)
            elif format == 'csv':
                handle_csv_output(m, m_type, timestamp, csv_sep, fields, isbin, islog, output_fh)
            elif format == 'mat':
                handle_mat_output(m, m_type, out_dict)
            elif format == 'pretty':
                handle_pretty_output(m, istlog, output_fh)
            elif format == 'types-only':
                # do nothing
                pass
            else:
                handle_standard_output(m, timestamp, show_source, show_seq, output_fh, isbin, islog, m_type, parms)

            if show_loss:
                if last_loss != mlog.mav_loss:
                    print("lost %d messages" % (mlog.mav_loss - last_loss))
                    last_loss = mlog.mav_loss
            
            count+=1

        if format == 'mat':
            # Export the .mat file
            savemat(output_path, out_dict, do_compression=compress)
        elif format == 'json':
            output_fh.write('\n]\n')
        elif format == 'types-only':
            for msgType in available_types:
                output_fh.write(msgType)

        if profile:
            yappi.get_func_stats().print_all()
            yappi.get_thread_stats().print_all()

        mlog.close()

if __name__=="__main__":
    args = parse_args()
    dump_log(**vars(args))