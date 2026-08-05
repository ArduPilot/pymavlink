#!/usr/bin/env python3

'''
show MAVLink packet loss
'''
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("--stats", action='store_true',
                    help="also break loss down by triggering message type")
parser.add_argument("--sysid", type=int, default=None,
                    help="restrict loss calculation to packets with this srcSystem")
parser.add_argument("--compid", type=int, default=None,
                    help="restrict loss calculation to packets with this srcComponent")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil


def mavloss(filename):
    '''work out signal loss times for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=args.planner,
                                      notimestamps=args.notimestamps,
                                      dialect=args.dialect,
                                      robust_parsing=args.robust)

    # Track the reasons for MAVLink parsing errors and print them all out at the end.
    reason_ids = set()
    reasons = []

    # Per-(srcSystem, srcComponent) and per-message-type loss attribution.
    # This mirrors the sequence-tracking that mavutil.post_message() does so
    # the totals here add up to mlog.mav_loss. The "triggering" message for
    # a sequence gap is the message whose seq != expected.
    last_seq = {}
    radio_tuple = (ord('3'), ord('D'))
    src_count = {}
    src_loss = {}
    type_count = {}     # how many messages of each (src, type) were seen
    type_loss = {}      # diff attributed to each (src, type) when it broke sequence
    type_trigger = {}   # how many times each (src, type) was the trigger

    def loss_hook(_mlog, msg):
        if msg.get_type() == 'BAD_DATA':
            return
        src_tuple = (msg.get_srcSystem(), msg.get_srcComponent())
        if src_tuple == radio_tuple or msg.get_msgId() < 0:
            return
        if args.sysid is not None and src_tuple[0] != args.sysid:
            return
        if args.compid is not None and src_tuple[1] != args.compid:
            return
        mtype = msg.get_type()
        type_count[(src_tuple, mtype)] = type_count.get((src_tuple, mtype), 0) + 1
        seq2 = msg.get_seq()
        prev = last_seq.get(src_tuple, -1)
        if prev != -1:
            expected = (prev + 1) % 256
            if expected != seq2:
                diff = (seq2 - expected) % 256
                src_loss[src_tuple] = src_loss.get(src_tuple, 0) + diff
                type_loss[(src_tuple, mtype)] = type_loss.get((src_tuple, mtype), 0) + diff
                type_trigger[(src_tuple, mtype)] = type_trigger.get((src_tuple, mtype), 0) + 1
        last_seq[src_tuple] = seq2
        src_count[src_tuple] = src_count.get(src_tuple, 0) + 1

    filtering = args.sysid is not None or args.compid is not None
    mlog.message_hooks.append(loss_hook)

    while True:
        m = mlog.recv_match(condition=args.condition)

        # Stop parsing the file once we've reached the end
        if m is None:
            break

        # Save the parsing failure reason for this message if it's a new one
        if m.get_type() == 'BAD_DATA':
            reason_id = ''.join(m.reason.split(' ')[0:3])
            if reason_id not in reason_ids:
                reason_ids.add(reason_id)
                reasons.append(m.reason)

    # Per-(sysid, compid) packet loss table, sorted by packet count desc.
    # MAVLink sequence numbers are per-source, so each tuple is independent.
    print("  %-8s %-8s %10s %10s %7s" % ("sysid", "compid", "packets", "lost", "loss%"))
    sources = sorted(src_count.keys(), key=lambda s: -src_count[s])
    for s in sources:
        n = src_count[s]
        lost = src_loss.get(s, 0)
        pct = (100.0 * lost) / (n + lost) if (n + lost) > 0 else 0.0
        print("  %-8u %-8u %10u %10u %6.1f%%" % (s[0], s[1], n, lost, pct))

    total_count = sum(src_count.values())
    total_loss = sum(src_loss.values())
    pct = (100.0 * total_loss) / (total_count + total_loss) if (total_count + total_loss) > 0 else 0.0
    label = "Total"
    if filtering:
        filt = []
        if args.sysid is not None:
            filt.append("sysid=%u" % args.sysid)
        if args.compid is not None:
            filt.append("compid=%u" % args.compid)
        label = "Total [%s]" % ",".join(filt)
    print("%s: %u packets, %u lost %.1f%%" % (label, total_count, total_loss, pct))

    # Also print out the reasons why losses occurred
    if len(reasons) > 0:
        print("Packet loss at least partially attributed to the following:")
        for r in reasons:
            print("  * " + r)

    if args.stats:
        print_msg_type_stats(type_count, type_loss, type_trigger)


def print_msg_type_stats(type_count, type_loss, type_trigger):
    '''print per-(src, message-type) loss attribution'''
    triggered = [k for k, v in type_loss.items() if v > 0]
    if not triggered:
        print("")
        print("No message-type triggered any sequence gaps.")
        return

    print("")
    print("Message types triggering loss-count increments:")
    print("  %-6s %-6s %-30s %10s %10s %10s" %
          ("sysid", "compid", "msg_type", "seen", "triggers", "lost"))
    for key in sorted(triggered, key=lambda k: -type_loss[k]):
        (src, mtype) = key
        print("  %-6u %-6u %-30s %10u %10u %10u" %
              (src[0], src[1], mtype,
               type_count.get(key, 0),
               type_trigger.get(key, 0),
               type_loss[key]))


for filename in args.logs:
    mavloss(filename)
