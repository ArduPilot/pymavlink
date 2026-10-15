#!/usr/bin/env python3
'''
graph a MAVLink log file
Andrew Tridgell August 2011
'''

import datetime
import matplotlib
import os
import re
import sys
import time
from math import *

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported.")

text_types = frozenset([str,])

# cope with rename of raw_input in python3
try:
    input = raw_input
except NameError:
    pass

colourmap = {
    'ardupilot' : {
        'MANUAL'    : (1.0,   0,   0),
        'AUTO'      : (  0, 1.0,   0),
        'LOITER'    : (  0,   0, 1.0),
        'FBWA'      : (1.0, 0.5,   0),
        'RTL'       : (  1,   0, 0.5),
        'STABILIZE' : (0.5, 1.0,   0),
        'LAND'      : (  0, 1.0, 0.5),
        'STEERING'  : (0.5,   0, 1.0),
        'HOLD'      : (  0, 0.5, 1.0),
        'ALT_HOLD'  : (1.0, 0.5, 0.5),
        'CIRCLE'    : (0.5, 1.0, 0.5),
        'POSITION'  : (1.0, 0.0, 1.0),
        'GUIDED'    : (0.5, 0.5, 1.0),
        'ACRO'      : (1.0, 1.0,   0),
        'CRUISE'    : (  0, 1.0, 1.0)
        },
    'px4' : {
        'MANUAL'    : (1.0,   0,   0),
        'SEATBELT'  : (  0.5, 0.5,   0),
        'EASY'      : (  0, 1.0,   0),
        'AUTO'    : (  0,   0, 1.0),
        'UNKNOWN'    : (  1.0,   1.0, 1.0)
        }
    }
colourmap["apm"] = colourmap["ardupilot"]

edge_colour = (0.1, 0.1, 0.1)

lowest_x = None
highest_x = None
plot_axes = {}  # y axis number -> matplotlib axes
unit_axis = {}  # unit -> y axis number, kept across logs
axis_units = {} # y axis number -> units of the fields plotted on it
plotted_lines = []  # every line plotted, in the order plotted
restack_cid = None  # resize handler restacking the y axes
restack_axes = []   # y axes whose range changes restack them
restacking = False  # set while the y axes are being stacked
restack_pending = False  # a y range has changed since the last restack
left_layout = None  # where the y axes were last stacked
redraw_timer = None  # draws the figure again after the y axes are restacked

def field_axes():
    '''work out which y axis each field is plotted on'''
    if 2 in axes:
        # axes chosen by hand with :2 suffixes
        return axes[:]
    # give each unit its own axis; fields without a unit share the first.
    # A unit keeps its axis for every log plotted
    ret = []
    for unit in units:
        if not unit:
            ret.append(1)
            continue
        if unit not in unit_axis:
            unit_axis[unit] = len(unit_axis) + 1
        ret.append(unit_axis[unit])
    return ret

def extra_axes():
    '''return the numbers of the y axes stacked left of the first'''
    return sorted(n for n in plot_axes if n >= 2)

def get_axis(n):
    '''return y axis n, creating it if needed'''
    if n not in plot_axes:
        plot_axes[n] = plot_axes[1].twinx()
    return plot_axes[n]

def time_locator(xrange, max_ticks):
    '''return a locator aiming for at most max_ticks ticks on xrange
    seconds; very long ranges get daily ticks however many there are'''
    intervals = [ 1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600,
                  900, 1800, 3600, 7200, 5*3600, 10*3600, 24*3600 ]
    for interval in intervals:
        if xrange / interval < max_ticks:
            break
    return matplotlib.dates.SecondLocator(interval=interval)

def figure_renderer(fig):
    '''return a renderer for measuring the artists of fig'''
    get_renderer = getattr(fig.canvas, 'get_renderer', None)
    if get_renderer is not None:
        return get_renderer()
    # non-Agg backends (e.g. cairo) have no get_renderer
    get_renderer = getattr(fig, '_get_renderer', None)
    if get_renderer is not None:
        return get_renderer()
    from matplotlib.backend_bases import _get_renderer
    return _get_renderer(fig)

def stack_left_axes(fig, extra):
    '''place the extra y axes to the left of the first, each one just clear
    of the tick labels and label of the axis inside it. Return True if
    anything moved'''
    global restacking, left_layout
    if restacking:
        # measuring the labels can autoscale an axis, changing its y range
        return False
    restacking = True
    try:
        layout = place_left_axes(fig, extra)
    finally:
        restacking = False
    # do it again whenever the window is resized or a y range changes, as
    # both change how much room the labels need
    global restack_cid
    if restack_cid is None:
        restack_cid = fig.canvas.mpl_connect(
            'resize_event', lambda event: stack_left_axes(fig, extra_axes()))
        fig.canvas.mpl_connect('draw_event', restack_after_draw)
    for ax in plot_axes.values():
        if ax not in restack_axes:
            restack_axes.append(ax)
            ax.callbacks.connect('ylim_changed', ylim_changed)
    changed = layout != left_layout
    left_layout = layout
    return changed

def ylim_changed(ax):
    '''a y range has changed, which changes how much room the tick labels
    need. Panning changes every axis's range at every mouse step, so
    restack once, after the next draw, rather than on every change'''
    global restack_pending
    if not restacking:
        restack_pending = True

def restack_after_draw(event):
    '''restack the y axes if a y range has changed, drawing again if that
    moved anything'''
    global restack_pending, redraw_timer
    if not restack_pending:
        return
    restack_pending = False
    fig = event.canvas.figure
    if stack_left_axes(fig, extra_axes()):
        # backends ignore a request to draw made during a draw, so make it
        # once this draw has finished
        redraw_timer = fig.canvas.new_timer(interval=0)
        redraw_timer.single_shot = True
        redraw_timer.add_callback(fig.canvas.draw_idle)
        redraw_timer.start()

def place_left_axes(fig, extra):
    '''do the work of stack_left_axes, returning the layout chosen'''
    ax1 = plot_axes[1]
    for n in extra:
        get_axis(n).spines['left'].set_position(('axes', 0))
        get_axis(n).yaxis.get_offset_text().set_x(0)
    # a non-Agg backend's renderer can change the figure's dpi
    dpi = fig.dpi
    renderer = figure_renderer(fig)
    # how far each axis's tick labels and label reach left of its line
    widths = [ax1.bbox.x0 - get_axis(n).yaxis.get_tightbbox(renderer).x0
              for n in [1] + extra]
    gap = 6 * fig.dpi / 72.0
    offsets = []
    offset = 0
    for w in widths[:-1]:
        offset += w + gap
        offsets.append(offset)
    left = (offsets[-1] + widths[-1] + gap) / fig.bbox.width
    fig.subplots_adjust(left=min(0.6, left))
    for n, offset in zip(extra, offsets):
        get_axis(n).spines['left'].set_position(('axes', -offset / ax1.bbox.width))
        # put the axis's scale (e.g. 1e6) over the axis, not the plot
        get_axis(n).yaxis.get_offset_text().set_x(-offset / ax1.bbox.width)
    if fig.dpi != dpi:
        fig.dpi = dpi
    return [min(0.6, left)] + offsets

def plotit(x, y, fields, colors=[]):
    '''plot a set of graphs using date for x axis'''
    global lowest_x, highest_x
    pylab.ion()
    fig = pylab.figure(num=1, figsize=(12,6))
    if 1 not in plot_axes:
        plot_axes[1] = fig.gca()
    ax1 = plot_axes[1]
    # saved images get small legend text
    legend_prop = None
    if args.output is not None and not args.output.endswith('.html'):
        legend_prop = {'size': 8}
    xrange = 0.0
    for i in range(0, len(fields)):
        if len(x[i]) == 0: continue
        if lowest_x is None or x[i][0] < lowest_x:
            lowest_x = x[i][0]
        if highest_x is None or x[i][-1] > highest_x:
            highest_x = x[i][-1]
    if highest_x is None or lowest_x is None:
        return
    xrange = highest_x - lowest_x
    xrange *= 24 * 60 * 60
    formatter = matplotlib.dates.DateFormatter('%H:%M:%S')
    faxes = field_axes()
    locator = time_locator(xrange, 15)
    if not args.xaxis:
        ax1.xaxis.set_major_locator(locator)
        ax1.xaxis.set_major_formatter(formatter)
    empty = True
    for i in range(0, len(fields)):
        if len(x[i]) == 0:
            print("Failed to find any values for field %s" % fields[i])
            continue
        if i < len(colors):
            color = colors[i]
        else:
            color = 'red'
        (tz, tzdst) = time.tzname
        n = faxes[i]
        ax = get_axis(n)
        if n != 1 and not args.xaxis:
            ax.xaxis.set_major_locator(locator)
            ax.xaxis.set_major_formatter(formatter)
        label = fields[i]
        if label.endswith(":2"):
            label = label[:-2]
        lines = []
        if args.xaxis:
            if args.marker is not None:
                marker = args.marker
            else:
                marker = '+'
            if args.linestyle is not None:
                linestyle = args.linestyle
            else:
                linestyle = 'None'
            lines = ax.plot(x[i], y[i], color=color, label=label,
                            linestyle=linestyle, marker=marker)
        else:
            if args.marker is not None:
                marker = args.marker
            else:
                marker = 'None'
            if args.linestyle is not None:
                linestyle = args.linestyle
            else:
                linestyle = '-'
            if len(y[i]) > 0 and type(y[i][0]) in text_types:
                # assume this is a piece of text to be rendered at a point in time
                last_text_time = -1
                last_text = None
                for j in range(0, len(x[i])):
                    if last_text is None:
                        last_text = "[" + y[i][j] + "]"
                        last_text_time = x[i][j]
                    elif x[i][j] == last_text_time:
                        last_text += "[" + y[i][j] + "]"
                    else:
                        ax.text(x[i][j], 10, last_text,
                                rotation=90,
                                alpha=0.3,
                                verticalalignment='baseline')
                        last_text = None
                        last_label_time = x[i][j]
                if last_text is not None:
                    ax.text(x[i][j], 10, last_text,
                            rotation=90,
                            alpha=0.3,
                            verticalalignment='baseline')
            else:
                lines = ax.plot_date(x[i], y[i], color=color, label=label,
                                     linestyle=linestyle, marker=marker, tz=None)
        plotted_lines.extend(lines)
        if lines:
            # text fields have no lines, and don't count towards the label
            unit = units[i] or ""
            axis_units.setdefault(n, [])
            if unit not in axis_units[n]:
                axis_units[n].append(unit)
        empty = False
    if args.flightmode is not None:
        for i in range(len(modes)-1):
            c = colourmap[args.flightmode].get(modes[i][1], edge_colour)
            ax1.axvspan(modes[i][0], modes[i+1][0], fc=c, ec=edge_colour, alpha=0.1)
        c = colourmap[args.flightmode].get(modes[-1][1], edge_colour)
        ax1.axvspan(modes[-1][0], ax1.get_xlim()[1], fc=c, ec=edge_colour, alpha=0.1)
    # the first axis has its own legend; the others share one, listing
    # their lines in the order they were plotted.  It goes on the last
    # axis created, which is drawn on top, so the other axes' lines don't
    # cover it.  The legends list every line on the axes, so earlier logs
    # keep their entries
    handles, labels = ax1.get_legend_handles_labels()
    if handles:
        ax1.legend(handles, labels, loc=args.legend, prop=legend_prop)
    right = [n for n in plot_axes if n != 1]
    for n in right:
        # a later log may have added an axis, so the legend moves to it
        legend = get_axis(n).get_legend()
        if legend is not None:
            legend.remove()
    right_handles = [line for line in plotted_lines
                     if line.axes is not ax1 and not line.get_label().startswith('_')]
    if right_handles:
        get_axis(right[-1]).legend(right_handles,
                                   [line.get_label() for line in right_handles],
                                   loc=args.legend2, prop=legend_prop)
    for n in axis_units:
        # only label an axis if every field on it has a unit
        if axis_units[n] and "" not in axis_units[n]:
            get_axis(n).set_ylabel(", ".join(axis_units[n]))
        else:
            # an earlier log may have labelled it
            get_axis(n).set_ylabel("")
    if len(plot_axes) > 2:
        # as in MAVExplorer, stack the extra axes on the left, and colour
        # each axis whose lines (across all files) share a colour to match
        extra = extra_axes()
        for n in extra:
            ax = get_axis(n)
            ax.yaxis.set_ticks_position('left')
            ax.yaxis.set_label_position('left')
            ax.spines['right'].set_visible(False)
            ax.spines['left'].set_visible(True)
        for n in plot_axes:
            line_colors = set(line.get_color() for line in get_axis(n).get_lines())
            c = line_colors.pop() if len(line_colors) == 1 else 'black'
            get_axis(n).yaxis.label.set_color(c)
            get_axis(n).tick_params(axis='y', colors=c)
            get_axis(n).spines['left'].set_color(c)
        stack_left_axes(fig, extra)
        if not args.xaxis:
            # the stacked axes narrow the plot, so it gets fewer time
            # ticks; a little fewer than in proportion, as the tick count
            # can come out one over the limit
            width = ax1.get_position().width
            full_width = (matplotlib.rcParams['figure.subplot.right'] -
                          matplotlib.rcParams['figure.subplot.left'])
            locator = time_locator(xrange, 13 * width / full_width)
            for ax in plot_axes.values():
                ax.xaxis.set_major_locator(locator)
    if empty:
        print("No data to graph")
        return
    return fig


from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--condition", default=None, help="select packets by a condition")
parser.add_argument("--labels", default=None, help="comma separated field labels")
parser.add_argument("--legend", default='upper left', help="default legend position")
parser.add_argument("--legend2", default='upper right', help="default legend2 position")
parser.add_argument("--marker", default=None, help="point marker")
parser.add_argument("--linestyle", default=None, help="line style")
parser.add_argument("--xaxis", default=None, help="X axis expression")
parser.add_argument("--multi", action='store_true', help="multiple files with same colours")
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("--flightmode", default=None,
                    help="Choose the plot background according to the active flight mode of the specified type, e.g. --flightmode=apm for ArduPilot or --flightmode=px4 for PX4 stack logs.  Cannot be specified with --xaxis.")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("--output", default=None, help="provide an output format")
parser.add_argument("--no-unit-axes", dest="unit_axes", action='store_false',
                    help="don't label y axes with units or give each unit its own y axis")
parser.add_argument("--timeshift", type=float, default=0, help="shift time on first graph in seconds")
parser.add_argument("logs_fields", metavar="<LOG or FIELD>", nargs="+")
args = parser.parse_args()

from pymavlink import mavutil

if args.flightmode is not None and args.xaxis:
    print("Cannot request flightmode backgrounds with an x-axis expression")
    sys.exit(1)

if args.flightmode is not None and args.flightmode not in colourmap:
    print("Unknown flight controller '%s' in specification of --flightmode (choose from %s)" % (args.flightmode, ",".join(colourmap.keys())))
    sys.exit(1)


if args.output is not None:
    matplotlib.use('Agg')

import pylab

filenames = []
fields = []
for f in args.logs_fields:
    if os.path.exists(f):
        filenames.append(f)
    else:
        fields.append(f)
msg_types = set()
multiplier = []
field_types = []
units = []

colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey', 'yellow', 'brown', 'darkcyan', 'cornflowerblue', 'darkmagenta', 'deeppink', 'darkred']

# work out msg types we are interested in
x = []
y = []
modes = []
axes = []
first_only = []
exprs = []  # fields without their suffixes
re_caps = re.compile('[A-Z_][A-Z0-9_]+')
for f in fields:
    caps = set(re.findall(re_caps, f))
    msg_types = msg_types.union(caps)
    field_types.append(caps)
    y.append([])
    x.append([])
    axes.append(1)
    first_only.append(False)
    units.append(None)
    # parse the suffixes now, so they apply even if the field never
    # gets any data
    if f.endswith(":2"):
        axes[-1] = 2
        f = f[:-2]
    if f.endswith(":1"):
        first_only[-1] = True
        f = f[:-2]
    exprs.append(f)

re_field = re.compile(r'^([A-Z_][A-Z0-9_]*)(\[[^\]]*\])?\.([A-Za-z_][A-Za-z0-9_]*)$')

def field_unit(f, msg):
    '''return the unit of a plain MSG.Field expression, or "" if not known.
    Returns None if msg is not the field's message, so a later message
    can supply the unit'''
    m = re_field.match(f)
    if m is None:
        return ""
    if m.group(1) != msg.get_type():
        return None
    field = m.group(3)
    fmt = getattr(msg, 'fmt', None)
    if fmt is not None and hasattr(fmt, 'get_unit'):
        # DataFlash message; units come from the log's FMTU metadata
        try:
            unit = fmt.get_unit(field)
        except KeyError:
            return ""
        if unit == "UNKNOWN":
            # ArduPilot's placeholder for a unit it has not specified
            return ""
        return unit
    # MAVLink message; units come from the message definition
    return getattr(msg, 'fieldunits_by_name', {}).get(field, "")

def add_data(t, msg, vars, flightmode):
    '''add some data'''
    mtype = msg.get_type()
    if args.flightmode is not None and (len(modes) == 0 or modes[-1][1] != flightmode):
        modes.append((t, flightmode))
    if mtype not in msg_types:
        return
    for i in range(0, len(fields)):
        if mtype not in field_types[i]:
            continue
        f = exprs[i]
        v = mavutil.evaluate_expression(f, vars)
        if v is None:
            continue
        if args.xaxis is None:
            xv = t
        else:
            xv = mavutil.evaluate_expression(args.xaxis, vars)
            if xv is None:
                continue
        if units[i] is None:
            # only fields with data get a unit, so a field that is
            # not in the log does not get an empty axis
            units[i] = field_unit(f, msg) if args.unit_axes else ""
        y[i].append(v)
        x[i].append(xv)

def process_file(filename, timeshift):
    '''process one file'''
    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps, zero_time_base=args.zero_time_base, dialect=args.dialect)
    vars = {}
    all_messages = {}

    while True:
        msg = mlog.recv_match(args.condition)
        if msg is None: break
        try:
            tdays = matplotlib.dates.date2num(datetime.datetime.fromtimestamp(msg._timestamp+timeshift))
        except ValueError:
            # this can happen if the log is corrupt
            # ValueError: year is out of range
            break
        all_messages[msg.get_type()] = msg
        add_data(tdays, msg, all_messages, mlog.flightmode)

if len(filenames) == 0:
    print("No files to process")
    sys.exit(1)

if args.labels is not None:
    labels = args.labels.split(',')
    if len(labels) != len(fields)*len(filenames):
        print("Number of labels (%u) must match number of fields (%u)" % (
            len(labels), len(fields)*len(filenames)))
        sys.exit(1)
else:
    labels = None

timeshift = args.timeshift

for fi in range(0, len(filenames)):
    f = filenames[fi]
    process_file(f, timeshift)
    timeshift = 0
    for i in range(0, len(x)):
        if first_only[i] and fi != 0:
            x[i] = []
            y[i] = []
            units[i] = None
    if labels:
        lab = labels[fi*len(fields):(fi+1)*len(fields)]
    else:
        lab = fields[:]
    if args.multi:
        col = colors[:]
    else:
        col = colors[fi*len(fields):]
    fig = plotit(x, y, lab, colors=col)
    for i in range(0, len(x)):
        x[i] = []
        y[i] = []
        # a field's unit may differ between logs
        units[i] = None
if args.output is None:
    pylab.show()
    pylab.draw()
    input('press enter to exit....')
else:
    fname, fext = os.path.splitext(args.output)
    if fext == '.html':
        import mpld3
        html = mpld3.fig_to_html(fig)
        f_out = open(args.output, 'w')
        f_out.write(html)
        f_out.close()
    else:
        pylab.savefig(args.output, bbox_inches='tight', dpi=200)
