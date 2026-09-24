# Extended-header generator tests

Run from the pymavlink checkout with its parent on `PYTHONPATH`:

```sh
PYTHONPATH=.. python3 -m pytest -q tests/test_sysid32.py tests/test_sysid32_generators.py
```

The Python tests generate independent wire fixtures, including valid inner MAVLink
packets inside rejected payloads. Tests exercise empty and maximum payloads,
signed headers, both system-ID widths, payload broadcast and 8-bit targets,
fragmented input, and recovery to the following ordinary heartbeat. C and
JavaScript output is compared byte-for-byte with independently computed CRCs and
signatures. Optional compilers/runtimes are detected and reported as skips.

| Generator | Extended-header behavior | Test runtime |
| --- | --- | --- |
| C | Encode/decode, wide targets derived from payload target fields | GCC |
| Python/Python3 | Encode/decode, wide targets derived from payload target fields | Python |
| JavaScript_NextGen | Encode/decode, wide targets derived from payload target fields | Node |
| Lua | Decode C message structures; explicit layout selection for reduced payload buffers | Lua 5.3/5.4 shared library |
| C++11 | C framing consumes extensions; typed message deserialization rejects them | G++ |
| ObjC | C framing consumes extensions; message construction/dispatch rejects them | Apple Foundation/ARC; optional GNUstep syntax check |
| Java | Reject and consume unsupported frames | JDK |
| CS | Reject and consume unsupported frames | Mono |
| JavaScript/JavaScript_Stable | Reject and consume unsupported frames | Node |
| TypeScript | Generated framing adapter rejects unsupported frames before external runtime | TypeScript/Node |
| Ada 1/2 | Reject and consume unsupported frames within existing protocol scope | GNAT |
| Swift | Retain MAVLink 1; consume and reject MAVLink 2 frames | Swift |
| Spin2 | Reject and consume unsupported frames, retaining state across receive timeouts | FlexSpin (compile); compatible SpinSim or hardware (runtime) |
| WLua | Mark unsupported frames and advance to next frame | tshark |

Install the JavaScript dependencies in `generator/javascript`. For TypeScript,
run `npm install` in this directory or point `MAVLINK_TYPESCRIPT_NODE_MODULES` at
a directory containing the dependencies in `package.json`. Keep tool binaries on
`PATH`. Spin2 compile tests use FlexSpin 7.7.3. The runtime harness is included for
hardware or a compatible simulator, selected with `MAVLINK_SPINSIM`. SpinSim
0.99 cannot execute the compiler's structure/stack helpers correctly, so its
runtime test is opt-in and is not validated on that version.
The optional Linux Objective-C check uses `MAVLINK_GNUSTEP_ROOT` (the directory
containing `Foundation`) and `MAVLINK_OBJC_INCLUDE` (the directory containing
`objc`). Its runtime test requires macOS and is skipped on other systems.

## Wide targeting

`MAVLINK_IFLAG_SYSID32` (0x02) adds three source-system bytes.
`MAVLINK_IFLAG_TARGET32` (0x04) independently adds a uint32 target system
only. The target component stays in the payload. Header lengths are 10, 13, 14 and 17 bytes.
There is no system-ID capability flag or 8-bit target-header variant.

Set the message's `target_system` field (or equivalent, such as
`MANUAL_CONTROL.target`) to the full destination ID. Values above 255 use
the header and set the legacy payload target byte to 255, avoiding broadcast. Small targets and broadcast
stay in the payload. Changing a decoded target back to a small ID removes the
header target when repacked. MAVLink1 cannot carry wide IDs.

Messages without a target field remain untargeted. Adding XML target extension
fields to additional messages is a future protocol change, outside this PR.

C no longer generates per-message target-system getters, including differently
named fields such as `MANUAL_CONTROL.target`, for either protocol version.
With MAVLink 2 headers, use `mavlink_msg_get_target_system(&msg,
&packet.target_system, &target)` after decoding the payload. The helper returns
the full-width header target when present; pass a null payload-field pointer for
messages without that field. A present zero target returns true, while an
absent target returns false and leaves the output unchanged. MAVLink 1-only
headers have no extended targets; use the decoded payload field directly.

For raw C payload structs, use `packet.target_system = mavlink_msg_target_field(target)`
and pass the full `target` separately to the `_target()` send/finalize helper.
The inline conversion preserves 0–255 and maps larger IDs to the sentinel without
evaluating its argument twice. Generated packers apply this conversion automatically.
`_send_struct()` cannot carry a separate wide target; use
`_mav_finalize_message_chan_send_target()` with the converted payload and full target ID.

C provides two generic getters:

| Getter | Input | Result |
| --- | --- | --- |
| `mavlink_msg_get_target_sysid(msg, entry)` | Raw message and its `mavlink_get_msg_entry(msg->msgid)` metadata | Full target, or zero for broadcast/absent target; handles zero-trimmed payloads |
| `mavlink_msg_get_target_system(msg, field, out)` | Raw message and a pointer to its decoded uint8 target field (or null) | Target presence as a bool, writing the full target only when present |

C `_decode()` cannot store a wide target in its uint8 payload struct. Both the
wire byte and decoded field use `MAVLINK_TARGET_SYSTEM_SENTINEL` (255) for a wide
target. **255 is still a valid system ID, not a reserved value.** Never route
using that byte alone. `_decode()` followed by `_encode*()` loses the header
target and addresses system 255. Forward the original `mavlink_message_t`, or
recover the full target before editing and pass it to the widened `_pack*()` API:

```c
mavlink_command_long_t packet;
mavlink_msg_command_long_decode(&msg, &packet);
uint32_t target = mavlink_msg_get_target_sysid(&msg, mavlink_get_msg_entry(msg.msgid));
packet.param1 = 8;
mavlink_msg_command_long_pack(sysid, compid, &out, target, packet.target_component,
    packet.command, packet.confirmation, packet.param1, packet.param2, packet.param3,
    packet.param4, packet.param5, packet.param6, packet.param7);
```

A conforming sender only sets TARGET32 on messages with a target field. Handling
TARGET32 on a targetless message is outside this protocol contract; callers must
not rely on consistent interpretation across bindings.

The TypeScript generator still uses `@ifrunistuttgart/node-mavlink` for ordinary
MAVLink framing. Import `MAVLinkModule` from the generated `message-registry`
alongside `messageRegistry` to enable the framing guard. Importing the external
class directly bypasses the guard. The adapter buffers fragmented input, rejects
unsupported headers (including signatures, unsupported by the external runtime),
and exposes an `unsupportedFrames` count. Unsupported frames are discarded as
whole frames so embedded payload bytes cannot be dispatched as messages. If a
corrupted length/flag describes a larger frame, recovery waits for that declared
frame boundary; this deliberately favors avoiding false message dispatch.

Lua's `decode(message, message_map, wide_sysid)` and
`decode_header(message, wide_sysid)` accept the C structure layout explicitly:
`true` for the widened sysid structure, `false` for the original structure. This
layout is independent of the packet's SYSID32 flag. Omit the argument only for
the default full-size layouts; reduced `MAVLINK_MAX_PAYLOAD_LEN` values can make
the layouts indistinguishable by size and must specify it.

## C ABI compatibility

The widened `mavlink_message_t` and signing-stream structures change the C ABI.
With default payload storage, `mavlink_message_t` grows from 291 to 298 bytes.
Rebuild all libraries and consumers exchanging these structures together;
do not mix objects compiled against old and new generated headers.
