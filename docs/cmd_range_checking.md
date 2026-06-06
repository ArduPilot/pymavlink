# MAV_CMD parameter range checking (`mavlink_cmd_helpers.h`)

`mavlink_cmd_helpers.h` is a generated C header that provides O(log n) range validation for MAVLink command parameters.
It is produced by [`generator/mavgen_c_cmd_helpers.py`](../generator/mavgen_c_cmd_helpers.py) from the MAVLink XML `minValue`/`maxValue` attributes and the `hasLocation`/`isDestination` per-command flags.

## Generating the header

```sh
python3 generator/mavgen_c_cmd_helpers.py \
    message_definitions/v1.0/common.xml \
    --output path/to/mavlink_cmd_helpers.h
```

Include the result directly — it is a single self-contained `#pragma once` header with no library dependencies (only `<math.h>` and `<stdint.h>`).

---

## API

| Function | Description |
|---|---|
| `check_range(cmd, p1…p7)` | Validate all seven params against XML-defined bounds. Returns `0` (ok, including when the command has no range data), or `1–7` (1-based index of first failing param). Geographic range is **not** checked here. |
| `lat_in_range(v, is_int)` | `1` if `v` is a valid latitude (or sentinel), `0` if out of range. Optional — call when you want to catch out-of-range coordinates before passing to the flight stack. |
| `lon_in_range(v, is_int)` | `1` if `v` is a valid longitude (or sentinel), `0` if out of range. Optional — same conditions as `lat_in_range`. |
| `has_location(cmd)` | `1` if the command carries lat/lon/alt, `0` if not, `-1` if unknown. |
| `is_destination(cmd)` | `1` if the command is a waypoint destination, `0` if not, `-1` if unknown. |
| `is_sentinel(v)` | Non-zero if `v` is a "not provided" value (NaN or ±0.0 by default). Sentinels are skipped by `check_range`. |

### `is_int` — coordinate-type flag for `lat_in_range` / `lon_in_range`

`MISSION_ITEM_INT` and `COMMAND_INT` encode p5/p6 as `int32_t` multiplied by 1×10⁷.
`COMMAND_LONG` uses plain `float` degrees.
Pass `is_int` to select the appropriate range:

| Message type | `is_int` | Valid lat range | Valid lon range |
|---|---|---|---|
| `COMMAND_INT` / `MISSION_ITEM_INT` | `true` | ±9×10⁸ (±90° × 1e7) | ±1.8×10⁹ (±180° × 1e7) |
| `COMMAND_LONG` | `false` | ±90° | ±180° |

---

## Example: `MAV_CMD_DO_SET_HOME` (179)

This command illustrates all three validation paths:

| Param | Label | XML bounds | How `check_range` treats it |
|---|---|---|---|
| 1 | Use Current | *(none)* | no XML bound — passes unless sentinel |
| 2 | Roll | −180 … 180 °| explicit bounds from XML |
| 3 | Pitch | −90 … 90 ° | explicit bounds from XML |
| 4 | Yaw | −180 … 180 ° | explicit bounds from XML |
| 5 | Latitude | *(none in XML)* | implicit geographic range via `hasLocation="true"` |
| 6 | Longitude | *(none in XML)* | implicit geographic range via `hasLocation="true"` |
| 7 | Altitude | *(none)* | no XML bound — always passes |

In the generated table this looks like:

```c
// param_bounds[]
{ 179, 2, -180.0f,  180.0f },   // Roll
{ 179, 3,  -90.0f,   90.0f },   // Pitch
{ 179, 4, -180.0f,  180.0f },   // Yaw

// cmd_flags_table[]
{ 179, CMD_FLAG_HAS_LOCATION }, // DO_SET_HOME
```

---

### Unified handler (`COMMAND_INT`, `MISSION_ITEM_INT`, and `COMMAND_LONG`)

In practice both PX4 and ArduPilot normalise all three message types into a single command handler before range-checking.
The caller casts `int32_t` x/y fields to `float` and passes `is_int` to select the correct coordinate scale:
`int32_t × 1e7` values span ±9×10⁸/±1.8×10⁹, while float-degree values span ±90/±180.
`is_int` also enables the INT32\_MAX sentinel check inside `is_coord_sentinel`.

```c
#include "mavlink_cmd_helpers.h"

void handle_command_both(mavlink_channel_t chan,
                         uint8_t sender_sysid, uint8_t sender_compid,
                         uint16_t command,
                         float p1, float p2, float p3, float p4,
                         float p5, float p6, float p7,
                         bool is_int, uint8_t frame)
{
    int r = check_range(command, p1, p2, p3, p4, p5, p6, p7);

    // Geographic check is optional — only needed when the receiver cares
    // about catching out-of-range lat/lon before passing to the flight stack.
    if (r == 0 && has_location(command) == 1) {
        // COMMAND_LONG has no frame field so the check always applies.
        // For INT types, restrict to global frames where p5/p6 are lat/lon.
        bool check_geo = !is_int;
        if (is_int) {
            check_geo = (frame == MAV_FRAME_GLOBAL              ||
                         frame == MAV_FRAME_GLOBAL_INT          ||
                         frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
                         frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        }
        if (check_geo) {
            if (!lat_in_range(p5, is_int)) r = 5;
            else if (!lon_in_range(p6, is_int)) r = 6;
        }
    }

    if (r > 0) {
        // r = 1-7: NACK with the 1-based index of the failing param.
        mavlink_msg_command_ack_send(chan, command, MAV_RESULT_DENIED,
            /*progress=*/0, /*result_param2=*/r,
            sender_sysid, sender_compid);
        return;
    }

    // r == 0: params valid (or no range data for this command) — accept.
}
```

Call site for each message type:

```c
/* COMMAND_INT or MISSION_ITEM_INT — x/y are int32_t × 1e7: */
handle_command_both(chan, sysid, compid, msg->command,
    msg->param1, msg->param2, msg->param3, msg->param4,
    (float)msg->x, (float)msg->y, msg->z,
    /*is_int=*/true, msg->frame);

/* COMMAND_LONG — params are float degrees, no frame field: */
handle_command_both(chan, sysid, compid, msg->command,
    msg->param1, msg->param2, msg->param3, msg->param4,
    msg->param5, msg->param6, msg->param7,
    /*is_int=*/false, /*frame=*/0);
```

For `MAV_CMD_DO_SET_HOME` (179) with `MAV_FRAME_GLOBAL` and INT-scaled coordinates:

| Scenario | p2 | p3 | p4 | p5 (`x`) | p6 (`y`) | `r` |
|---|---|---|---|---|---|---|
| Roll out of range | 200.0 | 0.0 | 0.0 | 473000000 | 85000000 | **2** |
| Pitch out of range | 45.0 | -100.0 | 0.0 | 473000000 | 85000000 | **3** |
| Longitude out of range | 45.0 | 30.0 | -90.0 | 473000000 | 1900000000 | **6** |
| All valid | 45.0 | 30.0 | -90.0 | 473000000 | 85000000 | **0** |
| All sentinels (NaN) | NaN | NaN | NaN | NaN | NaN | **0** |

*(p5 = 473000000 = 47.3°N × 1e7, p6 = 85000000 = 8.5°E × 1e7, p6 = 1900000000 = 190° × 1e7 — out of the ±180° × 1e7 valid range.
Note: values ≥ 2×10⁹ are treated as an INT32\_MAX sentinel and skipped rather than rejected — use a value in (1.8×10⁹, 2.0×10⁹) to trigger a longitude rejection.)*

For `MAV_CMD_DO_SET_HOME` (179) with float params p5/p6 in degrees (`COMMAND_LONG`):

| Scenario | p2 | p3 | p4 | p5 (lat °) | p6 (lon °) | `r` |
|---|---|---|---|---|---|---|
| Roll out of range | 200.0 | 0.0 | 0.0 | 47.3 | 8.5 | **2** |
| Pitch out of range | 45.0 | -100.0 | 0.0 | 47.3 | 8.5 | **3** |
| Longitude out of range | 45.0 | 30.0 | -90.0 | 47.3 | 200.0 | **6** |
| All valid | 45.0 | 30.0 | -90.0 | 47.3 | 8.5 | **0** |

---

## Unknown / custom commands

If the command ID is not found in either `param_bounds` or `cmd_flags_table`, `check_range` returns `0` — the command is accepted.
Range checking simply does not apply to commands with no XML range data, so there is nothing to reject.
This covers vendor-specific commands, future commands not yet in the XML, and any standard command whose params have no `minValue`/`maxValue` attributes and whose `hasLocation` is false.

---

## C++ usage

The header wraps everything in `namespace mavlink_cmd_helpers` when compiled as C++:

```cpp
#include "mavlink_cmd_helpers.h"

int r = mavlink_cmd_helpers::check_range(cmd, p1, p2, p3, p4, p5, p6, p7);
bool loc = mavlink_cmd_helpers::has_location(cmd) == 1;
```
