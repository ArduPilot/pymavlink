# MAV_CMD parameter range checking (`mav_cmd_helpers.h`)

MAVLink command parameters (p1–p7) carry numeric values whose valid ranges are defined in the MAVLink XML — for example, a heading param may only accept 0–360°, or a speed param may require a non-negative value.
Validating these ranges at the MAVLink layer lets a vehicle or GCS reject malformed commands an mission items early, before they reach the flight stack, reducing the risk of undefined behaviour caused by out-of-range inputs.

This PR provides efficient range checking in a generated mavgen_c header `mav_cmd_helpers.h`.
These can be used by any flight stack to check inputs.

In addition to explicit range checking, it also provides methods for checking that lat/lon values are in valid ranges.

## Generating the header

The header is generated using mavgen as normal.

```sh
python3 generator/mavgen.py \
      --lang C \
      --output /path/to/output \
      message_definitions/v1.0/common.xml
```

The `mav_cmd_helpers.h` in this case would be generated to the root of each of the generated dialect folders.
Include the result directly — it is a single self-contained `#pragma once` header with no library dependencies (only `<math.h>`, `<stdbool.h>`, and `<stdint.h>`).

## API

| Function                           | Description                                                                                                                                                                                                      |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `check_range(cmd, p1…p7)`          | Validate all seven params against XML-defined bounds. Returns `0` (ok, including when the command has no range data), or `1–7` (1-based index of first failing param). Geographic range is **not** checked here. |
| `lat_in_range(lat, is_int)`        | `1` if `lat` is in range for a latitude, `0` if out of range or NaN. Call after `param_invalid` to check coordinates.                                                                                            |
| `lon_in_range(lon, is_int)`        | `1` if `lon` is in range for a longitude, `0` if out of range or NaN. Same conditions as `lat_in_range`.                                                                                                         |
| `has_location(cmd)`                | `1` if the command carries lat/lon/alt, `0` if not, `-1` if unknown.                                                                                                                                             |
| `is_destination(cmd)`              | `1` if the command is a waypoint destination, `0` if not, `-1` if unknown.                                                                                                                                       |
| `param_invalid(param_val, is_int)` | Non-zero if `param_val` is an invalid/default value (NaN or ±0.0 by default; also INT32_MAX-scale when `is_int` is true). Invalid params are skipped by `check_range` (called with `is_int=false`).              |

### Using the methods

The methods are intended to be used in command handlers and during mission upload to reject MAV_CMD with passed values that are out of range.
The `check_range()` method returns `0` if all the passed params are all in range, have no range, or are the sentinel values - NaN/0/INT32MAX-for-param5or6, and otherwise returns the value of the first out of range param.
The `lat_in_range()` and `lon_in_range()` can further be used to check that lat/lon values aren't passed ranges that are bigger than valid lat/lon values.

This code shows how you might check a mission item.
The result is either 0, or the param number of the first out of range param.

```c
#include "common/mav_cmd_helpers.h"

/* Returns the 1-based index of the first out-of-range param, or 0 if all ok. */
int check_mission_item(const mavlink_mission_item_int_t *item)
{
    // check_range uses is_int=false internally, but that is fine: location params
    // (x/y) have no XML bounds so are never evaluated; p1-p4/p7 are always float.
    int r = check_range(item->command,
                        item->param1, item->param2, item->param3, item->param4,
                        (float)item->x, (float)item->y, item->z);
    if (r != 0) return r;

    if (has_location(item->command) == 1) {
        if (!param_invalid((float)item->x, true) && !lat_in_range((float)item->x, true)) return 5;
        if (!param_invalid((float)item->y, true) && !lon_in_range((float)item->y, true)) return 6;
    }

    return 0;
}
```

For a `COMMAND_LONG` (all params are plain floats):

```c
/* Returns the 1-based index of the first out-of-range param, or 0 if all ok. */
int check_command_long(const mavlink_command_long_t *cmd)
{
    int r = check_range(cmd->command,
                        cmd->param1, cmd->param2, cmd->param3, cmd->param4,
                        cmd->param5, cmd->param6, cmd->param7);
    if (r != 0) return r;

    if (has_location(cmd->command) == 1) {
        if (!param_invalid(cmd->param5, false) && !lat_in_range(cmd->param5, false)) return 5;
        if (!param_invalid(cmd->param6, false) && !lon_in_range(cmd->param6, false)) return 6;
    }

    return 0;
}
```

For a `COMMAND_INT` (x/y are `int32_t` × 1e7, cast to float with `is_int=true`):

```c
/* Returns the 1-based index of the first out-of-range param, or 0 if all ok. */
int check_command_int(const mavlink_command_int_t *cmd)
{
    int r = check_range(cmd->command,
                        cmd->param1, cmd->param2, cmd->param3, cmd->param4,
                        (float)cmd->x, (float)cmd->y, cmd->z);
    if (r != 0) return r;

    if (has_location(cmd->command) == 1) {
        if (!param_invalid((float)cmd->x, true) && !lat_in_range((float)cmd->x, true)) return 5;
        if (!param_invalid((float)cmd->y, true) && !lon_in_range((float)cmd->y, true)) return 6;
    }

    return 0;
}
```
