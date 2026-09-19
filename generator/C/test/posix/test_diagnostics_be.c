/*
  Companion to test_diagnostics.c, built by the Makefile with
  -DNATIVE_BIG_ENDIAN so the #if MAVLINK_NEED_BYTE_SWAP branch of
  _decode()/_send_struct() is the one actually compiled (the default,
  little-endian build takes the #else/memcpy branch of those two
  functions instead, which calls nothing attributed and so cannot
  exercise their self-referential call to _get_<field>()/_send()).
  MAVLINK_USE_CONVENIENCE_FUNCTIONS is also enabled here, since
  _send_struct() only exists under that macro; the mavlink_system/
  comm_send_ch scaffolding below is only needed to satisfy the
  resulting declarations, not for anything this file actually calls.

  See test_diagnostics.c for the macro choices themselves.
*/
#include <stdint.h>

#define MAVLINK_WIP              __attribute__((error("MAVLink WIP message used")))
#define MAVLINK_DEPRECATED       __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_SUPERSEDED       __attribute__((deprecated("MAVLink superseded message used")))
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#define MAVLINK_ENUM_WIP         __attribute__((unavailable("MAVLink WIP command/enum entry used")))
#else
#define MAVLINK_ENUM_WIP         __attribute__((deprecated("MAVLink WIP command/enum entry used")))
#endif
#define MAVLINK_ENUM_DEPRECATED  __attribute__((deprecated("MAVLink deprecated command/enum entry used")))
#define MAVLINK_ENUM_SUPERSEDED  __attribute__((deprecated("MAVLink superseded command/enum entry used")))

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"
static mavlink_system_t mavlink_system = {1, 1};
static void comm_send_ch(mavlink_channel_t chan, uint8_t c);

#include "mavlink.h"

static void comm_send_ch(mavlink_channel_t chan, uint8_t c)
{
    (void)chan;
    (void)c;
}

int main(void)
{
    return 0;
}
