/*
  Regression test for the opt-in WIP/deprecated/superseded diagnostics
  documented in protocol.h.

  Every category is turned on here using exactly the attributes protocol.h's
  own usage example recommends, and this file never calls a flagged message
  or enum entry. It must still compile cleanly under -Wall -Werror: if the
  generator ever reintroduces a self-referential warning (e.g. an _encode()
  wrapper calling an already-attributed _pack(), or a _pack_status()/
  _encode_status() pair that silently carries no attribute at all), merely
  including the header starts failing even though no user code here should
  be diagnosed.

  MAVLINK_ENUM_WIP uses unavailable() where the toolchain supports it on an
  enumerator (clang, GCC >= 12) and falls back to deprecated() otherwise
  (e.g. GCC 11 on Ubuntu 22.04, this project's CI baseline): on an
  unsupporting GCC, unavailable() is not merely inert, it makes GCC emit
  "attribute directive ignored" for every enum entry so flagged, at
  declaration time, regardless of -Werror or of whether the entry is ever
  used - see protocol.h.
*/
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

#include "mavlink.h"

int main(void)
{
    return 0;
}
