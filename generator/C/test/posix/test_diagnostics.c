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
*/
#define MAVLINK_WIP              __attribute__((error("MAVLink WIP message used")))
#define MAVLINK_DEPRECATED       __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_SUPERSEDED       __attribute__((deprecated("MAVLink superseded message used")))
#define MAVLINK_ENUM_WIP         __attribute__((unavailable("MAVLink WIP command/enum entry used")))
#define MAVLINK_ENUM_DEPRECATED  __attribute__((deprecated("MAVLink deprecated command/enum entry used")))
#define MAVLINK_ENUM_SUPERSEDED  __attribute__((deprecated("MAVLink superseded command/enum entry used")))

#include "mavlink.h"

int main(void)
{
    return 0;
}
