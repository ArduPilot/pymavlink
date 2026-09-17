/*
  Regression test for the opt-in WIP/deprecated/superseded diagnostics
  documented in message.hpp/protocol.h.

  Every category is turned on here with real, type-valid attributes and
  this file never uses a flagged message struct or enum entry. It must
  still compile cleanly under -Wall -Wextra -Werror: if the C++11
  generator ever goes back to attributing the message struct with the
  function-only MAVLINK_WIP/DEPRECATED/SUPERSEDED macros (which accept
  warning()/error(), invalid on a type), this file fails to build even
  though no user code here should be diagnosed.
*/
#define MAVLINK_MSG_TYPE_WIP         __attribute__((unavailable("MAVLink WIP message used")))
#define MAVLINK_MSG_TYPE_DEPRECATED  __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_MSG_TYPE_SUPERSEDED  __attribute__((deprecated("MAVLink superseded message used")))
#define MAVLINK_ENUM_WIP             __attribute__((unavailable("MAVLink WIP command/enum entry used")))
#define MAVLINK_ENUM_DEPRECATED      __attribute__((deprecated("MAVLink deprecated command/enum entry used")))
#define MAVLINK_ENUM_SUPERSEDED      __attribute__((deprecated("MAVLink superseded command/enum entry used")))

#include "all.hpp"

int main()
{
    return 0;
}
