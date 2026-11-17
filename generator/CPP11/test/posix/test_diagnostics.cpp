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

  MAVLINK_MSG_TYPE_WIP/MAVLINK_ENUM_WIP use unavailable() where the
  toolchain supports it (clang, GCC >= 12) and fall back to deprecated()
  otherwise (e.g. GCC 11 on Ubuntu 22.04, this project's CI baseline): on
  an unsupporting GCC, unavailable() is not merely inert, it makes GCC
  emit "attribute directive ignored" at declaration time regardless of
  -Werror or of whether the entry/struct is ever used - see protocol.h.

  The function-only MAVLINK_WIP/DEPRECATED/SUPERSEDED macros are also
  defined here, exactly per protocol.h's own recipe (error()/deprecated()),
  even though nothing in this file calls the plain-C helper functions they
  back (all.hpp doesn't pull those in unless TEST_INTEROP is defined).
  Without them defined, a regression that puts one of these three back on
  the C++11 message struct - the exact bug this test exists to catch -
  would silently expand to the message.hpp no-op default and this file
  would build clean either way.
*/
#define MAVLINK_WIP                  __attribute__((error("MAVLink WIP message used")))
#define MAVLINK_DEPRECATED           __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_SUPERSEDED           __attribute__((deprecated("MAVLink superseded message used")))
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#define MAVLINK_MSG_TYPE_WIP         __attribute__((unavailable("MAVLink WIP message used")))
#define MAVLINK_ENUM_WIP             __attribute__((unavailable("MAVLink WIP command/enum entry used")))
#else
#define MAVLINK_MSG_TYPE_WIP         __attribute__((deprecated("MAVLink WIP message used")))
#define MAVLINK_ENUM_WIP             __attribute__((deprecated("MAVLink WIP command/enum entry used")))
#endif
#define MAVLINK_MSG_TYPE_DEPRECATED  __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_MSG_TYPE_SUPERSEDED  __attribute__((deprecated("MAVLink superseded message used")))
#define MAVLINK_ENUM_DEPRECATED      __attribute__((deprecated("MAVLink deprecated command/enum entry used")))
#define MAVLINK_ENUM_SUPERSEDED      __attribute__((deprecated("MAVLink superseded command/enum entry used")))

#include "all.hpp"

int main()
{
    return 0;
}
