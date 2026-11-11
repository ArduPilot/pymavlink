/*
  Builds and runs the generated testsuite.h (through testmav.c) with every
  diagnostic macro opted in, against the tests/wip_deprecated_superseded.xml
  fixture. The testsuite calls each flagged message's helpers, so it must
  wrap those calls in MAVLINK_DEPRECATED_CALL_BEGIN/END: if the generator
  drops that guard, this fails to build under -Werror. The default build of
  testmav.c defines none of the macros, so it cannot catch that.

  The macros are those of test_diagnostics.c.
*/
#define MAVLINK_WIP              __attribute__((deprecated("MAVLink WIP message used")))
#define MAVLINK_DEPRECATED       __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_SUPERSEDED       __attribute__((deprecated("MAVLink superseded message used")))
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#define MAVLINK_ENUM_WIP         __attribute__((unavailable("MAVLink WIP command/enum entry used")))
#else
#define MAVLINK_ENUM_WIP         __attribute__((deprecated("MAVLink WIP command/enum entry used")))
#endif
#define MAVLINK_ENUM_DEPRECATED  __attribute__((deprecated("MAVLink deprecated command/enum entry used")))
#define MAVLINK_ENUM_SUPERSEDED  __attribute__((deprecated("MAVLink superseded command/enum entry used")))

/* the fixture does not include minimal.xml, whose test testmav.c calls */
#define mavlink_test_minimal(system_id, component_id, last_msg) ((void)0)

#include "testmav.c"
