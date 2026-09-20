/*
  Companion to test_gtestsuite_diagnostics.cpp: the unavailable()-style
  MAVLINK_MSG_TYPE_WIP that message.hpp documents cannot be made to build
  the generated test suite without also defining
  MAVLINK_TESTSUITE_SKIP_WIP (see message.hpp - neither GCC nor clang can
  pragma-suppress unavailable()). Pins that this documented combination
  actually builds clean and still runs every non-WIP test; the (missing)
  WIP tests are covered instead by test_gtestsuite_diagnostics.cpp, which
  doesn't need the skip.
*/
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#define MAVLINK_MSG_TYPE_WIP         __attribute__((unavailable("MAVLink WIP message used")))
#else
#define MAVLINK_MSG_TYPE_WIP         __attribute__((deprecated("MAVLink WIP message used")))
#endif
#define MAVLINK_MSG_TYPE_DEPRECATED  __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_MSG_TYPE_SUPERSEDED  __attribute__((deprecated("MAVLink superseded message used")))
#define MAVLINK_TESTSUITE_SKIP_WIP
#include "mtest.cpp"
