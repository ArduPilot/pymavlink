/*
  Pins the generated gtestsuite.hpp fix history (see mavgen_cpp11.py's
  generate_gtestsuite_hpp()/generate_message_hpp()): builds the real
  generated common/gtestsuite.hpp test suite with every opt-in diagnostic
  macro actually defined, using the pragma-suppressible deprecated() form
  of MAVLINK_MSG_TYPE_WIP (so MAVLINK_TESTSUITE_SKIP_WIP is not needed -
  see test_gtestsuite_diagnostics_skipwip.cpp for that combination), and
  expects it to both compile clean under -Werror and run every test.

  If the blank-line leak, the unguarded macro duplication, or the
  self-referential diagnostic on the suite's own generated code (each
  fixed across a separate PR review round) ever comes back, this is what
  catches it, rather than needing a reviewer to notice by hand again.
*/
#define MAVLINK_MSG_TYPE_WIP         __attribute__((deprecated("MAVLink WIP message used")))
#define MAVLINK_MSG_TYPE_DEPRECATED  __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_MSG_TYPE_SUPERSEDED  __attribute__((deprecated("MAVLink superseded message used")))
#include "mtest.cpp"
