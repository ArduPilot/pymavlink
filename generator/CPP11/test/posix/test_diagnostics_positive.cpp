/*
  Positive counterpart to test_diagnostics.cpp: proves the opt-in
  diagnostics actually FIRE on real use of a flagged message struct or
  enum entry - test_diagnostics.cpp only proves the generated headers
  compile cleanly when nothing uses a flagged construct, which cannot
  tell a struct that correctly carries its attribute apart from one
  that never had it applied at all.

  Uses the dedicated tests/wip_deprecated_superseded.xml fixture rather
  than a real dialect message, so the set of flagged names this file
  depends on can't drift out from under it as common.xml/all.xml change.

  This file is expected to FAIL to build under -Werror - see the
  check_diagnostics_fire.sh driver in the Makefile rule, which requires
  the failure and additionally checks that every individual symbol
  below was actually diagnosed, not just that the build failed for some
  of them.

  MAVLINK_MSG_TYPE_WIP uses plain deprecated() here, rather than the
  unavailable() protocol.h documents: the point of this file is only to
  prove the attribute is actually present, and deprecated() keeps the
  diagnostic text uniform with the other two categories - the
  WIP-specific attribute choice itself is exercised by
  test_diagnostics.cpp instead. MAVLINK_ENUM_WIP keeps the toolchain
  guard (unavailable() on clang/GCC>=12, deprecated() otherwise) to
  match how that macro is actually documented for enum entries.
*/
#define MAVLINK_MSG_TYPE_WIP         __attribute__((deprecated("MAVLink WIP message used")))
#define MAVLINK_MSG_TYPE_DEPRECATED  __attribute__((deprecated("MAVLink deprecated message used")))
#define MAVLINK_MSG_TYPE_SUPERSEDED  __attribute__((deprecated("MAVLink superseded message used")))
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#define MAVLINK_ENUM_WIP             __attribute__((unavailable("MAVLink WIP command/enum entry used")))
#else
#define MAVLINK_ENUM_WIP             __attribute__((deprecated("MAVLink WIP command/enum entry used")))
#endif
#define MAVLINK_ENUM_DEPRECATED      __attribute__((deprecated("MAVLink deprecated command/enum entry used")))
#define MAVLINK_ENUM_SUPERSEDED      __attribute__((deprecated("MAVLink superseded command/enum entry used")))

#include "wip_deprecated_superseded.hpp"

using namespace mavlink::wip_deprecated_superseded;

static void use_wip_message()
{
    msg::WIP_MESSAGE m{};
    (void)m;
}

static void use_deprecated_message()
{
    msg::DEPRECATED_MESSAGE m{};
    (void)m;
}

static void use_superseded_message()
{
    msg::SUPERSEDED_MESSAGE m{};
    (void)m;
}

static void use_flagged_enum_entries()
{
    ENTRY_FLAGS_ENUM w = ENTRY_FLAGS_ENUM::WIP;
    ENTRY_FLAGS_ENUM d = ENTRY_FLAGS_ENUM::DEPRECATED;
    ENTRY_FLAGS_ENUM s = ENTRY_FLAGS_ENUM::SUPERSEDED;
    (void)w;
    (void)d;
    (void)s;
}

int main()
{
    use_wip_message();
    use_deprecated_message();
    use_superseded_message();
    use_flagged_enum_entries();
    return 0;
}
