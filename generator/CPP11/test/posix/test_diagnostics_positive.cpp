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
  of them, and that each statement marked "// diagnosed" or "// not
  diagnosed" is diagnosed on that line or not.

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
    msg::WIP_MESSAGE m{};  // diagnosed
    (void)m;
}

static void use_deprecated_message()
{
    msg::DEPRECATED_MESSAGE m{};  // diagnosed
    (void)m;
}

static void use_superseded_message()
{
    msg::SUPERSEDED_MESSAGE m{};  // diagnosed
    (void)m;
}

static void use_flagged_enum_entries()
{
    ENTRY_FLAGS_ENUM w = ENTRY_FLAGS_ENUM::WIP;  // diagnosed
    ENTRY_FLAGS_ENUM d = ENTRY_FLAGS_ENUM::DEPRECATED;  // diagnosed
    ENTRY_FLAGS_ENUM s = ENTRY_FLAGS_ENUM::SUPERSEDED;  // diagnosed
    (void)w;
    (void)d;
    (void)s;
}

static void use_flagged_enums()
{
    // these enums are flagged as a whole: their entries carry no flag of
    // their own and inherit the enum's, and the type name is flagged too
    auto w = WIP_ENUM::INHERITS_WIP;  // diagnosed
    auto d = DEPRECATED_ENUM::INHERITS_DEPRECATED;  // diagnosed
    auto s = SUPERSEDED_ENUM::INHERITS_SUPERSEDED;  // diagnosed
    auto wd = WIP_ENUM::OWN_DEPRECATED;  // diagnosed
    // each type named on its own, since GCC 15 reports only the entry
    // where both an entry and its type are flagged
    WIP_ENUM wt{};  // diagnosed
    DEPRECATED_ENUM dt{};  // diagnosed
    SUPERSEDED_ENUM st{};  // diagnosed
    (void)w;
    (void)d;
    (void)s;
    (void)wd;
    (void)wt;
    (void)dt;
    (void)st;
}

static void use_extended_enums()
{
    // enums extended from wip_deprecated_superseded_base.xml: an entry
    // carries the flag of the declaration it came from, and the type only
    // the flags every declaration has
    auto b = EXTENDED_DEPRECATED_ENUM::FROM_FLAGGED_BASE;  // diagnosed
    auto d = EXTENDED_WIP_ENUM::FROM_FLAGGED_DIALECT;  // diagnosed
    auto bb = EXTENDED_BOTH_DEPRECATED_ENUM::FROM_BASE;  // diagnosed
    auto bd = EXTENDED_BOTH_DEPRECATED_ENUM::FROM_DIALECT;  // diagnosed
    EXTENDED_BOTH_DEPRECATED_ENUM bt{};  // diagnosed
    (void)b;
    (void)d;
    (void)bb;
    (void)bd;
    (void)bt;
}

static void use_unflagged_enum()
{
    // neither of these may be diagnosed: an enum whose entries are flagged
    // individually is not itself flagged, and nor are its other entries
    ENTRY_FLAGS_ENUM e = ENTRY_FLAGS_ENUM::PLAIN;  // not diagnosed
    (void)e;
}

static void use_unflagged_entries()
{
    // none of these may be diagnosed: entries from an unflagged
    // declaration of an extended enum, whose type is therefore not flagged
    auto d = EXTENDED_DEPRECATED_ENUM::FROM_UNFLAGGED_DIALECT;  // not diagnosed
    auto b = EXTENDED_WIP_ENUM::FROM_UNFLAGGED_BASE;  // not diagnosed
    EXTENDED_DEPRECATED_ENUM dt{};  // not diagnosed
    EXTENDED_WIP_ENUM wt{};  // not diagnosed
    (void)d;
    (void)b;
    (void)dt;
    (void)wt;
}

int main()
{
    use_wip_message();
    use_deprecated_message();
    use_superseded_message();
    use_flagged_enum_entries();
    use_flagged_enums();
    use_extended_enums();
    use_unflagged_enum();
    use_unflagged_entries();
    return 0;
}
