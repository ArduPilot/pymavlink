/*
  Positive counterpart to test_diagnostics.c: proves the opt-in
  diagnostics actually FIRE on real use, one call site at a time, for
  every wrapper that carries MSG_ATTRIBUTE/ENTRY_ATTRIBUTE.

  test_diagnostics.c only proves the generated headers compile cleanly
  with nothing calling a flagged construct - it cannot tell a wrapper
  that correctly carries the attribute apart from one that silently
  lost it (which is exactly how _pack_status()/_encode_status() shipped
  without MSG_ATTRIBUTE for a while: that compile-only test could not
  have caught it, since it never called them).

  Uses the dedicated tests/wip_deprecated_superseded.xml fixture rather
  than a real dialect message, so the set of flagged names this file
  depends on can't drift out from under it as common.xml/all.xml change.

  Covers all three categories (WIP/deprecated/superseded), not just
  deprecated/superseded: an earlier version of this file omitted WIP
  entirely, so reverting the WIP half of the feature to a no-op left
  every test in this Makefile green.

  This file is expected to FAIL to build under -Werror - see the
  check_diagnostics_fire.sh driver in the Makefile rule, which requires
  the failure and additionally checks that every individual symbol
  below was actually diagnosed, not just that the build failed for some
  of them.
*/
#include <string.h>

/* WIP uses plain deprecated() here too, rather than the error()/unavailable()
   protocol.h documents: the point of this file is only to prove that the
   attribute is actually present on every wrapper, and deprecated() keeps
   the diagnostic text (and check_diagnostics_fire.sh's matching) uniform
   with the other two categories - the WIP-specific attribute choice itself
   is exercised by test_diagnostics.c/test_diagnostics_be.c instead. */
#define MAVLINK_WIP               __attribute__((deprecated("MAVLink WIP message used")))
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

static mavlink_message_t msg;
static mavlink_status_t status;

static void use_wip_message(void)
{
    mavlink_wip_message_t w;
    memset(&w, 0, sizeof(w));

    mavlink_msg_wip_message_pack(1, 1, &msg, 0);
    mavlink_msg_wip_message_pack_chan(1, 1, 0, &msg, 0);
    mavlink_msg_wip_message_pack_status(1, 1, &status, &msg, 0);
    mavlink_msg_wip_message_encode(1, 1, &msg, &w);
    mavlink_msg_wip_message_encode_chan(1, 1, 0, &msg, &w);
    mavlink_msg_wip_message_encode_status(1, 1, &status, &msg, &w);
    mavlink_msg_wip_message_decode(&msg, &w);
    (void)mavlink_msg_wip_message_get_value(&msg);
}

static void use_deprecated_message(void)
{
    mavlink_deprecated_message_t d;
    memset(&d, 0, sizeof(d));

    mavlink_msg_deprecated_message_pack(1, 1, &msg, 0);
    mavlink_msg_deprecated_message_pack_chan(1, 1, 0, &msg, 0);
    mavlink_msg_deprecated_message_pack_status(1, 1, &status, &msg, 0);
    mavlink_msg_deprecated_message_encode(1, 1, &msg, &d);
    mavlink_msg_deprecated_message_encode_chan(1, 1, 0, &msg, &d);
    mavlink_msg_deprecated_message_encode_status(1, 1, &status, &msg, &d);
    mavlink_msg_deprecated_message_decode(&msg, &d);
    (void)mavlink_msg_deprecated_message_get_value(&msg);
}

static void use_superseded_message(void)
{
    mavlink_superseded_message_t s;
    memset(&s, 0, sizeof(s));

    mavlink_msg_superseded_message_pack(1, 1, &msg, 0);
    mavlink_msg_superseded_message_pack_chan(1, 1, 0, &msg, 0);
    mavlink_msg_superseded_message_pack_status(1, 1, &status, &msg, 0);
    mavlink_msg_superseded_message_encode(1, 1, &msg, &s);
    mavlink_msg_superseded_message_encode_chan(1, 1, 0, &msg, &s);
    mavlink_msg_superseded_message_encode_status(1, 1, &status, &msg, &s);
    mavlink_msg_superseded_message_decode(&msg, &s);
    (void)mavlink_msg_superseded_message_get_value(&msg);
}

static void use_flagged_enum_entries(void)
{
    /* ENTRY_FLAGS_ENUM_A's own enum carries no <deprecated>/<superseded> of
       its own - these two entries are flagged individually. (DEPRECATED_ENUM_A
       and SUPERSEDED_ENUM_A, by contrast, are entries of enums that are
       themselves flagged as a whole; neither backend currently emits
       anything for that case - a separate, pre-existing gap - so they
       wouldn't exercise MAVLINK_ENUM_DEPRECATED/SUPERSEDED at all.) */
    int w = ENTRY_FLAGS_ENUM_WIP;
    int d = ENTRY_FLAGS_ENUM_DEPRECATED;
    int s = ENTRY_FLAGS_ENUM_SUPERSEDED;
    (void)w;
    (void)d;
    (void)s;
}

int main(void)
{
    use_wip_message();
    use_deprecated_message();
    use_superseded_message();
    use_flagged_enum_entries();
    return 0;
}
