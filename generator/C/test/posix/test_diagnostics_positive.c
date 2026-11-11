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

/* All three message macros use deprecated(), as protocol.h recommends
   for messages (SUPERSEDED included, which protocol.h leaves empty), so
   the diagnostic text check_diagnostics_fire.sh matches is uniform. */
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

/* the _send helpers exist only with convenience functions enabled, which
   needs mavlink_system and comm_send_ch declared before mavlink.h */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"
static mavlink_system_t mavlink_system = {1, 1};
static void comm_send_ch(mavlink_channel_t chan, uint8_t c);

#include "mavlink.h"

static void comm_send_ch(mavlink_channel_t chan, uint8_t c)
{
    (void)chan;
    (void)c;
}

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
    mavlink_msg_wip_message_send(MAVLINK_COMM_0, 0);
    mavlink_msg_wip_message_send_struct(MAVLINK_COMM_0, &w);
    mavlink_msg_wip_message_send_buf(&msg, MAVLINK_COMM_0, 0);
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
    mavlink_msg_deprecated_message_send(MAVLINK_COMM_0, 0);
    mavlink_msg_deprecated_message_send_struct(MAVLINK_COMM_0, &d);
    mavlink_msg_deprecated_message_send_buf(&msg, MAVLINK_COMM_0, 0);
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
    mavlink_msg_superseded_message_send(MAVLINK_COMM_0, 0);
    mavlink_msg_superseded_message_send_struct(MAVLINK_COMM_0, &s);
    mavlink_msg_superseded_message_send_buf(&msg, MAVLINK_COMM_0, 0);
}

static void use_flagged_enum_entries(void)
{
    /* ENTRY_FLAGS_ENUM itself carries no flag - these entries are
       flagged individually. */
    int w = ENTRY_FLAGS_ENUM_WIP;
    int d = ENTRY_FLAGS_ENUM_DEPRECATED;
    int s = ENTRY_FLAGS_ENUM_SUPERSEDED;
    (void)w;
    (void)d;
    (void)s;
}

static void use_flagged_enums(void)
{
    /* these enums are flagged as a whole: their entries carry no flag of
       their own and inherit the enum's, and the type name is flagged too */
    int w = WIP_ENUM_INHERITS_WIP;
    int d = DEPRECATED_ENUM_INHERITS_DEPRECATED;
    int s = SUPERSEDED_ENUM_INHERITS_SUPERSEDED;
    int wd = WIP_ENUM_OWN_DEPRECATED;
    WIP_ENUM wt = 0;
    DEPRECATED_ENUM dt = 0;
    SUPERSEDED_ENUM st = 0;
    (void)w;
    (void)d;
    (void)s;
    (void)wd;
    (void)wt;
    (void)dt;
    (void)st;
}

static void use_extended_enums(void)
{
    /* enums extended from wip_deprecated_superseded_base.xml: an entry
       carries the flag of the declaration it came from, and the type is
       flagged if any declaration is */
    int b = EXTENDED_DEPRECATED_ENUM_FROM_FLAGGED_BASE;
    int d = EXTENDED_WIP_ENUM_FROM_FLAGGED_DIALECT;
    int bb = EXTENDED_BOTH_DEPRECATED_ENUM_FROM_BASE;
    int bd = EXTENDED_BOTH_DEPRECATED_ENUM_FROM_DIALECT;
    EXTENDED_DEPRECATED_ENUM bt = 0;
    EXTENDED_WIP_ENUM dt = 0;
    EXTENDED_BOTH_DEPRECATED_ENUM bbt = 0;
    (void)b;
    (void)d;
    (void)bb;
    (void)bd;
    (void)bt;
    (void)dt;
    (void)bbt;
}

static void use_unflagged_enum(void)
{
    /* neither of these may be diagnosed: an enum whose entries are flagged
       individually is not itself flagged, and nor are its other entries */
    ENTRY_FLAGS_ENUM e = ENTRY_FLAGS_ENUM_PLAIN;
    (void)e;
}

static void use_unflagged_entries(void)
{
    /* none of these may be diagnosed: entries from an unflagged
       declaration of an extended enum, and the generated end markers */
    int d = EXTENDED_DEPRECATED_ENUM_FROM_UNFLAGGED_DIALECT;
    int b = EXTENDED_WIP_ENUM_FROM_UNFLAGGED_BASE;
    int we = WIP_ENUM_ENUM_END;
    int de = DEPRECATED_ENUM_ENUM_END;
    int se = SUPERSEDED_ENUM_ENUM_END;
    int ede = EXTENDED_DEPRECATED_ENUM_ENUM_END;
    int ewe = EXTENDED_WIP_ENUM_ENUM_END;
    int ebe = EXTENDED_BOTH_DEPRECATED_ENUM_ENUM_END;
    (void)d;
    (void)b;
    (void)we;
    (void)de;
    (void)se;
    (void)ede;
    (void)ewe;
    (void)ebe;
}

int main(void)
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
