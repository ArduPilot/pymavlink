#!/usr/bin/env bash
# Drives test_diagnostics_positive.c: that file calls every wrapper of three
# flagged messages (one per WIP/deprecated/superseded) plus three flagged
# enum entries, and must FAIL to build under -Werror, with every individual
# symbol below actually diagnosed - not just the build failing on the first
# one found, which would hide a regression that drops the attribute from
# just one wrapper (as happened previously with _pack_status/_encode_status),
# or that guts an entire category (as happened with WIP, which an earlier
# version of this file didn't exercise at all).
set -u

if [ "$#" -lt 2 ]; then
    echo "usage: $0 <CC> <CFLAGS...>" >&2
    exit 2
fi
CC=$1
shift

log=$(mktemp)
bin=$(mktemp)
trap 'rm -f "$log" "$bin"' EXIT

# Force the C locale for the compiler's own diagnostic text: this script
# greps for the English "is deprecated"/"is unavailable" wording, which a
# localised gcc/clang would translate, turning a working feature into a
# wall of spurious FAILs.
LC_ALL=C "$CC" "$@" -o "$bin" test_diagnostics_positive.c >"$log" 2>&1
rc=$?

if [ "$rc" -eq 0 ]; then
    echo "FAIL: test_diagnostics_positive.c built cleanly under -Werror;" >&2
    echo "      real calls to deprecated/superseded symbols should have been diagnosed." >&2
    cat "$log" >&2
    exit 1
fi

missing=0
for sym in \
    mavlink_msg_wip_message_pack \
    mavlink_msg_wip_message_pack_chan \
    mavlink_msg_wip_message_pack_status \
    mavlink_msg_wip_message_encode \
    mavlink_msg_wip_message_encode_chan \
    mavlink_msg_wip_message_encode_status \
    mavlink_msg_wip_message_decode \
    mavlink_msg_wip_message_get_value \
    mavlink_msg_deprecated_message_pack \
    mavlink_msg_deprecated_message_pack_chan \
    mavlink_msg_deprecated_message_pack_status \
    mavlink_msg_deprecated_message_encode \
    mavlink_msg_deprecated_message_encode_chan \
    mavlink_msg_deprecated_message_encode_status \
    mavlink_msg_deprecated_message_decode \
    mavlink_msg_deprecated_message_get_value \
    mavlink_msg_superseded_message_pack \
    mavlink_msg_superseded_message_pack_chan \
    mavlink_msg_superseded_message_pack_status \
    mavlink_msg_superseded_message_encode \
    mavlink_msg_superseded_message_encode_chan \
    mavlink_msg_superseded_message_encode_status \
    mavlink_msg_superseded_message_decode \
    mavlink_msg_superseded_message_get_value \
    ENTRY_FLAGS_ENUM_WIP \
    ENTRY_FLAGS_ENUM_DEPRECATED \
    ENTRY_FLAGS_ENUM_SUPERSEDED \
    ; do
    # GCC/clang wrap the symbol name in quotes before "is deprecated"/"is
    # unavailable" (ENTRY_FLAGS_ENUM_WIP is unavailable() on a toolchain
    # that supports it, deprecated() otherwise - see test_diagnostics_positive.c),
    # but which quote characters (ASCII ' or Unicode fancy quotes) depends
    # on the compiler and locale, so match loosely rather than pin the
    # exact quote glyphs.
    if ! grep -qE "${sym}.{0,3}is (deprecated|unavailable)" "$log"; then
        echo "FAIL: no diagnostic found for ${sym}" >&2
        missing=1
    fi
done

if [ "$missing" -ne 0 ]; then
    echo "--- compiler output ---" >&2
    cat "$log" >&2
    exit 1
fi

echo "OK: all 27 expected diagnostics fired ($(grep -Ec "is (deprecated|unavailable)" "$log") diagnostic lines total)"
