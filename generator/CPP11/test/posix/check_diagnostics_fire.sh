#!/usr/bin/env bash
# Drives test_diagnostics_positive.cpp: that file names one flagged message
# struct per category (WIP/deprecated/superseded) plus all three flagged
# enum entries, and must FAIL to build under -Werror, with every individual
# symbol below actually diagnosed - not just the build failing on the first
# one found, which would hide a regression that guts just one category (as
# happened with the C++11 struct attribute, and separately with WIP, which
# an earlier version of this file didn't exercise at all).
set -u

if [ "$#" -lt 2 ]; then
    echo "usage: $0 <CXX> <CXXFLAGS...>" >&2
    exit 2
fi
CXX=$1
shift

extra_flags=()
if "$CXX" --version 2>/dev/null | grep -qi clang; then
    # Clang caps itself at 20 errors by default (-ferror-limit=20). Only 6
    # symbols are checked here today, well under that cap, but the C
    # counterpart of this script (27 symbols) hit exactly this with a
    # plain `cc` that turned out to be clang (e.g. macOS) - matching the
    # fix here too so this file doesn't quietly grow into the same trap.
    extra_flags+=(-ferror-limit=0)
fi

log=$(mktemp)
bin=$(mktemp)
trap 'rm -f "$log" "$bin"' EXIT

# Force the C locale for the compiler's own diagnostic text: this script
# greps for the English "is deprecated"/"is unavailable" wording, which a
# localised g++/clang++ would translate, turning a working feature into a
# wall of spurious FAILs.
LC_ALL=C "$CXX" "$@" "${extra_flags[@]}" -o "$bin" test_diagnostics_positive.cpp >"$log" 2>&1
rc=$?

if [ "$rc" -eq 0 ]; then
    echo "FAIL: test_diagnostics_positive.cpp built cleanly under -Werror;" >&2
    echo "      real use of a flagged struct/enum entry should have been diagnosed." >&2
    cat "$log" >&2
    exit 1
fi

missing=0
# Enum entries are matched by their short (unqualified) name: GCC reports
# them fully qualified (mavlink::<dialect>::ENTRY_FLAGS_ENUM::WIP), clang
# reports just WIP, but the tight .{0,3} window below means the short name
# still can't cross-match e.g. DEPRECATED_MESSAGE's own diagnostic (there
# are too many characters between "DEPRECATED" and "is deprecated" there).
for sym in \
    WIP_MESSAGE \
    DEPRECATED_MESSAGE \
    SUPERSEDED_MESSAGE \
    WIP \
    DEPRECATED \
    SUPERSEDED \
    ; do
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

echo "OK: all 6 expected diagnostics fired ($(grep -Ec "is (deprecated|unavailable)" "$log") diagnostic lines total)"
