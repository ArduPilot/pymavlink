#!/usr/bin/env bash
# Drives test_diagnostics_positive.cpp: that file names one flagged message
# struct per category (WIP/deprecated/superseded), all three individually
# flagged enum entries, three enums flagged as a whole (an entry and the
# type name of each) and three enums extended across XML files, and must FAIL
# to build under -Werror, with every individual symbol below actually
# diagnosed - not just the build failing on the first
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
    # Clang caps itself at 20 errors by default (-ferror-limit=20). Only 18
    # symbols are checked here today, well under that cap, but the C
    # counterpart of this script, which checks more, hit exactly this with a
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
LC_ALL=C "$CXX" "$@" ${extra_flags[@]+"${extra_flags[@]}"} -o "$bin" test_diagnostics_positive.cpp >"$log" 2>&1
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
# are too many characters between "DEPRECATED" and "is deprecated" there),
# and the leading boundary stops it matching the tail of a longer name such
# as INHERITS_DEPRECATED.
for sym in \
    WIP_MESSAGE \
    DEPRECATED_MESSAGE \
    SUPERSEDED_MESSAGE \
    WIP \
    DEPRECATED \
    SUPERSEDED \
    INHERITS_WIP \
    INHERITS_DEPRECATED \
    INHERITS_SUPERSEDED \
    OWN_DEPRECATED \
    WIP_ENUM \
    DEPRECATED_ENUM \
    SUPERSEDED_ENUM \
    FROM_FLAGGED_BASE \
    FROM_FLAGGED_DIALECT \
    FROM_BASE \
    FROM_DIALECT \
    EXTENDED_BOTH_DEPRECATED_ENUM \
    ; do
    if ! grep -qE "(^|[^A-Za-z0-9_])${sym}.{0,3}is (deprecated|unavailable)" "$log"; then
        echo "FAIL: no diagnostic found for ${sym}" >&2
        missing=1
    fi
done

# an enum whose entries are flagged individually is not itself flagged, nor
# are its unflagged entries; nor are entries from an unflagged declaration
# of an extended enum, nor the type of an extended enum one of whose
# declarations is unflagged
for sym in \
    ENTRY_FLAGS_ENUM \
    PLAIN \
    FROM_UNFLAGGED_DIALECT \
    FROM_UNFLAGGED_BASE \
    EXTENDED_DEPRECATED_ENUM \
    EXTENDED_WIP_ENUM \
    ; do
    if grep -qE "(^|[^A-Za-z0-9_])${sym}.{0,3}is (deprecated|unavailable)" "$log"; then
        echo "FAIL: unexpected diagnostic for ${sym}" >&2
        missing=1
    fi
done

# The symbol checks above cannot see a diagnostic that names a different
# symbol: an enum entry is always written qualified by its type
# (E::ENTRY), and a flag on the type is reported against the type's name,
# not the entry's. So also check each statement in
# test_diagnostics_positive.cpp marked "// diagnosed" or "// not diagnosed"
# by its line number.
diagnosed_lines=" $(grep -oE "^test_diagnostics_positive\.cpp:[0-9]+:[0-9]+: (error|warning)" "$log" | cut -d: -f2 | sort -un | tr '\n' ' ')"
while IFS=: read -r line text; do
    case "$text" in
    *"// not diagnosed"*)
        if [[ "$diagnosed_lines" == *" $line "* ]]; then
            echo "FAIL: unexpected diagnostic on line ${line}:${text}" >&2
            missing=1
        fi
        ;;
    *)
        if [[ "$diagnosed_lines" != *" $line "* ]]; then
            echo "FAIL: no diagnostic on line ${line}:${text}" >&2
            missing=1
        fi
        ;;
    esac
done < <(grep -nE "// (not )?diagnosed[[:space:]]*$" test_diagnostics_positive.cpp)

if [ "$missing" -ne 0 ]; then
    echo "--- compiler output ---" >&2
    cat "$log" >&2
    exit 1
fi

echo "OK: all 18 expected diagnostics fired ($(grep -Ec "is (deprecated|unavailable)" "$log") diagnostic lines total)"
