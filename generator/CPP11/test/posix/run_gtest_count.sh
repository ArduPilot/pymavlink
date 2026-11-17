#!/usr/bin/env bash
# Runs a gtestsuite.hpp test binary and checks that it ran exactly the
# tests the generated header defines: all of them, or with "skipwip" all
# but those inside the MAVLINK_TESTSUITE_SKIP_WIP guards, of which there
# must be at least one. A guard that hides tests it should not, or that
# the generator stops emitting, then fails here even where the build
# itself would still pass (e.g. deprecated() MAVLINK_MSG_TYPE_WIP on
# GCC < 12, see test_gtestsuite_diagnostics_skipwip.cpp).
set -u

if [ "$#" -ne 3 ] || { [ "$2" != all ] && [ "$2" != skipwip ]; }; then
    echo "usage: $0 <gtestsuite.hpp> all|skipwip <test binary>" >&2
    exit 2
fi
header=$1
mode=$2
binary=$3

read -r total skipped < <(awk '
    /^#ifndef MAVLINK_TESTSUITE_SKIP_WIP$/ { in_skip = 1; next }
    /^#endif \/\/ MAVLINK_TESTSUITE_SKIP_WIP$/ { in_skip = 0; next }
    /^TEST\(/ { total++; if (in_skip) skipped++ }
    END { print total + 0, skipped + 0 }' "$header")

expected=$total
if [ "$mode" = skipwip ]; then
    if [ "$skipped" -eq 0 ]; then
        echo "FAIL: $header has no tests inside MAVLINK_TESTSUITE_SKIP_WIP guards" >&2
        exit 1
    fi
    expected=$((total - skipped))
fi

log=$(mktemp)
trap 'rm -f "$log"' EXIT

"./$binary" >"$log" 2>&1
rc=$?
cat "$log"
if [ "$rc" -ne 0 ]; then
    exit "$rc"
fi

passed=$(sed -nE 's/^\[  PASSED  \] ([0-9]+) tests?\.$/\1/p' "$log")
if [ "$passed" != "$expected" ]; then
    echo "FAIL: $binary passed ${passed:-no} tests, expected $expected" \
         "($total in $header, $skipped of them WIP, mode $mode)" >&2
    exit 1
fi
echo "OK: $binary ran all $expected expected tests"
