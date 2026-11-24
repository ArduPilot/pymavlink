#!/usr/bin/env bash
set -euo pipefail

alr -n build

for executable in build/*/tests/obj/test; do
    printf 'Running %s\n' "$executable"
    "$executable"
done

command -v gnatmake > /dev/null
python3 -m pytest -q -rs ../test_sysid32_generators.py -k ada
