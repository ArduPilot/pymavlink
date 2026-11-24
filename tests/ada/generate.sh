#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

definitions=../snapshottests/resources
if [[ -n ${MDEF:-} ]]; then
    definitions="$MDEF/v1.0"
fi

for protocol in 1.0 2.0; do
    for dialect in minimal common; do
        output="build/$dialect-$protocol"
        rm -rf -- "$output"
        python3 -m pymavlink.tools.mavgen --lang Ada --wire-protocol "$protocol" \
            --output "$output" "$definitions/$dialect.xml"
    done
done
