# Rust generator

Generate a Cargo library containing MAVSpec-compatible payload bindings and a
separate, allocation-free wire layer:

```sh
mavgen.py --lang=Rust --wire-protocol=2.0 --output=generated message_definitions/v1.0/all.xml
cargo check --manifest-path generated/Cargo.toml
```

The 2.0 output supports both MAVLink 1 and MAVLink 2. With `--wire-protocol=1.0`,
only MAVLink 1 messages and base fields are generated. MAVLink 1 cannot carry
system IDs above 255 or message IDs above 255; these are rejected, never truncated.

The crate defaults to `no_std` without allocation. Features `std` and `serde`
enable standard-library integration and serialization of generated types. The runtime uses
RustCrypto `sha2` without its default features. Rust 1.88 or newer is required. Generated bindings use
`mavspec` 0.6.7's public spec traits and derive macros.

## Compatibility with MAVSpec and mavio

The generated API follows MAVSpec's `dialects::<dialect>::messages`, `enums`,
CamelCase message structs and dialect enum variants, typed enum/bitmask fields,
`Default`, `Message`, `MessageSpec`, `MessageSpecStatic`, `IntoPayload`,
`TryFrom<&Payload>` and `Dialect` interfaces. Included messages are shared types when their enums are unchanged; extending
an enum creates a distinct type in the child dialect, along with new message
types for fields that use it. Parent dialect enums retain their original variants. The crate re-exports `mavspec` and its `spec` module.

Dialect version/ID metadata follows MAVInspect's existing inheritance rules,
including its direct-include precedence. Enum numeric values come from pymavlink's
parser, matching C; custom dialects should give entries explicit values to avoid
pymavlink's global renumbering of implicit values when merging enum extensions.

Existing mavio transports can consume these message types for ordinary IDs. The
test suite exercises the actual mavio crate, rather than a mock trait definition.
Mavio 0.5.10 itself has 8-bit source IDs and does not understand extended headers:
wide frames must use the generated `wire` layer or a separately updated mavio
transport. This generator does not replace mavio's I/O or endpoint APIs, nor
MAVSpec's optional microservice/definition/reflection generators.

## Opaque routing and signing

`wire::Frame` stores an opaque payload and 32-bit source/target IDs. Parsing and
routing require only `WireInfo` (message ID, CRC_EXTRA, lengths and target offset),
not decoding fields or constructing generated messages. IDs absent from that
metadata table are rejected because their CRC_EXTRA and routing offsets are unknown. `parse_with` and
`Parser::push_with` accept an application-supplied metadata lookup;
`parse::<D>` and `Parser::push::<D>` use a generated dialect's lookup.

```rust
use mavlink_generated::{DefaultDialect, wire::{Frame, MAX_FRAME_SIZE}};

fn route(packet: &[u8], target: u32, key: &[u8; 32], timestamp: u64)
    -> Result<([u8; MAX_FRAME_SIZE], usize), mavlink_generated::wire::Error>
{
    let mut frame = Frame::parse::<DefaultDialect>(packet)?;
    frame.verify_signature(key)?;
    frame.retarget(target)?;
    frame.sign(key, 3, timestamp)?;
    let mut output = [0; MAX_FRAME_SIZE];
    let length = frame.write(&mut output);
    Ok((output, length))
}
```

`retarget` modifies only the payload target byte and optional target header.
Targets above 255 use the 255 payload marker; 255 remains a valid ordinary ID.
Changing back to a small target removes the extended target header. A source
above 255 independently selects the wide source header. Payload layouts do not
change. Use `target_system()` to route; the decoded payload's byte alone cannot
represent a wide target. A target in an extension field is absent in MAVLink 1.
As in C, a received TARGET32 header takes precedence even if it contains a small
ID or the payload byte is not the canonical marker. Use the full-width getter
consistently; parsing does not impose stricter routing policy than the C library.

`strip_signature`, `sign` and `verify_signature` operate without typed message
decoding. Any header/target mutation removes the old signature. `write` includes
the current headers and payload in the CRC; `sign` includes the CRC and all
extended header bytes in the MAVLink 2 SHA-256 signature. The caller supplies
and persists a strictly increasing 48-bit signing timestamp (10us units since
2015-01-01). Timestamp overflow is rejected.

Parsing checks framing/CRC, **not authentication**. `verify_signature` checks the
key/hash; `ReplayGuard<N>::verify` additionally rejects replayed timestamps and
new streams more than one minute behind a supplied trusted clock. Keep one guard
per key across parser resets. Its fixed table never evicts authenticated streams;
choose sufficient capacity for the swarm's system/component/link combinations.
A full table rejects new streams instead of silently forgetting replay history.

The streaming parser consumes rejected packets as complete frames so payload
bytes cannot masquerade as new messages. A truncated frame requires the rest of
its declared length, or an explicit `reset` at a transport timeout/boundary.

## Tests

```sh
MDEF=/path/to/message_definitions python3 -m pytest -q tests/test_mavgen_rust.py
```

Requires Cargo, a C compiler, pytest, and pymavlink installed (or its parent on
`PYTHONPATH`). Every message in `all.xml`, including included test dialects, is
instantiated with zero/low and nonzero/boundary fields. C-generated frames are
compared byte-for-byte with Rust output across MAVLink 1, MAVLink 2, independent
source/target widths, broadcast/255/256/maximum IDs, and signed/unsigned frames.
Rust decodes the C payloads through the generated dialect; C parses the Rust
frames and verifies signatures and full-width routing. Additional cases exercise
opaque retargeting/re-signing, truncated/fragmented input, corrupted CRCs,
unsupported flags with embedded packets, authentication failures and replay
protection. No-std, no-std/serde, and std/serde configurations are compiled. Dialect-inheritance
tests compile an exhaustive parent-enum match and verify that a child extension
does not change parent types or accepted values.
