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
metadata table are rejected by this API because their CRC_EXTRA and routing offsets
are unknown. For future message IDs, use the explicit raw forwarding policy below. `parse_with` and
`StreamParser::push_with` accept an application-supplied metadata lookup;
`parse::<D>` and `StreamParser::push::<D>` use a generated dialect's lookup.

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

### Future messages and enum values

`ReceivedFrame::parse::<D>(bytes, UnknownMessages::Preserve)` returns either
`Known(Frame)` with a validated CRC, or `Unknown(RawFrame)` with an **unchecked**
CRC. The default policy is `Reject`. A known message with a bad CRC is rejected
under both policies; unsupported incompatibility flags are always rejected.
`parse_with` accepts an application metadata lookup, including definitions learned
after generation.

`RawFrame` is immutable and forwards the exact original bytes, including CRC,
signature, noncanonical header widths and payload extensions. Without CRC_EXTRA,
even stripping a signature would require a checksum change that cannot be computed
safely, so raw frames have no editing API. `validate_with` resolves metadata later
and returns a CRC-checked `Frame`. `target_system()` exposes TARGET32 when present;
otherwise it returns `None` because the payload routing offset is unknown. The
application must choose how to forward traffic whose target is unknown.

`crc_status()` explicitly reports `UnknownExtra` for unknown messages. Their
signatures can still be authenticated with `RawFrame::verify_signature` and
`ReplayGuard::verify_raw`, using the same replay table as known frames. Successful
authentication does not change CRC status or establish payload schema validity.
Unsigned unknown packets are only length-delimited, so accepting them is an
explicit application policy, not an integrity guarantee.

Opaque forwarding preserves future enum values, bitmask bits and extension fields.
The MAVSpec-compatible `Frame::decode` API rejects unknown enums, but MAVSpec can
silently discard unknown bitmask bits or values wider than the generated enum's
representation. For safe typed use, `decode_with::<D>(DecodePolicy::Strict)` also
re-encodes the result and compares zero-extended payload bytes. It reports an error
if information would be lost, including nonzero future extension bytes and a
TARGET32 header whose target differs from the payload target byte. With
`DecodePolicy::PreserveRaw`, the same failure returns `Decoded::Raw(&Frame)` rather
than a lossy typed value. Harmless trailing zeros compare equal; NaN bit patterns
are compared as bytes. The original frame remains authoritative for forwarding,
even when decoding succeeds, since payload re-encoding may change padding.

### Stream recovery

Use `StreamParser` for streaming input. It defaults to C-style recovery
(`Recovery::Recover`) and rejects unknown message IDs (`UnknownMessages::Reject`).
Select `UnknownMessages::Preserve` to forward future messages without decoding
them, as in this example:

```rust
use mavlink_generated::{DefaultDialect, wire::{ParserOptions, StreamParser,
    UnknownMessages, MAX_FRAME_SIZE}};

let mut parser = StreamParser::new(ParserOptions {
    unknown_messages: UnknownMessages::Preserve,
    ..Default::default()
});
let input: &[u8] = &[]; // Bytes received from the transport.
parser.feed::<DefaultDialect>(input, |event| {
    if let Ok(frame) = event {
        let mut output = [0; MAX_FRAME_SIZE];
        let length = frame.write(&mut output);
        // Apply routing/authentication policy before sending output[..length].
        let _ = length;
    }
});
```

`Recover` uses the C `mavlink_parse_char` strategy: track one candidate at a
time, reject unsupported MAVLink 2 flags as soon as they arrive, and reset after
a CRC failure. Only the byte completing a failed CRC can be reused, and only if
it is the MAVLink 2 start byte (`0xFD`). The parser never scans buffered header or
payload bytes for embedded packets. This applies both to corrupt frames and to
intact frames following an error, so recovery does not promote logged packets
from an intact tlog-transfer payload into the outer stream. A corrupted length
may delay recovery until the declared payload and CRC arrive; applications can
call `reset` at a known transport boundary or timeout.

As in C's convenience parser, a signed frame with a bad CRC is rejected at CRC2,
without waiting for the signature; its remaining signature bytes are then input
to the idle parser. A valid signed frame waits for all 13 signature bytes.
Signature authentication and replay protection remain explicit application steps.
Unknown IDs in `Reject` mode follow the same reset/reuse rule as a bad CRC,
because no CRC_EXTRA is available. `Recovery::Conservative` instead consumes the
whole declared packet, including the signature, before any rejection. The simpler
`Parser` API also consumes whole packets and always rejects unknown message IDs.

`Preserve` is an explicit extension to C's known-message parser. Unknown frames
are accepted as unchecked, length-delimited packets, including after an error or
reset. Noise can therefore look like an unknown packet and swallow subsequent
traffic up to its declared length. Use `crc_status()` and authentication/routing
policy before trusting or forwarding them. Resetting either parser must not reset
replay protection.

The recovery strategy follows C, with stricter validation for known messages:
MAVLink 1 requires its fixed message length, MAVLink 2 rejects empty payloads, and
invalid local metadata is rejected. The C default build does not impose those
length checks. Payload length rejections at CRC2 use the same `0xFD` restart rule
as a bad CRC. Reusing `0xFD` always starts MAVLink 2 here, including after a
failed MAVLink 1 frame; it does not retain C's stale MAVLink 1 state in that case.

Storage is fixed at 287 wire bytes plus parser state. Each known candidate has
at most one metadata lookup and one CRC check, with no scanning of alternate starts.
`push` returns at most one event; `feed`/`feed_with` deliver all events from a
slice. No separate draining is needed: `drain_next`/`drain_next_with` return `None`
after a push or feed. The metadata lookup is `FnMut`, consistent throughout a
packet's reception.

### Protocol constants

`Heartbeat::new()` sets `mavlink_version` to the exported
`messages::heartbeat::MAVLINK_VERSION` constant (3). `Frame::from_message` also
fills XML protocol constants when originating a typed message, including one
created with `Heartbeat::default()`, matching the C encoder. Received opaque
frames and `from_payload` are never normalized. Forward through `write`/`retarget`
instead of decoding and reconstructing a typed outgoing message.

MAVSpec's derive owns `Default` and still zeroes scalar fields. Code using the
MAVSpec `encode` traits or mavio directly should construct `Heartbeat::new()`;
the upstream trait implementation cannot be given a per-field default here.

## Tests

```sh
PYMAVLINK_TEST_RUST=1 MDEF=/path/to/message_definitions python3 -m pytest -q tests/test_mavgen_rust.py
```

Cargo-dependent tests are opt-in so they run only in the dedicated Rust CI job,
not each Python matrix job. Cheap Python generator checks still run normally.
When opted in, missing Cargo or C compiler is a test failure.
Requires Cargo, a C compiler, pytest, and pymavlink installed (or its parent on
`PYTHONPATH`). Every message in `all.xml`, including included test dialects, is
instantiated with zero/low and nonzero/boundary fields. C-generated frames are
compared byte-for-byte with Rust output across MAVLink 1, MAVLink 2, independent
source/target widths, broadcast/255/256/maximum IDs, and signed/unsigned frames.
Rust decodes the C payloads through the generated dialect; C parses the Rust
frames and verifies signatures and full-width routing. Additional cases exercise
opaque retargeting/re-signing, truncated/fragmented input, corrupted CRCs,
unsupported flags with embedded packets, authentication failures and replay
protection, unknown-message forwarding, and lossless decoding. Stream recovery
tests compare exact packets and their delivery byte positions with the C parser,
including nested packets in SERIAL_CONTROL and FTP log transfers, CRC2 restart
bytes, and signed-frame boundaries. No-std, no-std/serde, and std/serde
configurations are compiled. Dialect-inheritance tests compile an exhaustive
parent-enum match and verify that a child extension does not change parent types
or accepted values.
