use mavlink_generated::{
    messages::{
        AutopilotVersion, CommandLong, FileTransferProtocol, Heartbeat, SerialControl, SetMode,
    },
    spec::{MavLinkVersion as V, Payload},
    wire::*,
    DefaultDialect as All,
};

fn bytes(frame: &Frame) -> Vec<u8> {
    let mut output = [0; MAX_FRAME_SIZE];
    let n = frame.write(&mut output);
    output[..n].to_vec()
}
fn forwarded(frame: &ReceivedFrame) -> Vec<u8> {
    let mut output = [0; MAX_FRAME_SIZE];
    let n = frame.write(&mut output);
    output[..n].to_vec()
}
// Compare packet identity and delivery byte, not just counts: payloads may contain
// complete, CRC-valid packets which must never replace their intact outer frame.
fn c_frames(stream: &[u8]) -> Vec<(usize, Vec<u8>)> {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let path = format!(
        "{}.{}",
        std::env::var("CORRUPTION_STREAM").unwrap(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    );
    std::fs::write(&path, stream).unwrap();
    let output = std::process::Command::new(std::env::var("C_ORACLE").unwrap())
        .args(["--recover", &path])
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(output.status.success());
    std::str::from_utf8(&output.stdout)
        .unwrap()
        .lines()
        .filter(|s| !s.is_empty())
        .map(|line| {
            let (offset, hex) = line.split_once(' ').unwrap();
            let packet = (0..hex.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
                .collect();
            (offset.parse().unwrap(), packet)
        })
        .collect()
}
fn assert_c_recovery(stream: &[u8], chunks: &[usize]) -> Vec<Vec<u8>> {
    let reference = c_frames(stream);
    let mut parser = StreamParser::default();
    let mut received = Vec::new();
    for (offset, byte) in stream.iter().enumerate() {
        if let Some(Ok(frame)) = parser.push::<All>(*byte) {
            received.push((offset + 1, forwarded(&frame)));
        }
        assert!(parser.drain_next::<All>().is_none());
    }
    assert_eq!(
        received, reference,
        "delivery offsets/packets differ from C"
    );
    let expected: Vec<_> = reference.into_iter().map(|(_, frame)| frame).collect();
    for &chunk in chunks {
        let mut parser = StreamParser::default();
        let mut received = Vec::new();
        for part in stream.chunks(chunk) {
            parser.feed::<All>(part, |event| {
                if let Ok(frame) = event {
                    received.push(forwarded(&frame));
                }
            });
        }
        assert_eq!(received, expected, "chunk size {chunk}");
    }
    expected
}
fn heartbeat(version: V, wide: bool, signed: bool) -> Frame {
    let mut frame = Frame::from_message(
        version,
        239,
        if wide { u32::MAX } else { 42 },
        11,
        &Heartbeat::default(),
    )
    .unwrap();
    if signed {
        frame.sign(&[42; 32], 3, 1000).unwrap();
    }
    frame
}

#[test]
fn heartbeat_constants_match_c_without_rewriting_received_payloads() {
    assert_eq!(mavlink_generated::messages::heartbeat::MAVLINK_VERSION, 3);
    assert_eq!(Heartbeat::new().mavlink_version, 3);
    let c = std::process::Command::new(std::env::var("C_ORACLE").unwrap())
        .arg("--heartbeat")
        .output()
        .unwrap();
    assert!(c.status.success());
    let fixtures = std::str::from_utf8(&c.stdout).unwrap();
    for version in [V::V1, V::V2] {
        let frame = heartbeat(version, false, false);
        assert_eq!(frame.payload()[8], 3);
        // C fixtures include the default heartbeat fields and always force version 3.
        let expected = fixtures
            .lines()
            .find(|line| {
                line.starts_with(if version == V::V1 {
                    "0 0 1 42 0 0 "
                } else {
                    "0 0 2 42 0 0 "
                })
            })
            .unwrap()
            .split_whitespace()
            .last()
            .unwrap();
        let actual: String = bytes(&frame).iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(actual, expected);
        let payload = Payload::new(0, &[0; 9], version);
        let received = Frame::from_payload(239, 42, 11, &payload, Heartbeat::WIRE_INFO).unwrap();
        let forwarded = Frame::parse::<All>(&bytes(&received)).unwrap();
        assert_eq!(forwarded.to_payload().bytes(), payload.bytes());
        assert!(matches!(
            forwarded.decode_with::<All>(DecodePolicy::Strict),
            Ok(Decoded::Known(_))
        ));
    }
}

#[test]
fn unknown_messages_preserve_wire_bytes_without_claiming_crc_validation() {
    for version in [V::V1, V::V2] {
        for wide in [false, true] {
            for signed in [false, true] {
                if version == V::V1 && (wide || signed) {
                    continue;
                }
                let info = WireInfo {
                    id: if version == V::V1 { 201 } else { 6_000_000 },
                    crc_extra: 117,
                    min_length: 9,
                    length: 9,
                    target_offset: Some(0),
                };
                let mut frame = Frame::from_payload(
                    9,
                    if wide { u32::MAX } else { 42 },
                    11,
                    &Payload::new(info.id, &[255, 7, 3, 4, 5, 6, 7, 8, 9], version),
                    info,
                )
                .unwrap();
                frame.retarget(if wide { 100_000 } else { 7 }).unwrap();
                if signed {
                    frame.sign(&[42; 32], 3, 1000).unwrap();
                }
                let wire = bytes(&frame);
                assert!(matches!(
                    ReceivedFrame::parse_with(&wire, |_| None, UnknownMessages::Reject),
                    Err(Error::UnknownMessage(_))
                ));
                let received =
                    ReceivedFrame::parse_with(&wire, |_| None, UnknownMessages::Preserve).unwrap();
                assert_eq!(received.crc_status(), CrcStatus::UnknownExtra);
                assert_eq!(forwarded(&received), wire);
                let ReceivedFrame::Unknown(raw) = received else {
                    panic!("unknown metadata");
                };
                assert_eq!(raw.version(), version);
                assert_eq!(raw.sequence(), 9);
                assert_eq!(raw.system_id(), frame.system_id());
                assert_eq!(raw.component_id(), 11);
                assert_eq!(raw.message_id(), info.id);
                assert_eq!(raw.target_system(), if wide { Some(100_000) } else { None });
                assert_eq!(raw.payload(), frame.payload());
                assert_eq!(raw.as_bytes(), wire);
                let resolved = raw.validate_with(|_| Some(info)).unwrap();
                assert_eq!(bytes(&resolved), wire);
                if signed {
                    raw.verify_signature(&[42; 32]).unwrap();
                    assert!(matches!(
                        raw.verify_signature(&[43; 32]),
                        Err(Error::BadSignature)
                    ));
                    let mut replay = ReplayGuard::<1>::default();
                    assert!(replay.verify_raw(&raw, &[43; 32], 1000).is_err());
                    replay.verify_raw(&raw, &[42; 32], 1000).unwrap();
                    // Raw and known representations use the same replay namespace.
                    assert!(matches!(
                        replay.verify(&resolved, &[42; 32], 1000),
                        Err(Error::Replay)
                    ));
                    let mut tampered = wire.clone();
                    *tampered.last_mut().unwrap() ^= 1;
                    assert!(matches!(
                        RawFrame::parse(&tampered)
                            .unwrap()
                            .verify_signature(&[42; 32]),
                        Err(Error::BadSignature)
                    ));
                } else {
                    assert!(matches!(
                        raw.verify_signature(&[42; 32]),
                        Err(Error::Unsigned)
                    ));
                }
                let mut corrupt = wire.clone();
                let end = corrupt.len() - if signed { 15 } else { 2 };
                corrupt[end] ^= 1;
                assert!(matches!(
                    ReceivedFrame::parse_with(&corrupt, |_| Some(info), UnknownMessages::Preserve),
                    Err(Error::BadCrc)
                ));
                if version == V::V2 {
                    corrupt[2] |= 0x80;
                    assert!(matches!(
                        ReceivedFrame::parse_with(&corrupt, |_| None, UnknownMessages::Preserve),
                        Err(Error::UnsupportedFlags(_))
                    ));
                }
                for cut in 0..wire.len() {
                    assert!(RawFrame::parse(&wire[..cut]).is_err());
                }
            }
        }
    }
}

fn assert_raw_fallback(frame: &Frame) {
    assert!(frame.decode_with::<All>(DecodePolicy::Strict).is_err());
    let Decoded::Raw(raw) = frame.decode_with::<All>(DecodePolicy::PreserveRaw).unwrap() else {
        panic!("lossy typed decode accepted");
    };
    assert!(std::ptr::eq(raw, frame));
    assert_eq!(bytes(raw), bytes(frame));
}
#[test]
fn lossless_decode_catches_unknown_enums_masks_and_extensions() {
    let mut payload = [0; 9];
    payload[4] = 250; // Future MAV_TYPE.
    payload[8] = 3;
    let frame = Frame::from_payload(
        0,
        42,
        11,
        &Payload::new(0, &payload, V::V2),
        Heartbeat::WIRE_INFO,
    )
    .unwrap();
    assert!(frame.decode::<All>().is_err());
    assert_raw_fallback(&frame);
    // Unknown bit within the enum representation, and bit above its repr width.
    for bit in [31, 63] {
        let info = AutopilotVersion::WIRE_INFO;
        let mut payload = vec![0; info.length as usize];
        payload[..8].copy_from_slice(&(1u64 << bit).to_le_bytes());
        let frame =
            Frame::from_payload(0, 42, 11, &Payload::new(info.id, &payload, V::V2), info).unwrap();
        assert!(frame.decode::<All>().is_ok()); // MAVSpec silently truncates the mask.
        assert_raw_fallback(&frame);
    }
    let frame = Frame::from_message(V::V2, 0, 42, 11, &CommandLong::default()).unwrap();
    let mut payload = frame.payload().to_vec();
    payload.resize(CommandLong::WIRE_INFO.length as usize + 2, 0);
    *payload.last_mut().unwrap() = 1; // Future extension field.
    let frame = Frame::from_payload(
        0,
        42,
        11,
        &Payload::new(frame.message_id(), &payload, V::V2),
        CommandLong::WIRE_INFO,
    )
    .unwrap();
    assert_raw_fallback(&frame);
    *payload.last_mut().unwrap() = 0;
    // Check exact NaN bits, signed zero, and harmless trailing zero extension bytes.
    for bits in [0x7fc00001u32, 0xffc12345, 0x80000000, 0] {
        payload[..4].copy_from_slice(&bits.to_le_bytes());
        let frame = Frame::from_payload(
            0,
            42,
            11,
            &Payload::new(frame.message_id(), &payload, V::V2),
            CommandLong::WIRE_INFO,
        )
        .unwrap();
        assert!(matches!(
            frame.decode_with::<All>(DecodePolicy::Strict),
            Ok(Decoded::Known(_))
        ));
    }
}

#[test]
fn recovery_does_not_replay_corrupt_payloads() {
    let inner = bytes(&heartbeat(V::V2, false, false));
    let mut serial = SerialControl::default();
    serial.count = 70;
    for (i, byte) in inner.iter().cycle().take(70).enumerate() {
        serial.data[i] = *byte;
    }
    let outer = bytes(&Frame::from_message(V::V2, 0, 42, 11, &serial).unwrap());
    for split in 0..=outer.len() {
        let mut parser = StreamParser::default();
        let mut received = Vec::new();
        parser.feed::<All>(&outer[..split], |r| received.push(r));
        parser.feed::<All>(&outer[split..], |r| received.push(r));
        assert_eq!(received.len(), 1);
        assert_eq!(forwarded(received[0].as_ref().unwrap()), outer);
    }
    let mut bad = outer.clone();
    // Keep the final CRC byte away from the C parser's FD restart case.
    *bad.last_mut().unwrap() = if outer[outer.len() - 1] == 0 { 1 } else { 0 };
    for recovery in [Recovery::Recover, Recovery::Conservative] {
        let mut parser = StreamParser::new(ParserOptions {
            recovery,
            ..Default::default()
        });
        let mut received = Vec::new();
        parser.feed::<All>(&bad, |r| received.push(r));
        assert_eq!(received.len(), 1);
        assert!(matches!(received[0], Err(Error::BadCrc)));
        parser.feed::<All>(&outer, |r| received.push(r));
        assert_eq!(received.len(), 2);
        assert_eq!(forwarded(received[1].as_ref().unwrap()), outer);
    }
}

#[test]
fn nested_frames_after_corruption_match_c() {
    for version in [V::V1, V::V2] {
        for wide in [false, true] {
            for signed in [false, true] {
                if version == V::V1 && (wide || signed) {
                    continue;
                }
                let good = bytes(&heartbeat(version, wide, signed));
                let mut bad = good.clone();
                let checksum = bad.len() - if signed { 15 } else { 2 };
                bad[checksum] ^= 1;
                let embedded =
                    bytes(&Frame::from_message(V::V2, 7, 255, 11, &SetMode::default()).unwrap());
                let mut serial = SerialControl::default();
                serial.count = 70;
                serial.data[..embedded.len()].copy_from_slice(&embedded);
                serial.data[69] = 1;
                let mut outer =
                    Frame::from_message(version, 8, if wide { u32::MAX } else { 42 }, 11, &serial)
                        .unwrap();
                if signed {
                    outer.sign(&[42; 32], 3, 1000).unwrap();
                }
                let outer = bytes(&outer);
                let stream = [bad, outer.clone(), good.clone()].concat();
                let chunks: Vec<_> = (1..=stream.len()).collect();
                assert_eq!(assert_c_recovery(&stream, &chunks), vec![outer, good]);
            }
        }
    }
}

#[test]
fn ftp_log_bursts_do_not_promote_logged_commands() {
    for seed in 0..8u32 {
        let mut state = seed + 1;
        let mut log = Vec::new();
        while log.len() < 38 * 239 {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            let mut frame = match state % 3 {
                0 => Frame::from_message(V::V2, state as u8, 255, 11, &CommandLong::default())
                    .unwrap(),
                1 => Frame::from_message(V::V2, state as u8, 42, 11, &Heartbeat::new()).unwrap(),
                _ => Frame::from_message(V::V2, state as u8, 255, 11, &SetMode::default()).unwrap(),
            };
            if state & 8 != 0 {
                frame.sign(&[42; 32], 3, state as u64).unwrap();
            }
            log.extend(bytes(&frame));
        }
        let version = if seed == 0 { V::V1 } else { V::V2 };
        let wide = seed & 2 != 0;
        let signed = seed & 1 != 0;
        let mut stream = Vec::new();
        let mut expected = Vec::new();
        for i in 0..38 {
            let mut ftp = FileTransferProtocol::default();
            ftp.payload[0..2].copy_from_slice(&(i as u16).to_le_bytes());
            ftp.payload[3] = 128; // FTP ACK
            ftp.payload[4] = 239;
            ftp.payload[5] = 5; // ReadFile
            ftp.payload[8..12].copy_from_slice(&((i * 239) as u32).to_le_bytes());
            ftp.payload[12..].copy_from_slice(&log[i * 239..(i + 1) * 239]);
            let mut frame =
                Frame::from_message(version, i as u8, if wide { u32::MAX } else { 42 }, 11, &ftp)
                    .unwrap();
            frame.retarget(if wide { 100_000 } else { 255 }).unwrap();
            if signed {
                frame.sign(&[42; 32], 3, 1000 + i as u64).unwrap();
            }
            let mut packet = bytes(&frame);
            if i == 2 {
                let payload_start =
                    packet.len() - frame.payload().len() - if signed { 15 } else { 2 };
                packet[payload_start + 30] ^= 1;
            } else {
                expected.push(packet.clone());
            }
            stream.extend(packet);
        }
        assert_eq!(
            assert_c_recovery(&stream, &[1, 2, 7, 17, 64, 239, 287, stream.len()]),
            expected,
            "seed {seed}"
        );
    }
}

#[test]
fn unknown_stream_policy_and_recovery_boundaries() {
    let known = bytes(&heartbeat(V::V2, true, true));
    let info = WireInfo {
        id: 6_000_000,
        crc_extra: 117,
        min_length: 9,
        length: 9,
        target_offset: None,
    };
    let unknown = bytes(
        &Frame::from_payload(0, 42, 11, &Payload::new(info.id, &[0xa5; 9], V::V2), info).unwrap(),
    );
    let options = ParserOptions {
        unknown_messages: UnknownMessages::Preserve,
        ..Default::default()
    };
    let mut parser = StreamParser::new(options);
    let mut results = Vec::new();
    parser.feed::<All>(&unknown, |r| results.push(r));
    assert_eq!(
        results[0].as_ref().unwrap().crc_status(),
        CrcStatus::UnknownExtra
    );
    results.clear();
    parser.feed::<All>(&[253, 255, 128], |r| results.push(r));
    assert!(matches!(
        results.pop().unwrap(),
        Err(Error::UnsupportedFlags(128))
    ));
    parser.feed::<All>(&unknown, |r| results.push(r));
    assert_eq!(results.len(), 1);
    assert_eq!(
        results[0].as_ref().unwrap().crc_status(),
        CrcStatus::UnknownExtra
    );
    results.clear();
    parser.feed::<All>(&known, |r| results.push(r));
    parser.feed::<All>(&unknown, |r| results.push(r));
    assert_eq!(results.len(), 2);
    assert_eq!(
        results[0].as_ref().unwrap().crc_status(),
        CrcStatus::Validated
    );
    assert_eq!(
        results[1].as_ref().unwrap().crc_status(),
        CrcStatus::UnknownExtra
    );
    parser.reset();
    results.clear();
    parser.feed::<All>(&unknown, |r| results.push(r));
    assert_eq!(results.len(), 1);
    let mut strict = StreamParser::default();
    results.clear();
    strict.feed::<All>(&unknown, |r| results.push(r));
    assert_eq!(results.len(), 1);
    assert!(matches!(results[0], Err(Error::UnknownMessage(_))));
}

#[test]
fn corruption_recovery_compared_with_c() {
    for version in [V::V1, V::V2] {
        for wide in [false, true] {
            for signed in [false, true] {
                if version == V::V1 && (wide || signed) {
                    continue;
                }
                let mut template = heartbeat(version, wide, signed);
                if wide {
                    template =
                        Frame::from_message(V::V2, 239, u32::MAX, 11, &CommandLong::default())
                            .unwrap();
                    template.retarget(100_000).unwrap();
                    if signed {
                        template.sign(&[42; 32], 3, 1000).unwrap();
                    }
                }
                let good = bytes(&template);
                let mut long = good.clone();
                long[1] = 255;
                let mut one = good.clone();
                one[1] += 1;
                let mut crc = good.clone();
                let checksum = crc.len() - if signed { 15 } else { 2 };
                crc[checksum] ^= 1;
                let mut missing = good.clone();
                missing.remove(checksum + 1);
                for (name, mut stream) in [
                    ("unsupported_flags", vec![253, 255, 128]),
                    ("length_plus_one", one),
                    ("missing_crc_byte", missing),
                    ("length_255", long),
                    ("bad_crc", crc),
                    ("noise", vec![0, 12, 15, 0xfd, 1, 0x80, 0xfe]),
                ] {
                    for sequence in 0..20 {
                        let mut next = template.clone();
                        next.set_sequence(sequence);
                        if signed {
                            next.sign(&[42; 32], 3, 1001 + sequence as u64).unwrap();
                        }
                        stream.extend(bytes(&next));
                    }
                    println!("{version:?} wide={wide} signed={signed} {name}");
                    let frames = assert_c_recovery(&stream, &[1, 7, 31, stream.len()]);
                    assert!(!frames.is_empty());
                    if name == "unsupported_flags" {
                        assert_eq!(frames.len(), 20);
                    }
                }
            }
        }
    }
}

#[test]
fn restart_bytes_and_signed_crc_boundaries_match_c() {
    let good = bytes(&heartbeat(V::V2, false, false));
    for marker in [253, 254] {
        let mut bad = good.clone();
        let n = bad.len();
        bad[n - 2] ^= 1;
        bad[n - 1] = marker;
        let stream = [bad, good[1..].to_vec(), good.clone()].concat();
        let recovered = assert_c_recovery(&stream, &[1, 3, stream.len()]);
        assert_eq!(
            recovered,
            vec![good.clone(); if marker == 253 { 2 } else { 1 }]
        );
    }
    // Unsupported flags consume the FD flags byte; it is not a new start byte.
    let stream = [vec![253, 255, 253], good[1..].to_vec(), good.clone()].concat();
    assert_eq!(
        assert_c_recovery(&stream, &[1, 3, stream.len()]),
        vec![good.clone()]
    );

    // Rust's length check rejects a zero-length heartbeat before checking its
    // CRC. Still reuse CRC2 just as C does for the resulting checksum failure.
    let stream = [
        vec![253, 0, 0, 0, 0, 42, 11, 0, 0, 0, 0, 253],
        good[1..].to_vec(),
        good.clone(),
    ]
    .concat();
    assert_eq!(
        assert_c_recovery(&stream, &[1, 3, stream.len()]),
        vec![good.clone(), good.clone()]
    );

    let signed = bytes(&heartbeat(V::V2, true, true));
    let crc_end = signed.len() - 13;
    let mut bad = signed[..crc_end].to_vec();
    bad[crc_end - 2] ^= 1;
    bad[crc_end - 1] = 0;
    // A new packet begins where the bad frame's signature would have started.
    // Waiting for those 13 bytes would swallow the start of this intact packet.
    let stream = [bad.clone(), good.clone()].concat();
    assert_eq!(
        assert_c_recovery(&stream, &[1, crc_end, stream.len()]),
        vec![good.clone()]
    );
    let mut parser = StreamParser::default();
    for (i, byte) in bad.iter().enumerate() {
        let event = parser.push::<All>(*byte);
        if i + 1 == crc_end {
            assert!(matches!(event, Some(Err(Error::BadCrc))));
        } else {
            assert!(event.is_none());
        }
    }

    // A valid CRC reserves all signature bytes, even if they contain a complete
    // packet. Authentication is deliberately separate from stream framing.
    let inner = bytes(&Frame::from_message(V::V2, 7, 255, 11, &SetMode::default()).unwrap());
    assert_eq!(inner.len(), 13);
    let mut signed = signed;
    signed[crc_end..].copy_from_slice(&inner);
    let stream = [signed.clone(), good.clone()].concat();
    assert_eq!(
        assert_c_recovery(&stream, &[1, crc_end, stream.len()]),
        vec![signed, good.clone()]
    );

    // C treats a missing CRC_EXTRA as a CRC rejection and reuses CRC2 too.
    let info = WireInfo {
        id: 6_000_000,
        crc_extra: 117,
        min_length: 9,
        length: 9,
        target_offset: None,
    };
    let mut unknown = bytes(
        &Frame::from_payload(0, 42, 11, &Payload::new(info.id, &[0xa5; 9], V::V2), info).unwrap(),
    );
    *unknown.last_mut().unwrap() = 253;
    let stream = [unknown, good[1..].to_vec()].concat();
    assert_eq!(assert_c_recovery(&stream, &[1, stream.len()]), vec![good]);
}

#[test]
fn preserve_policy_applies_after_crc_failure() {
    let good = bytes(&heartbeat(V::V2, false, false));
    let mut bad = good.clone();
    let n = bad.len();
    bad[n - 2] ^= 1;
    bad[n - 1] = 0;
    let info = WireInfo {
        id: 6_000_000,
        crc_extra: 117,
        min_length: 1,
        length: 255,
        target_offset: None,
    };
    // Unknown packets may themselves carry complete known packets. Preserve
    // forwards the outer packet without interpreting its payload.
    let unknown =
        bytes(&Frame::from_payload(0, 42, 11, &Payload::new(info.id, &good, V::V2), info).unwrap());
    let stream = [bad, unknown.clone(), good.clone()].concat();
    for chunk in 1..=stream.len() {
        let mut parser = StreamParser::new(ParserOptions {
            unknown_messages: UnknownMessages::Preserve,
            ..Default::default()
        });
        let mut events = Vec::new();
        for part in stream.chunks(chunk) {
            parser.feed::<All>(part, |r| events.push(r));
        }
        assert_eq!(events.len(), 3);
        assert!(matches!(events[0], Err(Error::BadCrc)));
        assert_eq!(
            events[1].as_ref().unwrap().crc_status(),
            CrcStatus::UnknownExtra
        );
        assert_eq!(forwarded(events[1].as_ref().unwrap()), unknown);
        assert_eq!(forwarded(events[2].as_ref().unwrap()), good);
    }
}

#[test]
fn maximum_frame_and_byte_api_recovery() {
    let info = WireInfo {
        id: 6_000_000,
        crc_extra: 117,
        min_length: 9,
        length: 255,
        target_offset: Some(0),
    };
    let mut frame = Frame::from_payload(
        0,
        u32::MAX,
        11,
        &Payload::new(info.id, &[0xa5; 255], V::V2),
        info,
    )
    .unwrap();
    frame.retarget(u32::MAX).unwrap();
    frame.sign(&[42; 32], 3, 1000).unwrap();
    let packet = bytes(&frame);
    assert_eq!(packet.len(), MAX_FRAME_SIZE);
    let mut parser = StreamParser::new(ParserOptions {
        unknown_messages: UnknownMessages::Preserve,
        ..Default::default()
    });
    for (i, byte) in packet.iter().enumerate() {
        let result = parser.push::<All>(*byte);
        if i + 1 == packet.len() {
            let received = result.unwrap().unwrap();
            assert_eq!(forwarded(&received), packet);
            let ReceivedFrame::Unknown(raw) = received else {
                panic!("missing metadata");
            };
            raw.verify_signature(&[42; 32]).unwrap();
        } else {
            assert!(result.is_none());
        }
        assert!(parser.drain_next::<All>().is_none());
    }
    // Candidate starts at every byte, maximal false lengths and unsupported flags.
    let mut state = 12345u32;
    for _ in 0..10_000 {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        let byte = [253, 254, 255, 0, 128, (state >> 24) as u8][(state % 6) as usize];
        if let Some(result) = parser.push::<All>(byte) {
            assert!(!matches!(result, Err(Error::BufferFull)));
        }
        while let Some(result) = parser.drain_next::<All>() {
            assert!(!matches!(result, Err(Error::BufferFull)));
        }
    }
    parser.reset();
    let known = bytes(&heartbeat(V::V2, false, false));
    let mut count = 0;
    parser.feed::<All>(&known, |r| {
        r.unwrap();
        count += 1;
    });
    assert_eq!(count, 1);
}

#[test]
fn rejecting_unknown_schema_does_not_promote_embedded_packets() {
    let inner = bytes(&heartbeat(V::V2, false, false));
    let info = WireInfo {
        id: 6_000_000,
        crc_extra: 117,
        min_length: 1,
        length: 255,
        target_offset: None,
    };
    let mut payload = vec![0; 5];
    payload.extend_from_slice(&inner);
    let outer =
        Frame::from_payload(0, 42, 11, &Payload::new(info.id, &payload, V::V2), info).unwrap();
    let mut results = Vec::new();
    StreamParser::default().feed::<All>(&bytes(&outer), |r| results.push(r));
    assert_eq!(results.len(), 1);
    assert!(matches!(results[0], Err(Error::UnknownMessage(6_000_000))));
}

#[test]
fn lossless_decode_preserves_wide_header_targets() {
    let mut frame = Frame::from_message(V::V2, 0, 42, 11, &CommandLong::default()).unwrap();
    for target in [256, u32::MAX] {
        frame.retarget(target).unwrap();
        frame.sign(&[42; 32], 3, 1000).unwrap();
        assert_eq!(frame.target_system(), Some(target));
        assert_raw_fallback(&frame);
    }
}

#[test]
fn stream_checks_each_frame_once() {
    use std::cell::Cell;
    let calls = Cell::new(0);
    let mut lookup = |id| {
        calls.set(calls.get() + 1);
        All::wire_info(id)
    };
    let mut parser = StreamParser::default();
    parser.feed_with(&[253, 255, 128], &mut lookup, |_| {});
    let mut frame = heartbeat(V::V2, true, true);
    frame.set_sequence(253);
    frame.sign(&[42; 32], 3, 1000).unwrap();
    let packet = bytes(&frame);
    let mut received = Vec::new();
    for byte in &packet {
        parser.feed_with(&[*byte], &mut lookup, |r| received.push(r));
        let before = calls.get();
        for _ in 0..8 {
            assert!(parser.drain_next_with(&mut lookup).is_none());
        }
        assert_eq!(calls.get(), before);
    }
    assert_eq!(calls.get(), 1);
    assert_eq!(received.len(), 1);
    assert_eq!(forwarded(received[0].as_ref().unwrap()), packet);
}
