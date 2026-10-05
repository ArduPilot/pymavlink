use mavlink_generated::{
    messages::{CommandLong, Heartbeat},
    spec::{IntoPayload, MavLinkVersion as V},
    wire::*,
    DefaultDialect as All,
};
fn frame() -> Frame {
    Frame::from_message(V::V2, 9, 0x80000001, 11, &CommandLong::default()).unwrap()
}
fn bytes(frame: &Frame) -> Vec<u8> {
    let mut bytes = [0; MAX_FRAME_SIZE];
    let n = frame.write(&mut bytes);
    bytes[..n].to_vec()
}
#[test]
fn signing_and_replay() {
    let mut a = frame();
    a.retarget(u32::MAX).unwrap();
    assert!(matches!(
        a.verify_signature(&[42; 32]),
        Err(Error::Unsigned)
    ));
    a.sign(&[42; 32], 7, 8_000_000).unwrap();
    a.verify_signature(&[42; 32]).unwrap();
    assert!(matches!(
        a.verify_signature(&[41; 32]),
        Err(Error::BadSignature)
    ));
    let mut wire = bytes(&a);
    let end = wire.len() - 1;
    wire[end] ^= 1;
    let tampered = Frame::parse::<All>(&wire).unwrap();
    assert!(matches!(
        tampered.verify_signature(&[42; 32]),
        Err(Error::BadSignature)
    ));
    let mut guard = ReplayGuard::<2>::default();
    guard.verify(&a, &[42; 32], 8_000_000).unwrap();
    assert!(matches!(
        guard.verify(&a, &[42; 32], 8_000_000),
        Err(Error::Replay)
    ));
    a.sign(&[42; 32], 7, 8_000_001).unwrap();
    assert!(guard.verify(&a, &[41; 32], 8_000_000).is_err());
    guard.verify(&a, &[42; 32], 8_000_000).unwrap();
    let mut b = a.clone();
    b.set_source(1, 11).unwrap();
    b.sign(&[42; 32], 7, 8_000_001).unwrap();
    guard.verify(&b, &[42; 32], 8_000_000).unwrap(); // Same low source byte, independent stream.
    b.set_source(2, 11).unwrap();
    b.sign(&[42; 32], 7, 8_000_001).unwrap();
    assert!(matches!(
        guard.verify(&b, &[42; 32], 8_000_000),
        Err(Error::ReplayTableFull)
    ));
    let mut fresh = ReplayGuard::<1>::default();
    assert!(matches!(
        fresh.verify(&b, &[42; 32], 20_000_000),
        Err(Error::StaleTimestamp)
    ));
    assert!(a.sign(&[42; 32], 7, 1 << 48).is_err());
    a.retarget(255).unwrap();
    assert!(!a.is_signed());
    a.sign(&[42; 32], 7, (1 << 48) - 1).unwrap();
    a.verify_signature(&[42; 32]).unwrap();
    a.strip_signature();
    assert!(!a.is_signed());
    assert!(Frame::parse::<All>(&bytes(&a)).is_ok());
}
#[test]
fn v1_rejects_wide_ids_without_mutating_frame() {
    assert!(Frame::from_message(V::V1, 0, 256, 1, &Heartbeat::default()).is_err());
    let mut f = Frame::from_message(V::V1, 0, 255, 1, &CommandLong::default()).unwrap();
    let original = bytes(&f);
    assert!(f.retarget(256).is_err());
    assert_eq!(bytes(&f), original);
    assert!(f.set_source(256, 1).is_err());
    assert_eq!(bytes(&f), original);
    assert!(f.sign(&[42; 32], 3, 1000).is_err());
    assert_eq!(bytes(&f), original);
    let mut heartbeat = Frame::from_message(V::V2, 0, 1, 1, &Heartbeat::default()).unwrap();
    assert!(matches!(heartbeat.retarget(0), Err(Error::NoTarget)));
}
#[test]
fn stream_fragmentation_rejection_and_recovery() {
    let mut f = frame();
    f.retarget(u32::MAX).unwrap();
    f.sign(&[42; 32], 3, 1000).unwrap();
    let valid = bytes(&f);
    for split in 0..=valid.len() {
        let mut parser = Parser::default();
        let mut count = 0;
        for chunk in [&valid[..split], &valid[split..]] {
            for b in chunk {
                if let Some(result) = parser.push::<All>(*b) {
                    assert_eq!(result.unwrap().system_id(), 0x80000001);
                    count += 1;
                }
            }
        }
        assert_eq!(count, 1);
    }
    for cut in 0..valid.len() {
        assert!(Frame::parse::<All>(&valid[..cut]).is_err());
    }
    let mut corrupt = valid.clone();
    corrupt[17] ^= 1;
    assert!(matches!(Frame::parse::<All>(&corrupt), Err(Error::BadCrc)));
    // Unknown flags, maximum payload, signed/wide header. Embedded packets cannot leak.
    let mut bad = vec![253, 255, 0x87, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
    let payload_start = bad.len();
    bad.resize(payload_start + 255, 0);
    bad[payload_start..payload_start + valid.len().min(255)]
        .copy_from_slice(&valid[..valid.len().min(255)]);
    bad.resize(MAX_FRAME_SIZE, 0);
    for rejected in [&bad, &corrupt] {
        let mut parser = Parser::default();
        let mut results = Vec::new();
        for b in [0, 12, 15]
            .iter()
            .chain(rejected.iter())
            .chain(valid.iter())
        {
            if let Some(r) = parser.push::<All>(*b) {
                results.push(r);
            }
        }
        assert_eq!(results.len(), 2);
        assert!(results[0].is_err());
        assert!(results[1].is_ok());
    }
}
#[test]
fn opaque_routing_preserves_unknown_field_values() {
    let info = <CommandLong as WireMessage>::WIRE_INFO;
    let mut payload = [0xa5; 33];
    payload[30] = 3; // Unknown command enum, deliberately not decoded.
    let p = mavlink_generated::spec::Payload::new(info.id, &payload, V::V2);
    let mut f = Frame::from_payload(1, 0xffffffff, 11, &p, info).unwrap();
    for target in [256, 0xffffffff, 7, 0] {
        f.retarget(target).unwrap();
        let received = Frame::parse::<All>(&bytes(&f)).unwrap();
        assert_eq!(received.target_system(), Some(target));
        for i in 0..33 {
            if Some(i) != info.target_offset {
                assert_eq!(received.payload()[i], payload[i]);
            }
        }
    }
}
#[test]
fn actual_mavio_compatibility() {
    let msg = Heartbeat {
        mavlink_version: 3,
        ..Default::default()
    };
    let stock = mavio::Frame::builder()
        .version(mavio::protocol::V2)
        .sequence(9)
        .system_id(42)
        .component_id(11)
        .message(&msg)
        .unwrap()
        .build();
    let mut encoded = [0; MAX_FRAME_SIZE];
    let n = stock.serialize(&mut encoded).unwrap();
    let received = Frame::parse::<All>(&encoded[..n]).unwrap();
    assert_eq!(received.decode::<All>().unwrap(), msg.clone().into());
    assert_eq!(stock.decode::<All>().unwrap(), msg.clone().into());
    let ours = Frame::from_message(V::V2, 9, 42, 11, &msg).unwrap();
    assert_eq!(bytes(&ours), encoded[..n]);
    assert_eq!(msg.encode(V::V1).unwrap().bytes().len(), 9);
}

#[test]
fn opaque_forwarding_retains_future_extension_bytes() {
    let info = <CommandLong as WireMessage>::WIRE_INFO;
    let payload = mavlink_generated::spec::Payload::new(info.id, &[0xa5; 255], V::V2);
    let mut f = Frame::from_payload(1, u32::MAX, 11, &payload, info).unwrap();
    f.retarget(u32::MAX).unwrap();
    f.sign(&[42; 32], 3, 1000).unwrap();
    let serialized = bytes(&f);
    assert_eq!(serialized.len(), MAX_FRAME_SIZE);
    let mut received = Frame::parse::<All>(&serialized).unwrap();
    received.verify_signature(&[42; 32]).unwrap();
    received.retarget(7).unwrap();
    assert_eq!(&received.payload()[33..], &[0xa5; 222]);
}

fn replace_crc(bytes: &mut [u8], extra: u8) {
    let end = bytes.len() - 2;
    let mut crc = 0xffffu16;
    for byte in bytes[1..end].iter().chain(core::iter::once(&extra)) {
        crc ^= *byte as u16;
        for _ in 0..8 {
            crc = (crc >> 1) ^ if crc & 1 != 0 { 0x8408 } else { 0 };
        }
    }
    bytes[end..].copy_from_slice(&crc.to_le_bytes());
}

#[test]
fn header_target_precedence_matches_c() {
    let mut f = frame();
    f.retarget(256).unwrap();
    let info = <CommandLong as WireMessage>::WIRE_INFO;
    for target in [0u32, 7, 255, u32::MAX] {
        let mut packet = bytes(&f);
        packet[13..17].copy_from_slice(&target.to_le_bytes());
        packet[17 + info.target_offset.unwrap()] = 19;
        replace_crc(&mut packet, info.crc_extra);
        let received = Frame::parse::<All>(&packet).unwrap();
        assert_eq!(received.target_system(), Some(target));
        assert_eq!(bytes(&received), packet);
    }
}

#[test]
fn malformed_v1_length_and_unknown_metadata() {
    let f = Frame::from_message(V::V1, 0, 1, 11, &Heartbeat::default()).unwrap();
    let mut packet = bytes(&f);
    packet[1] -= 1;
    packet.remove(6);
    replace_crc(&mut packet, <Heartbeat as WireMessage>::WIRE_INFO.crc_extra);
    assert!(matches!(
        Frame::parse::<All>(&packet),
        Err(Error::InvalidLength)
    ));
    assert!(matches!(
        Frame::parse_with(&bytes(&f), |_| None),
        Err(Error::UnknownMessage(0))
    ));
}

#[test]
fn actual_mavio_signing_compatibility() {
    use mavio::protocol::{MavTimestamp, SigningConf, V2};
    let mut ours = Frame::from_message(V::V2, 9, 42, 11, &Heartbeat::default()).unwrap();
    ours.sign(&[42; 32], 3, 1000).unwrap();
    // The buffer is a complete frame produced by our serializer and checked against C.
    let mut stock = unsafe { mavio::Frame::<V2>::deserialize(&bytes(&ours)).unwrap() };
    let mut signer = mavio::utils::MavSha256::default();
    stock.validate_checksum::<All>().unwrap();
    assert!(stock
        .validate_signature(&mut signer, &[42; 32].into())
        .is_ok());
    stock.add_signature(
        &mut signer,
        &SigningConf {
            link_id: 3,
            timestamp: MavTimestamp::from_raw_u64(1001),
            secret: [42; 32].into(),
        },
    );
    ours.sign(&[42; 32], 3, 1001).unwrap();
    let mut output = [0; MAX_FRAME_SIZE];
    let n = stock.serialize(&mut output).unwrap();
    assert_eq!(bytes(&ours), output[..n]);
}
