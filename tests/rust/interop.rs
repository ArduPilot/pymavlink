use mavlink_generated::spec::{IntoPayload, MavLinkVersion, MessageSpec};
use mavlink_generated::wire::{Frame, WireDialect, MAX_FRAME_SIZE};
use std::io::Write;

fn unhex(value: &str) -> Vec<u8> {
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|b| u8::from_str_radix(std::str::from_utf8(b).unwrap(), 16).unwrap())
        .collect()
}
fn emit(output: &mut std::fs::File, frame: &Frame) {
    let mut bytes = [0; MAX_FRAME_SIZE];
    let length = frame.write(&mut bytes);
    write!(
        output,
        "{} {} {} {} {} {} ",
        frame.message_id(),
        if frame.version() == MavLinkVersion::V1 {
            1
        } else {
            2
        },
        frame.system_id(),
        frame.target_system().unwrap_or(0),
        u8::from(frame.is_signed()),
        length
    )
    .unwrap();
    for byte in &bytes[..length] {
        write!(output, "{byte:02x}").unwrap();
    }
    writeln!(output).unwrap();
}
#[test]
fn every_message_c_to_rust_and_back() {
    let mut output = std::fs::File::create(std::env::var("RUST_FRAMES").unwrap()).unwrap();
    let mut covered = std::collections::BTreeSet::new();
    let mut cases = 0;
    for line in include_str!("c.frames").lines() {
        let parts: Vec<_> = line.split_whitespace().collect();
        let id: u32 = parts[0].parse().unwrap();
        let pattern = parts[1].parse().unwrap();
        let version = if parts[2] == "1" {
            MavLinkVersion::V1
        } else {
            MavLinkVersion::V2
        };
        let source = parts[3].parse().unwrap();
        let target = parts[4].parse().unwrap();
        let signed = parts[5] == "1";
        let expected = unhex(parts[6]);
        let message = fixture(id, pattern);
        let info = All::wire_info(id).unwrap();
        let payload = message.encode(version).unwrap();
        let mut frame = Frame::from_payload(239, source, 11, &payload, info).unwrap();
        if info
            .target_offset
            .is_some_and(|n| version == MavLinkVersion::V2 || n < info.min_length as usize)
        {
            frame.retarget(target).unwrap();
        }
        if signed {
            frame.sign(&[42; 32], 3, 1000).unwrap();
        }
        let mut actual = [0; MAX_FRAME_SIZE];
        let length = frame.write(&mut actual);
        assert_eq!(&actual[..length], expected, "message {id}, pattern {pattern}, v{version:?}, source {source}, target {target}, signed {signed}");
        let received = Frame::parse::<All>(&expected).unwrap();
        assert_eq!(received.system_id(), source);
        assert_eq!(received.target_system().unwrap_or(0), target);
        if signed {
            received.verify_signature(&[42; 32]).unwrap();
        }
        let decoded = received.decode::<All>().unwrap();
        assert_eq!(decoded.id(), id);
        assert_eq!(
            decoded.encode(version).unwrap().bytes(),
            received.payload(),
            "decode payload {id}"
        );
        emit(&mut output, &received);
        // Routing/signing never instantiate a message and preserve every other payload byte.
        if info.target_offset.is_some() && version == MavLinkVersion::V2 {
            for new_target in [0, 255, 256, u32::MAX] {
                let mut routed = received.clone();
                routed.retarget(new_target).unwrap();
                assert!(!routed.is_signed());
                assert_eq!(routed.target_system(), Some(new_target));
                emit(&mut output, &routed);
                routed.sign(&[42; 32], 3, 1001).unwrap();
                emit(&mut output, &routed);
            }
        }
        covered.insert(id);
        cases += 1;
    }
    assert_eq!(
        covered.len(),
        std::env::var("EXPECT_MESSAGES")
            .unwrap()
            .parse::<usize>()
            .unwrap()
    );
    println!(
        "{} messages, {} C/Rust wire comparisons",
        covered.len(),
        cases
    );
}
