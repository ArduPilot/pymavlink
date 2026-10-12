//! Allocation-free framing, routing and MAVLink 2 signing.
//!
//! Payloads stay opaque: routing needs only `WireInfo`, never message decoding.
//! Known-message parsing checks CRCs but does not authenticate signatures. Explicit
//! unknown-message forwarding keeps an unchecked RawFrame. Use `verify_signature`
//! or ReplayGuard before trusting signed traffic.
use crate::spec::{Dialect, MavLinkVersion, Message, Payload, SpecError};
use sha2::{Digest, Sha256};

pub const MAX_FRAME_SIZE: usize = 287;
pub const SYSID32: u8 = 2;
pub const TARGET32: u8 = 4;
pub const SIGNED: u8 = 1;
const MAX_TIMESTAMP: u64 = (1u64 << 48) - 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WireInfo {
    pub id: u32,
    pub crc_extra: u8,
    pub min_length: u8,
    pub length: u8,
    pub target_offset: Option<usize>,
}
pub trait WireMessage: Message {
    const WIRE_INFO: WireInfo;
    /// Encode a newly originated message, filling XML protocol constants.
    fn encode_wire(&self, version: MavLinkVersion) -> Result<Payload, SpecError> {
        self.encode(version)
    }
}
pub trait WireDialect: Dialect {
    fn wire_info(id: u32) -> Option<WireInfo>;
}

#[derive(Debug)]
pub enum Error {
    Payload(SpecError),
    InvalidLength,
    InvalidMetadata,
    UnsupportedVersion,
    UnsupportedFlags(u8),
    UnknownMessage(u32),
    WideIdInV1,
    NoTarget,
    BadCrc,
    BadSignature,
    Unsigned,
    TimestampOverflow,
    Replay,
    StaleTimestamp,
    ReplayTableFull,
    LossyDecode,
    BufferFull,
}
impl core::fmt::Display for Error {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(formatter, "MAVLink error: {self:?}")
    }
}
#[cfg(feature = "std")]
impl std::error::Error for Error {}

impl From<SpecError> for Error {
    fn from(value: SpecError) -> Self {
        Self::Payload(value)
    }
}

/// An opaque, CRC-validated frame. Fields are private so mutations cannot leave
/// stale signatures. Serialization recalculates the CRC, including extended headers.
#[derive(Clone, Debug)]
pub struct Frame {
    version: MavLinkVersion,
    sequence: u8,
    system_id: u32,
    component_id: u8,
    compat_flags: u8,
    wide_source: bool,
    header_target: Option<u32>,
    payload: [u8; 255],
    length: usize,
    info: WireInfo,
    signature: Option<[u8; 13]>,
}

impl Frame {
    /// Originate a typed message, filling XML protocol constants (e.g. heartbeat
    /// version). Use write/retarget on the original frame for opaque forwarding.
    pub fn from_message<M: WireMessage>(
        version: MavLinkVersion,
        sequence: u8,
        system_id: u32,
        component_id: u8,
        message: &M,
    ) -> Result<Self, Error> {
        let payload = message.encode_wire(version)?;
        Self::from_payload(sequence, system_id, component_id, &payload, M::WIRE_INFO)
    }

    pub fn from_payload(
        sequence: u8,
        system_id: u32,
        component_id: u8,
        payload: &Payload,
        info: WireInfo,
    ) -> Result<Self, Error> {
        Self::validate_info(info)?;
        if payload.id() != info.id {
            return Err(Error::InvalidMetadata);
        }
        if payload.version() == MavLinkVersion::V1 && (system_id > 255 || info.id > 255) {
            return Err(Error::WideIdInV1);
        }
        let length = payload.bytes().len();
        Self::validate_length(payload.version(), length, info)?;
        let mut frame = Self {
            version: payload.version(),
            sequence,
            system_id,
            component_id,
            compat_flags: 0,
            wide_source: system_id > 255,
            header_target: None,
            payload: [0; 255],
            length,
            info,
            signature: None,
        };
        frame.payload[..length].copy_from_slice(payload.bytes());
        Ok(frame)
    }

    fn validate_info(info: WireInfo) -> Result<(), Error> {
        if info.id > 0xffffff
            || info.min_length > info.length
            || info.length == 0
            || info
                .target_offset
                .is_some_and(|n| n >= info.length as usize)
        {
            return Err(Error::InvalidMetadata);
        }
        Ok(())
    }
    fn validate_length(
        version: MavLinkVersion,
        length: usize,
        info: WireInfo,
    ) -> Result<(), Error> {
        if (version == MavLinkVersion::V1 && length != info.min_length as usize)
            || length == 0
            || length > 255
        {
            return Err(Error::InvalidLength);
        }
        Ok(())
    }
    pub fn version(&self) -> MavLinkVersion {
        self.version
    }
    pub fn sequence(&self) -> u8 {
        self.sequence
    }
    pub fn system_id(&self) -> u32 {
        self.system_id
    }
    pub fn component_id(&self) -> u8 {
        self.component_id
    }
    pub fn message_id(&self) -> u32 {
        self.info.id
    }
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.length]
    }
    pub fn compat_flags(&self) -> u8 {
        self.compat_flags
    }
    pub fn is_signed(&self) -> bool {
        self.signature.is_some()
    }
    pub fn signature(&self) -> Option<&[u8; 13]> {
        self.signature.as_ref()
    }
    pub fn target_system(&self) -> Option<u32> {
        self.header_target.or_else(|| {
            self.info
                .target_offset
                .filter(|&n| self.version == MavLinkVersion::V2 || n < self.length)
                .map(|n| u32::from(self.payload[n]))
        })
    }
    pub fn to_payload(&self) -> Payload {
        Payload::new(self.info.id, self.payload(), self.version)
    }
    /// MAVSpec-compatible decode: unknown enums fail, but unknown bitmask bits may
    /// be discarded. Use decode_with when the typed result must preserve all data.
    pub fn decode<D: Dialect>(&self) -> Result<D, SpecError> {
        D::decode(&self.to_payload())
    }

    /// Decode only when the typed value can represent every nonzero payload byte.
    /// Unlike MAVSpec's decoder, this detects silently discarded bitmask bits and
    /// future extensions and header targets not represented by the payload target.
    /// Trailing zero padding is equivalent in MAVLink 2.
    pub fn decode_with<D: Dialect>(&self, policy: DecodePolicy) -> Result<Decoded<'_, D>, Error> {
        let checked = (|| {
            if let Some(target) = self.header_target {
                let payload_target = self
                    .info
                    .target_offset
                    .map(|offset| u32::from(self.payload[offset]));
                if payload_target != Some(target) {
                    return Err(Error::LossyDecode);
                }
            }
            let message = self.decode::<D>()?;
            let encoded = message.encode(self.version)?;
            let original = self.payload();
            let encoded = encoded.bytes();
            if (0..original.len().max(encoded.len())).any(|i| {
                original.get(i).copied().unwrap_or(0) != encoded.get(i).copied().unwrap_or(0)
            }) {
                return Err(Error::LossyDecode);
            }
            Ok(message)
        })();
        match checked {
            Ok(message) => Ok(Decoded::Known(message)),
            Err(_) if policy == DecodePolicy::PreserveRaw => Ok(Decoded::Raw(self)),
            Err(error) => Err(error),
        }
    }

    /// Change only the routing byte/header. Wide targets use a 255 payload marker,
    /// and switching back to a small target removes TARGET32. Any signature is removed.
    pub fn retarget(&mut self, target: u32) -> Result<(), Error> {
        let offset = self.info.target_offset.ok_or(Error::NoTarget)?;
        if self.version == MavLinkVersion::V1 && target > 255 {
            return Err(Error::WideIdInV1);
        }
        if self.version == MavLinkVersion::V1 && offset >= self.length {
            return Err(Error::NoTarget);
        }
        self.strip_signature();
        self.payload[offset] = if target > 255 { 255 } else { target as u8 };
        self.header_target = if target > 255 { Some(target) } else { None };
        if self.version == MavLinkVersion::V2 {
            self.length = self.length.max(offset + 1);
            while self.length > 1 && self.payload[self.length - 1] == 0 {
                self.length -= 1;
            }
        }
        Ok(())
    }
    pub fn set_source(&mut self, system_id: u32, component_id: u8) -> Result<(), Error> {
        if self.version == MavLinkVersion::V1 && system_id > 255 {
            return Err(Error::WideIdInV1);
        }
        self.strip_signature();
        self.system_id = system_id;
        self.component_id = component_id;
        self.wide_source = system_id > 255;
        Ok(())
    }
    pub fn set_sequence(&mut self, sequence: u8) {
        self.strip_signature();
        self.sequence = sequence;
    }
    pub fn set_compat_flags(&mut self, flags: u8) {
        self.strip_signature();
        self.compat_flags = flags;
    }
    pub fn strip_signature(&mut self) {
        self.signature = None;
    }

    fn header(&self, output: &mut [u8; MAX_FRAME_SIZE]) -> usize {
        output[1] = self.length as u8;
        if self.version == MavLinkVersion::V1 {
            output[0] = 254;
            output[2] = self.sequence;
            output[3] = self.system_id as u8;
            output[4] = self.component_id;
            output[5] = self.info.id as u8;
            return 6;
        }
        output[0] = 253;
        output[2] = if self.is_signed() { SIGNED } else { 0 }
            | if self.wide_source { SYSID32 } else { 0 }
            | if self.header_target.is_some() {
                TARGET32
            } else {
                0
            };
        output[3] = self.compat_flags;
        output[4] = self.sequence;
        let size = if self.wide_source { 4 } else { 1 };
        output[5..5 + size].copy_from_slice(&self.system_id.to_le_bytes()[..size]);
        let mut n = 5 + size;
        output[n] = self.component_id;
        n += 1;
        output[n..n + 3].copy_from_slice(&self.info.id.to_le_bytes()[..3]);
        n += 3;
        if let Some(target) = self.header_target {
            output[n..n + 4].copy_from_slice(&target.to_le_bytes());
            n += 4;
        }
        n
    }
    /// Serialize into caller-owned storage, returning the frame length.
    pub fn write(&self, output: &mut [u8; MAX_FRAME_SIZE]) -> usize {
        let header_len = self.header(output);
        let mut n = header_len + self.length;
        output[header_len..n].copy_from_slice(self.payload());
        let checksum = crc(&output[1..n], self.info.crc_extra);
        output[n..n + 2].copy_from_slice(&checksum.to_le_bytes());
        n += 2;
        if let Some(signature) = self.signature {
            output[n..n + 13].copy_from_slice(&signature);
            n += 13;
        }
        n
    }

    /// Decode one complete frame using a dialect's small metadata table.
    pub fn parse<D: WireDialect>(bytes: &[u8]) -> Result<Self, Error> {
        Self::parse_with(bytes, D::wire_info)
    }
    /// The lookup only supplies lengths, target offset and CRC_EXTRA. No generated
    /// message type is constructed, and unknown enum/flag field values stay intact.
    pub fn parse_with(
        bytes: &[u8],
        lookup: impl FnOnce(u32) -> Option<WireInfo>,
    ) -> Result<Self, Error> {
        let header = parse_header(bytes)?;
        let info = lookup(header.id).ok_or(Error::UnknownMessage(header.id))?;
        Self::validate_payload(bytes, &header, info)?;
        Ok(Self::from_validated(bytes, header, info))
    }

    // The streaming parser checks this at CRC2, before waiting for a signature,
    // just like mavlink_parse_char. Callers must supply the full header and CRC.
    fn validate_payload(bytes: &[u8], header: &Header, info: WireInfo) -> Result<(), Error> {
        Self::validate_info(info)?;
        if info.id != header.id {
            return Err(Error::InvalidMetadata);
        }
        let length = bytes[1] as usize;
        Self::validate_length(header.version, length, info)?;
        let end = header.header_len + length;
        if crc(&bytes[1..end], info.crc_extra) != u16::from_le_bytes([bytes[end], bytes[end + 1]]) {
            return Err(Error::BadCrc);
        }
        Ok(())
    }

    // Only called after checksum validation and reception of the entire frame.
    fn from_validated(bytes: &[u8], header: Header, info: WireInfo) -> Self {
        let Header {
            version,
            flags,
            sequence,
            system_id,
            component_id,
            target,
            header_len,
            ..
        } = header;
        let length = bytes[1] as usize;
        let end = header_len + length;
        let mut payload = [0; 255];
        payload[..length].copy_from_slice(&bytes[header_len..end]);
        let signature = if flags & SIGNED != 0 {
            Some(bytes[end + 2..].try_into().unwrap())
        } else {
            None
        };
        Self {
            version,
            sequence,
            system_id,
            component_id,
            compat_flags: if version == MavLinkVersion::V2 {
                bytes[3]
            } else {
                0
            },
            wide_source: flags & SYSID32 != 0,
            header_target: target,
            payload,
            length,
            info,
            signature,
        }
    }

    /// Attach a fresh signature. Timestamp is in MAVLink's 10-microsecond units.
    /// Caller must persist/increment timestamps; reuse can cause receiver rejection.
    pub fn sign(&mut self, key: &[u8; 32], link_id: u8, timestamp: u64) -> Result<(), Error> {
        if self.version != MavLinkVersion::V2 {
            return Err(Error::UnsupportedVersion);
        }
        if timestamp > MAX_TIMESTAMP {
            return Err(Error::TimestampOverflow);
        }
        let mut signature = [0; 13];
        signature[0] = link_id;
        signature[1..7].copy_from_slice(&timestamp.to_le_bytes()[..6]);
        self.signature = Some(signature);
        let mut bytes = [0; MAX_FRAME_SIZE];
        let length = self.write(&mut bytes);
        let hash = Sha256::new()
            .chain_update(key)
            .chain_update(&bytes[..length - 6])
            .finalize();
        signature[7..].copy_from_slice(&hash[..6]);
        self.signature = Some(signature);
        Ok(())
    }
    /// Authenticate bytes only; use ReplayGuard when replay rejection is required.
    pub fn verify_signature(&self, key: &[u8; 32]) -> Result<(), Error> {
        let signature = self.signature.ok_or(Error::Unsigned)?;
        let mut bytes = [0; MAX_FRAME_SIZE];
        let length = self.write(&mut bytes);
        verify_hash(&bytes[..length], &signature, key)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodePolicy {
    /// Error if decoding fails or re-encoding would lose payload information.
    Strict,
    /// Return the original frame on any decode error or information loss.
    PreserveRaw,
}
#[derive(Debug)]
pub enum Decoded<'a, D> {
    Known(D),
    Raw(&'a Frame),
}

struct Header {
    version: MavLinkVersion,
    flags: u8,
    sequence: u8,
    system_id: u32,
    component_id: u8,
    id: u32,
    target: Option<u32>,
    header_len: usize,
}
fn parse_header(bytes: &[u8]) -> Result<Header, Error> {
    let (_, total) = frame_lengths(bytes)?;
    if bytes.len() != total {
        return Err(Error::InvalidLength);
    }
    parse_header_prefix(bytes)
}
fn parse_header_prefix(bytes: &[u8]) -> Result<Header, Error> {
    let (header_len, _) = frame_lengths(bytes)?;
    if bytes.len() < header_len {
        return Err(Error::InvalidLength);
    }
    let version = if bytes[0] == 254 {
        MavLinkVersion::V1
    } else {
        MavLinkVersion::V2
    };
    let flags = if version == MavLinkVersion::V1 {
        0
    } else {
        bytes[2]
    };
    if flags & !7 != 0 {
        return Err(Error::UnsupportedFlags(flags));
    }
    let (sequence, system_id, component_id, id, target) = if version == MavLinkVersion::V1 {
        (
            bytes[2],
            u32::from(bytes[3]),
            bytes[4],
            u32::from(bytes[5]),
            None,
        )
    } else {
        let size = if flags & SYSID32 != 0 { 4 } else { 1 };
        let mut source = [0; 4];
        source[..size].copy_from_slice(&bytes[5..5 + size]);
        let n = 5 + size;
        let id = u32::from_le_bytes([bytes[n + 1], bytes[n + 2], bytes[n + 3], 0]);
        let target = if flags & TARGET32 != 0 {
            Some(u32::from_le_bytes(bytes[n + 4..n + 8].try_into().unwrap()))
        } else {
            None
        };
        (bytes[4], u32::from_le_bytes(source), bytes[n], id, target)
    };
    Ok(Header {
        version,
        flags,
        sequence,
        system_id,
        component_id,
        id,
        target,
        header_len,
    })
}

fn verify_hash(bytes: &[u8], signature: &[u8; 13], key: &[u8; 32]) -> Result<(), Error> {
    let hash = Sha256::new()
        .chain_update(key)
        .chain_update(&bytes[..bytes.len() - 6])
        .finalize();
    let difference = signature[7..]
        .iter()
        .zip(&hash[..6])
        .fold(0u8, |v, (a, b)| v | (a ^ b));
    if difference != 0 {
        return Err(Error::BadSignature);
    }
    Ok(())
}

/// A complete frame with a supported header layout, but an UNCHECKED checksum.
/// This immutable representation permits forwarding messages without CRC_EXTRA.
/// No payload routing offsets are assumed. Authentication does not validate CRC.
#[derive(Clone, Debug)]
pub struct RawFrame {
    bytes: [u8; MAX_FRAME_SIZE],
    length: usize,
}
impl RawFrame {
    /// Check layout and length only. Prefer ReceivedFrame::parse_with so known
    /// messages must pass their CRC check before they can be forwarded.
    pub fn parse(bytes: &[u8]) -> Result<Self, Error> {
        parse_header(bytes)?;
        let mut frame = Self {
            bytes: [0; MAX_FRAME_SIZE],
            length: bytes.len(),
        };
        frame.bytes[..bytes.len()].copy_from_slice(bytes);
        Ok(frame)
    }
    fn header(&self) -> Header {
        // Construction validated the private, immutable buffer.
        parse_header(self.as_bytes()).unwrap()
    }
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }
    pub fn write(&self, output: &mut [u8; MAX_FRAME_SIZE]) -> usize {
        output[..self.length].copy_from_slice(self.as_bytes());
        self.length
    }
    pub fn version(&self) -> MavLinkVersion {
        self.header().version
    }
    pub fn sequence(&self) -> u8 {
        self.header().sequence
    }
    pub fn system_id(&self) -> u32 {
        self.header().system_id
    }
    pub fn component_id(&self) -> u8 {
        self.header().component_id
    }
    pub fn message_id(&self) -> u32 {
        self.header().id
    }
    /// Only TARGET32 is discoverable without message metadata. None means unknown,
    /// not necessarily broadcast; the application must choose its routing policy.
    pub fn target_system(&self) -> Option<u32> {
        self.header().target
    }
    pub fn payload(&self) -> &[u8] {
        let start = self.header().header_len;
        &self.bytes[start..start + self.bytes[1] as usize]
    }
    pub fn signature(&self) -> Option<&[u8; 13]> {
        if self.header().flags & SIGNED == 0 {
            return None;
        }
        Some(
            self.bytes[self.length - 13..self.length]
                .try_into()
                .unwrap(),
        )
    }
    pub fn verify_signature(&self, key: &[u8; 32]) -> Result<(), Error> {
        verify_hash(
            self.as_bytes(),
            self.signature().ok_or(Error::Unsigned)?,
            key,
        )
    }
    /// Resolve metadata later, validating the original checksum before any edits.
    pub fn validate_with(
        &self,
        lookup: impl FnOnce(u32) -> Option<WireInfo>,
    ) -> Result<Frame, Error> {
        Frame::parse_with(self.as_bytes(), lookup)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UnknownMessages {
    #[default]
    Reject,
    /// Accept an unchecked length-delimited packet at the current stream boundary.
    Preserve,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrcStatus {
    Validated,
    UnknownExtra,
}
#[derive(Clone, Debug)]
pub enum ReceivedFrame {
    Known(Frame),
    Unknown(RawFrame),
}
impl ReceivedFrame {
    pub fn parse<D: WireDialect>(bytes: &[u8], unknown: UnknownMessages) -> Result<Self, Error> {
        Self::parse_with(bytes, D::wire_info, unknown)
    }
    pub fn parse_with(
        bytes: &[u8],
        lookup: impl FnOnce(u32) -> Option<WireInfo>,
        unknown: UnknownMessages,
    ) -> Result<Self, Error> {
        let id = parse_header(bytes)?.id;
        match lookup(id) {
            Some(info) => Frame::parse_with(bytes, |_| Some(info)).map(Self::Known),
            None if unknown == UnknownMessages::Preserve => {
                RawFrame::parse(bytes).map(Self::Unknown)
            }
            None => Err(Error::UnknownMessage(id)),
        }
    }
    pub fn crc_status(&self) -> CrcStatus {
        match self {
            Self::Known(_) => CrcStatus::Validated,
            Self::Unknown(_) => CrcStatus::UnknownExtra,
        }
    }
    pub fn write(&self, output: &mut [u8; MAX_FRAME_SIZE]) -> usize {
        match self {
            Self::Known(frame) => frame.write(output),
            Self::Unknown(frame) => frame.write(output),
        }
    }
}

fn crc(bytes: &[u8], extra: u8) -> u16 {
    let mut result = 0xffffu16;
    for byte in bytes.iter().chain(core::iter::once(&extra)) {
        let mut t = byte ^ result as u8;
        t ^= t << 4;
        result = (result >> 8) ^ ((t as u16) << 8) ^ ((t as u16) << 3) ^ ((t as u16) >> 4);
    }
    result
}

fn frame_lengths(bytes: &[u8]) -> Result<(usize, usize), Error> {
    if bytes.len() < 3 {
        return Err(Error::InvalidLength);
    }
    let (header, signature) = match bytes[0] {
        254 => (6, 0),
        253 => (
            10 + if bytes[2] & SYSID32 != 0 { 3 } else { 0 }
                + if bytes[2] & TARGET32 != 0 { 4 } else { 0 },
            if bytes[2] & SIGNED != 0 { 13 } else { 0 },
        ),
        _ => return Err(Error::UnsupportedVersion),
    };
    Ok((header, header + bytes[1] as usize + 2 + signature))
}

/// Streaming parser with fixed storage, independent of message types. A rejected
/// frame is consumed whole, so valid-looking packets inside its payload cannot leak.
#[derive(Clone)]
pub struct Parser {
    bytes: [u8; MAX_FRAME_SIZE],
    length: usize,
}
impl Default for Parser {
    fn default() -> Self {
        Self {
            bytes: [0; MAX_FRAME_SIZE],
            length: 0,
        }
    }
}
impl Parser {
    pub fn reset(&mut self) {
        self.length = 0;
    }
    pub fn push<D: WireDialect>(&mut self, byte: u8) -> Option<Result<Frame, Error>> {
        self.push_with(byte, D::wire_info)
    }
    pub fn push_with(
        &mut self,
        byte: u8,
        lookup: impl FnOnce(u32) -> Option<WireInfo>,
    ) -> Option<Result<Frame, Error>> {
        if self.length == 0 && byte != 253 && byte != 254 {
            return None;
        }
        self.bytes[self.length] = byte;
        self.length += 1;
        if self.length < 3 {
            return None;
        }
        let (_, total) = frame_lengths(&self.bytes[..self.length]).unwrap();
        if self.length < total {
            return None;
        }
        let frame = Frame::parse_with(&self.bytes[..self.length], lookup);
        self.reset();
        Some(frame)
    }
}

/// Neither mode rescans payload bytes, which may contain complete MAVLink packets.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Recovery {
    /// Consume the entire declared packet on rejection, like Parser.
    Conservative,
    /// Follow mavlink_parse_char: reject unsupported flags immediately, reject a
    /// bad CRC at CRC2, and reuse that byte only if it is a MAVLink 2 start byte.
    /// A rejected payload length uses the same CRC2 restart rule.
    #[default]
    Recover,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct ParserOptions {
    pub recovery: Recovery,
    pub unknown_messages: UnknownMessages,
}

/// Single-candidate streaming parser with bounded storage. Each push consumes
/// one byte and returns at most one event; feed delivers events for whole slices.
#[derive(Clone)]
pub struct StreamParser {
    bytes: [u8; MAX_FRAME_SIZE],
    length: usize,
    // A valid signed frame waits for its signature without rechecking the CRC or
    // consulting mutable metadata again. None denotes an explicitly preserved
    // unknown message, whose CRC cannot be checked.
    checksum_checked: bool,
    info: Option<WireInfo>,
    options: ParserOptions,
}
impl Default for StreamParser {
    fn default() -> Self {
        Self::new(ParserOptions::default())
    }
}
impl StreamParser {
    pub fn new(options: ParserOptions) -> Self {
        Self {
            bytes: [0; MAX_FRAME_SIZE],
            length: 0,
            checksum_checked: false,
            info: None,
            options,
        }
    }
    /// Discard buffered input and treat the next magic byte as a fresh boundary.
    /// Replay state must be retained separately across resets.
    pub fn reset(&mut self) {
        self.length = 0;
        self.checksum_checked = false;
        self.info = None;
    }
    pub fn push<D: WireDialect>(&mut self, byte: u8) -> Option<Result<ReceivedFrame, Error>> {
        self.push_with(byte, D::wire_info)
    }
    /// Lookup must provide consistent metadata throughout a packet's reception.
    pub fn push_with(
        &mut self,
        byte: u8,
        lookup: impl FnMut(u32) -> Option<WireInfo>,
    ) -> Option<Result<ReceivedFrame, Error>> {
        // Valid declared lengths always complete within MAX_FRAME_SIZE.
        if self.length == MAX_FRAME_SIZE {
            return Some(Err(Error::BufferFull));
        }
        if self.length == 0 && byte != 253 && byte != 254 {
            return None;
        }
        self.bytes[self.length] = byte;
        self.length += 1;
        self.drain_next_with(lookup)
    }
    /// push delivers the only pending event, so draining after push or feed
    /// always returns None.
    pub fn drain_next<D: WireDialect>(&mut self) -> Option<Result<ReceivedFrame, Error>> {
        self.drain_next_with(D::wire_info)
    }
    pub fn drain_next_with(
        &mut self,
        mut lookup: impl FnMut(u32) -> Option<WireInfo>,
    ) -> Option<Result<ReceivedFrame, Error>> {
        if self.length < 3 {
            return None;
        }
        if self.options.recovery == Recovery::Recover
            && self.bytes[0] == 253
            && self.bytes[2] & !7 != 0
        {
            let error = Error::UnsupportedFlags(self.bytes[2]);
            // C consumes all three bytes, even if the flags byte is FD.
            self.reset();
            return Some(Err(error));
        }
        let (header_len, total) = frame_lengths(&self.bytes[..self.length]).unwrap();
        if self.options.recovery == Recovery::Conservative {
            if self.length < total {
                return None;
            }
            let result = ReceivedFrame::parse_with(
                &self.bytes[..total],
                lookup,
                self.options.unknown_messages,
            );
            self.reset();
            return Some(result);
        }
        let crc_end = header_len + self.bytes[1] as usize + 2;
        if self.length < crc_end {
            return None;
        }
        if !self.checksum_checked {
            let result = (|| {
                let bytes = &self.bytes[..self.length];
                let header = parse_header_prefix(bytes)?;
                let info = lookup(header.id);
                if let Some(info) = info {
                    Frame::validate_payload(bytes, &header, info)?;
                } else if self.options.unknown_messages == UnknownMessages::Reject {
                    return Err(Error::UnknownMessage(header.id));
                }
                Ok(info)
            })();
            match result {
                Ok(info) => {
                    self.info = info;
                    self.checksum_checked = true;
                }
                Err(error) => {
                    // C treats missing CRC_EXTRA as a CRC failure too. Apply the
                    // same restart rule to lengths rejected by Rust's stricter
                    // validation. Reuse only CRC2, never header or payload bytes.
                    let restart = self.bytes[self.length - 1] == 253
                        && matches!(
                            error,
                            Error::BadCrc | Error::UnknownMessage(_) | Error::InvalidLength
                        );
                    self.reset();
                    if restart {
                        self.bytes[0] = 253;
                        self.length = 1;
                    }
                    return Some(Err(error));
                }
            }
        }
        if self.length < total {
            return None;
        }
        let bytes = &self.bytes[..total];
        let result = match self.info {
            Some(info) => parse_header(bytes)
                .map(|header| ReceivedFrame::Known(Frame::from_validated(bytes, header, info))),
            None => RawFrame::parse(bytes).map(ReceivedFrame::Unknown),
        };
        self.reset();
        Some(result)
    }
    pub fn feed<D: WireDialect>(
        &mut self,
        bytes: &[u8],
        emit: impl FnMut(Result<ReceivedFrame, Error>),
    ) {
        self.feed_with(bytes, D::wire_info, emit);
    }
    /// Deliver each completed frame or rejection exactly once.
    pub fn feed_with(
        &mut self,
        bytes: &[u8],
        mut lookup: impl FnMut(u32) -> Option<WireInfo>,
        mut emit: impl FnMut(Result<ReceivedFrame, Error>),
    ) {
        for byte in bytes {
            if let Some(event) = self.push_with(*byte, &mut lookup) {
                emit(event);
            }
        }
    }
}

#[derive(Clone, Copy)]
struct ReplayStream {
    system: u32,
    component: u8,
    link: u8,
    timestamp: u64,
}
/// Bounded replay state per signing key. Never evicts: eviction would allow old
/// frames to be replayed. Provision N for all expected (system, component, link)
/// streams, and keep this guard across parser resets. Use a separate guard per key.
pub struct ReplayGuard<const N: usize> {
    streams: [Option<ReplayStream>; N],
}
impl<const N: usize> Default for ReplayGuard<N> {
    fn default() -> Self {
        Self { streams: [None; N] }
    }
}
impl<const N: usize> ReplayGuard<N> {
    /// New streams must be no more than one minute behind the local signing clock.
    /// `now` must be a trusted MAVLink timestamp (10us units since 2015-01-01).
    pub fn verify(&mut self, frame: &Frame, key: &[u8; 32], now: u64) -> Result<(), Error> {
        frame.verify_signature(key)?;
        self.accept(
            frame.system_id(),
            frame.component_id(),
            frame.signature().unwrap(),
            now,
        )
    }
    /// Authenticate an unknown-schema frame before updating the same replay table.
    /// Its checksum remains unchecked, even when authentication succeeds.
    pub fn verify_raw(&mut self, frame: &RawFrame, key: &[u8; 32], now: u64) -> Result<(), Error> {
        frame.verify_signature(key)?;
        self.accept(
            frame.system_id(),
            frame.component_id(),
            frame.signature().unwrap(),
            now,
        )
    }
    fn accept(
        &mut self,
        system: u32,
        component: u8,
        signature: &[u8; 13],
        now: u64,
    ) -> Result<(), Error> {
        let mut timestamp = [0; 8];
        timestamp[..6].copy_from_slice(&signature[1..7]);
        let timestamp = u64::from_le_bytes(timestamp);
        let stream = ReplayStream {
            system,
            component,
            link: signature[0],
            timestamp,
        };
        for previous in self.streams.iter_mut().flatten() {
            if (previous.system, previous.component, previous.link)
                == (stream.system, stream.component, stream.link)
            {
                if timestamp <= previous.timestamp {
                    return Err(Error::Replay);
                }
                *previous = stream;
                return Ok(());
            }
        }
        if timestamp.saturating_add(6_000_000) < now {
            return Err(Error::StaleTimestamp);
        }
        let slot = self
            .streams
            .iter_mut()
            .find(|slot| slot.is_none())
            .ok_or(Error::ReplayTableFull)?;
        *slot = Some(stream);
        Ok(())
    }
}
