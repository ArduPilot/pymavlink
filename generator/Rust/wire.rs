//! Allocation-free framing, routing and MAVLink 2 signing.
//!
//! Payloads stay opaque: routing needs only `WireInfo`, never message decoding.
//! Parsing checks CRCs but does not authenticate signatures. Use `verify_signature`
//! or `ReplayGuard::verify` before trusting signed traffic.
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
    pub fn from_message<M: WireMessage>(
        version: MavLinkVersion,
        sequence: u8,
        system_id: u32,
        component_id: u8,
        message: &M,
    ) -> Result<Self, Error> {
        let payload = message.encode(version)?;
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
    pub fn decode<D: Dialect>(&self) -> Result<D, SpecError> {
        D::decode(&self.to_payload())
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
        let (header_len, total) = frame_lengths(bytes)?;
        if bytes.len() != total {
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
        let info = lookup(id).ok_or(Error::UnknownMessage(id))?;
        Self::validate_info(info)?;
        if info.id != id {
            return Err(Error::InvalidMetadata);
        }
        let length = bytes[1] as usize;
        Self::validate_length(version, length, info)?;
        let end = header_len + length;
        if crc(&bytes[1..end], info.crc_extra) != u16::from_le_bytes([bytes[end], bytes[end + 1]]) {
            return Err(Error::BadCrc);
        }
        let mut payload = [0; 255];
        payload[..length].copy_from_slice(&bytes[header_len..end]);
        let signature = if flags & SIGNED != 0 {
            Some(bytes[end + 2..].try_into().unwrap())
        } else {
            None
        };
        Ok(Self {
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
        })
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
        let hash = Sha256::new()
            .chain_update(key)
            .chain_update(&bytes[..length - 6])
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
        let signature = frame.signature.unwrap();
        let mut timestamp = [0; 8];
        timestamp[..6].copy_from_slice(&signature[1..7]);
        let timestamp = u64::from_le_bytes(timestamp);
        let stream = ReplayStream {
            system: frame.system_id,
            component: frame.component_id,
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
