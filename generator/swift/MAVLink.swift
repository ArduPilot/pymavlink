import Foundation

/// Common protocol for all MAVLink entities which describes types
/// metadata properties.
public protocol MAVLinkEntity: CustomStringConvertible, CustomDebugStringConvertible {
    
    /// Original MAVLink enum name (from declarations xml)
    static var typeName: String { get }
    
    /// Compact type description
    static var typeDescription: String { get }
    
    /// Verbose type description
    static var typeDebugDescription: String { get }
}

// MARK: - Enumeration protocol

/// Enumeration protocol description with common for all MAVLink enums
/// properties requirements.
public protocol Enumeration: RawRepresentable, Equatable, MAVLinkEntity {
    
    /// Array with all members of current enum
    static var allMembers: [Self] { get }
    
    // Array with `Name` - `Description` tuples (values from declarations xml file)
    static var membersDescriptions: [(String, String)] { get }
    
    /// `ENUM_END` flag for checking if enum case value is valid
    static var enumEnd: UInt { get }
    
    /// Original MAVLinks enum member name (as declared in definition's xml file)
    var memberName: String { get }
    
    /// Specific member description from definitions xml
    var memberDescription: String { get }
}

/// Enumeration protocol default behaviour implementation.
extension Enumeration {
    public static var typeDebugDescription: String {
        let cases = allMembers.map({ $0.debugDescription }).joined(separator: "\\n\\t")
        return "Enum \(typeName): \(typeDescription)\\nMembers:\\n\\t\(cases)"
    }
    
    public var description: String {
        return memberName
    }
    
    public var debugDescription: String {
        return "\(memberName): \(memberDescription)"
    }
    
    public var memberName: String {
        return Self.membersDescriptions[Self.allMembers.index(of: self)!].0
    }
    
    public var memberDescription: String {
        return Self.membersDescriptions[Self.allMembers.index(of: self)!].1
    }
}

// MARK: - MAVLinkBitmask protocol

public protocol MAVLinkBitmask: OptionSet, MAVLinkEntity {
    /// Array with all members of current bitmask
    static var allMembers: [Self.Element] { get }

    // Array with `Name` - `Description` tuples (values from declarations xml file)
    static var membersDescriptions: [(String, String)] { get }

    /// `ENUM_END` flag for checking if enum case value is valid
    static var enumEnd: UInt { get }

    /// Original MAVLinks enum member name (as declared in definition's xml file)
    var usedMemberName: [String] { get }

    /// Specific member description from definitions xml
    var usedMemberDescriptions: [String] { get }
}

/// MAVLinkBitmask protocol default behaviour implementation.
extension MAVLinkBitmask {
    public static var typeDebugDescription: String {
        let cases = membersDescriptions.map { "\($0.0): \($0.1)" }.joined(separator: "\\n\\t")
        return "Bitmask \(typeName): \(typeDescription)\\nMembers:\\n\\t\(cases)"
    }

    public var description: String {
        return metadataForUsedMembers().map { $0.1 }.joined(separator:", ")
    }

    public var debugDescription: String {
        let usedValuesExplained = metadataForUsedMembers().map {
            "\($0.0): \($0.1)"
            }.joined(separator: "\n")

        return usedValuesExplained
    }

    public var usedMemberName: [String] {
        return metadataForUsedMembers().map { $0.0 }
    }

    public var usedMemberDescriptions: [String] {
        return metadataForUsedMembers().map { $0.1 }
    }

    private func metadataForUsedMembers() -> [(String, String)] {
        return zip(Self.allMembers, Self.membersDescriptions).filter {
                self.contains($0.0)
            }.map {
                $0.1
            }
    }
}

// MARK: - Message protocol

/// Message field definition tuple.
public typealias FieldDefinition = (name: String, offset: Int, type: String, length: UInt, description: String)

/// Message protocol describes all common MAVLink messages properties and
/// methods requirements.
public protocol Message: MAVLinkEntity {
    static var id: UInt32 { get }
    
    static var payloadLength: UInt8 { get }
    
    /// Array of tuples with field definition info
    static var fieldDefinitions: [FieldDefinition] { get }
    
    /// All field's names and values of current Message
    var allFields: [(String, Any)] { get }
    
    /// Initialize Message from received data.
    ///
    /// - parameter data: Data to decode.
    ///
    /// - throws: Throws `ParseError` or `ParseEnumError` if any parsing errors
    /// occur.
    init(data: Data) throws
    
    /// Returns `Data` representation of current `Message` struct guided
    /// by format from `fieldDefinitions`.
    ///
    /// - throws: Throws `PackError` if any of message fields do not comply
    /// format from `fieldDefinitions`.
    ///
    /// - returns: Receiver's `Data` representation
    func pack() throws -> Data
    var mavlinkTargetSystem: UInt32 { get }
    mutating func setMavlinkTargetSystem(_ value: UInt32)
}

/// Message protocol default behaviour implementation.
extension Message {
    public var mavlinkTargetSystem: UInt32 { return 0 }
    public mutating func setMavlinkTargetSystem(_ value: UInt32) { }

    public static var payloadLength: UInt8 {
        return messageLengths[id] ?? Packet.Constant.maxPayloadLength
    }
    
    public static var typeDebugDescription: String {
        let fields = fieldDefinitions.map({ "\($0.name): \($0.type): \($0.description)" }).joined(separator: "\n\t")
        return "Struct \(typeName): \(typeDescription)\nFields:\n\t\(fields)"
    }
    
    public var description: String {
        let describeField: ((String, Any)) -> String = { (arg) in
            let (name, value) = arg
            let valueString = value is String ? "\"\(value)\"" : value
            return "\(name): \(valueString)"
        }
        let fieldsDescription = allFields.map(describeField).joined(separator: ", ")
        return "\(type(of: self))(\(fieldsDescription))"
    }
    
    public var debugDescription: String {
        let describeFieldVerbose: ((String, Any)) -> String = { (arg) in
            let (name, value) = arg
            let valueString = value is String ? "\"\(value)\"" : value
            let (_, _, _, _, description) = Self.fieldDefinitions.filter { $0.name == name }.first!
            return "\(name) = \(valueString) : \(description)"
        }
        let fieldsDescription = allFields.map(describeFieldVerbose).joined(separator: "\n\t")
        return "\(Self.typeName): \(Self.typeDescription)\nFields:\n\t\(fieldsDescription)"
    }
    
    public var allFields: [(String, Any)] {
        var result: [(String, Any)] = []
        let mirror = Mirror(reflecting: self)
        for case let (label?, value) in mirror.children {
            result.append((label, value))
        }
        return result
    }
}

// MARK: - Type aliases

public typealias Channel = UInt8

// MARK: - Errors

public protocol MAVLinkError: Error, CustomStringConvertible, CustomDebugStringConvertible { }

// MARK: Parsing error enumeration

/// Parsing errors
public enum ParseError: MAVLinkError {
    
    /// Size of expected number is larger than receiver's data length.
    /// - offset:     Expected number offset in received data.
    /// - size:       Expected number size in bytes.
    /// - upperBound: The number of bytes in the data.
    case valueSizeOutOfBounds(offset: Int, size: Int, upperBound: Int)
    
    /// Data contains non ASCII characters.
    /// - offset: String offset in received data.
    /// - length: Expected length of string to read.
    case invalidStringEncoding(offset: Int, length: Int)
    
    /// Length check of payload for known `messageId` did fail.
    /// - messageId:      Id of expected `Message` type.
    /// - receivedLength: Received payload length.
    /// - properLength:   Expected payload length for `Message` type.
    case invalidPayloadLength(messageId: UInt32, receivedLength: UInt8, expectedLength: UInt8)
    
    /// Received `messageId` was not recognized so we can't create appropriate
    /// `Message`.
    /// - messageId: Id of the message that was not found in the known message
    /// list (`messageIdToClass` array).
    case unknownMessageId(messageId: UInt32)
    
    /// Checksum check failed. Message id is known but calculated CRC bytes
    /// do not match received CRC value.
    /// - messageId: Id of expected `Message` type.
    case badCRC(messageId: UInt32)
}

extension ParseError {
    
    /// Textual representation used when written to output stream.
    public var description: String {
        switch self {
        case .valueSizeOutOfBounds:
            return "ParseError.valueSizeOutOfBounds"
        case .invalidStringEncoding:
            return "ParseError.invalidStringEncoding"
        case .invalidPayloadLength:
            return "ParseError.invalidPayloadLength"
        case .unknownMessageId:
            return "ParseError.unknownMessageId"
        case .badCRC:
            return "ParseError.badCRC"
        }
    }
    
    /// Debug textual representation used when written to output stream, which
    /// includes all associated values and their labels.
    public var debugDescription: String {
        switch self {
        case let .valueSizeOutOfBounds(offset, size, upperBound):
            return "ParseError.valueSizeOutOfBounds(offset: \(offset), size: \(size), upperBound: \(upperBound))"
        case let .invalidStringEncoding(offset, length):
            return "ParseError.invalidStringEncoding(offset: \(offset), length: \(length))"
        case let .invalidPayloadLength(messageId, receivedLength, expectedLength):
            return "ParseError.invalidPayloadLength(messageId: \(messageId), receivedLength: \(receivedLength), expectedLength: \(expectedLength))"
        case let .unknownMessageId(messageId):
            return "ParseError.unknownMessageId(messageId: \(messageId))"
        case let .badCRC(messageId):
            return "ParseError.badCRC(messageId: \(messageId))"
        }
    }
}

// MARK: Parsing enumeration error

/// Special error type for returning Enum parsing errors with details in associated
/// values (types of these values are not compatible with `ParseError` enum).
public enum ParseEnumError<T: RawRepresentable>: MAVLinkError {
    
    /// Enumeration case with `rawValue` at `valueOffset` was not found in
    /// `enumType` enumeration.
    /// - enumType: Type of expected enumeration.
    /// - rawValue: Raw value that was not found in `enumType`.
    /// - valueOffset: Value offset in received payload data.
    case unknownValue(enumType: T.Type, rawValue: T.RawValue, valueOffset: Int)
}

extension ParseEnumError {
    
    /// Textual representation used when written to the output stream.
    public var description: String {
        switch self {
        case .unknownValue:
            return "ParseEnumError.unknownValue"
        }
    }
    
    /// Debug textual representation used when written to the output stream, which
    /// includes all associated values and their labels.
    public var debugDescription: String {
        switch self {
        case let .unknownValue(enumType, rawValue, valueOffset):
            return "ParseEnumError.unknownValue(enumType: \(enumType), rawValue: \(rawValue), valueOffset: \(valueOffset))"
        }
    }
}

// MARK: Packing errors

/// Errors that can occur while packing `Message` for sending.
public enum PackError: MAVLinkError {
    
    /// Size of received value (together with offset) is out of receiver's length.
    /// - offset:     Expected value offset in payload.
    /// - size:       Provided field value size in bytes.
    /// - upperBound: Available payload length.
    case valueSizeOutOfBounds(offset: Int, size: Int, upperBound: Int)
    
    /// Length check for provided field value did fail.
    /// - offset:              Expected value offset in payload.
    /// - providedValueLength: Count of elements (characters) in provided value.
    /// - allowedLength:       Maximum number of elements (characters) allowed in field.
    case invalidValueLength(offset: Int, providedValueLength: Int, allowedLength: Int)
    
    /// String field contains non ASCII characters.
    /// - offset: Expected value offset in payload.
    /// - string: Original string.
    case invalidStringEncoding(offset: Int, string: String)
    
    /// CRC extra byte not found for provided `messageId` type.
    /// - messageId: Id of message type.
    case crcExtraNotFound(messageId: UInt32)
    
    /// Packet finalization process failed due to `message` absence.
    case unsupportedProtocol
    case messageNotSet
}

extension PackError {
    
    /// Textual representation used when written to the output stream.
    public var description: String {
        switch self {
        case .valueSizeOutOfBounds:
            return "PackError.valueSizeOutOfBounds"
        case .invalidValueLength:
            return "PackError.invalidValueLength"
        case .invalidStringEncoding:
            return "PackError.invalidStringEncoding"
        case .crcExtraNotFound:
            return "PackError.crcExtraNotFound"
        case .unsupportedProtocol:
            return "PackError.unsupportedProtocol"
        case .messageNotSet:
            return "PackError.messageNotSet"
        }
    }
    
    /// Debug textual representation used when written to the output stream, which
    /// includes all associated values and their labels.
    public var debugDescription: String {
        switch self {
        case let .valueSizeOutOfBounds(offset, size, upperBound):
            return "PackError.valueSizeOutOfBounds(offset: \(offset), size: \(size), upperBound: \(upperBound))"
        case let .invalidValueLength(offset, providedValueLength, allowedLength):
            return "PackError.invalidValueLength(offset: \(offset), providedValueLength: \(providedValueLength), allowedLength: \(allowedLength))"
        case let .invalidStringEncoding(offset, string):
            return "PackError.invalidStringEncoding(offset: \(offset), string: \(string))"
        case let .crcExtraNotFound(messageId):
            return "PackError.crcExtraNotFound(messageId: \(messageId))"
        case .unsupportedProtocol:
            return "PackError.unsupportedProtocol"
        case .messageNotSet:
            return "PackError.messageNotSet"
        }
    }
}

// MARK: - Delegate protocol

/// Alternative way to receive parsed Messages, finalized packet's data and all
/// errors is to implement this protocol and set as `MAVLink`'s delegate.
public protocol MAVLinkDelegate: class {
    
    /// Called when MAVLink packet is successfully received, payload length
    /// and CRC checks are passed.
    ///
    /// - parameter packet:  Completely received `Packet`.
    /// - parameter channel: Channel on which `packet` was received.
    /// - parameter link:    `MAVLink` object that handled `packet`.
    func didReceive(packet: Packet, on channel: Channel, via link: MAVLink)
    
    /// Packet receiving failed due to `InvalidPayloadLength` or `BadCRC` error.
    ///
    /// - parameter packet:    Partially received `Packet`.
    /// - parameter error:     Error that  occurred while receiving `data`
    /// (`InvalidPayloadLength` or `BadCRC` error).
    /// - parameter channel:   Channel on which `packet` was received.
    /// - parameter link:      `MAVLink` object that received `data`.
    func didFailToReceive(packet: Packet?, with error: MAVLinkError, on channel: Channel, via link: MAVLink)
    
    /// Called when received data was successfully parsed into appropriate
    /// `message` structure.
    ///
    /// - parameter message: Successfully parsed `Message`.
    /// - parameter packet:  Completely received `Packet`.
    /// - parameter channel: Channel on which `message` was received.
    /// - parameter link:    `MAVLink` object that handled `packet`.
    func didParse(message: Message, from packet: Packet, on channel: Channel, via link: MAVLink)
    
    /// Called when `packet` completely received but `MAVLink` was not able to
    /// finish `Message` processing due to unknown `messageId` or type validation
    /// errors.
    ///
    /// - parameter packet:  Completely received `Packet`.
    /// - parameter error:   Error that  occurred while parsing `packet`'s
    /// payload into `Message`.
    /// - parameter channel: Channel on which `message` was received.
    /// - parameter link:    `MAVLink` object that handled `packet`.
    func didFailToParseMessage(from packet: Packet, with error: MAVLinkError, on channel: Channel, via link: MAVLink)
    
    /// Called when message is finalized and ready for sending to aircraft.
    ///
    /// - parameter message: Message to be sent.
    /// - parameter data:    Compiled data that represents `message`.
    /// - parameter channel: Channel on which `message` should be sent.
    /// - parameter link:    `MAVLink` object that handled `message`.
    func didFinalize(message: Message, from packet: Packet, to data: Data, on channel: Channel, in link: MAVLink)
}


/// MAVLink signing state. Use a separate instance for each outgoing link.
/// Unsigned incoming frames remain accepted unless requireSigned is enabled.
public final class Signing {
    public var secretKey: Data
    public var linkId: UInt8
    public var timestamp: UInt64
    public var signOutgoing = true
    public var requireSigned = false
    fileprivate var streams: [String: UInt64] = [:]
    public init(secretKey: Data, linkId: UInt8 = 0, timestamp: UInt64 = 0) {
        self.secretKey = secretKey; self.linkId = linkId; self.timestamp = timestamp
    }
    fileprivate func signature(_ data: Data) -> Data {
        return Data(mavlinkSHA256(secretKey + data).prefix(6))
    }
}

private func mavlinkSHA256(_ data: Data) -> Data {
    let k: [UInt32] = [0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2]
    var h: [UInt32] = [0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19]
    var bytes = Array(data)
    let bits = UInt64(bytes.count) * 8
    bytes.append(0x80)
    while bytes.count % 64 != 56 { bytes.append(0) }
    for i in (0..<8).reversed() { bytes.append(UInt8(truncatingIfNeeded: bits >> (i * 8))) }
    func rotate(_ v: UInt32, _ n: UInt32) -> UInt32 { return (v >> n) | (v << (32 - n)) }
    for block in stride(from: 0, to: bytes.count, by: 64) {
        var w = [UInt32](repeating: 0, count: 64)
        for i in 0..<16 {
            for j in 0..<4 { w[i] = (w[i] << 8) | UInt32(bytes[block + 4 * i + j]) }
        }
        for i in 16..<64 {
            let a = rotate(w[i-15],7) ^ rotate(w[i-15],18) ^ (w[i-15] >> 3)
            let b = rotate(w[i-2],17) ^ rotate(w[i-2],19) ^ (w[i-2] >> 10)
            w[i] = w[i-16] &+ a &+ w[i-7] &+ b
        }
        var a=h[0], b=h[1], c=h[2], d=h[3], e=h[4], f=h[5], g=h[6], z=h[7]
        for i in 0..<64 {
            let t1 = z &+ (rotate(e,6) ^ rotate(e,11) ^ rotate(e,25)) &+ ((e & f) ^ (~e & g)) &+ k[i] &+ w[i]
            let t2 = (rotate(a,2) ^ rotate(a,13) ^ rotate(a,22)) &+ ((a & b) ^ (a & c) ^ (b & c))
            z=g; g=f; f=e; e=d &+ t1; d=c; c=b; b=a; a=t1 &+ t2
        }
        for (i,v) in [a,b,c,d,e,f,g,z].enumerated() { h[i] = h[i] &+ v }
    }
    var result = Data()
    for word in h { for i in (0..<4).reversed() { result.append(UInt8(truncatingIfNeeded: word >> (i*8))) } }
    return result
}

// MARK: - Classes implementations

/// Main MAVLink class, performs `Packet` receiving, recognition, validation,
/// `Message` structure creation and `Message` packing, finalizing for sending.
/// Also returns errors through delegation if any errors occurred.
/// Supports MAVLink 1 and MAVLink 2, including extended system IDs.
public class MAVLink {
    
    /// States for the parsing state machine.
    enum ParseState {
        case uninit
        case idle
        case gotStx
        case gotSequence
        case gotLength
        case gotSystemId
        case gotComponentId
        case gotMessageId
        case gotPayload
        case gotCRC1
        case gotBadCRC1
        case unsupportedLength
        case unsupportedFlags
        case discardUnsupported
    }
    
    enum Framing: UInt8 {
        case incomplete = 0
        case ok = 1
        case badCRC = 2
    }
    
    /// Storage for MAVLink parsed packets count, states and errors statistics.
    class Status {
        
        /// Number of received packets
        var packetReceived: Framing = .incomplete
        
        /// Number of parse errors
        var parseError: UInt8 = 0
        
        /// Parsing state machine
        var parseState: ParseState = .uninit
        var frame = Data()
        
        /// Sequence number of the last received packet
        var currentRxSeq: UInt8 = 0
        
        /// Sequence number of the last sent packet
        var currentTxSeq: UInt8 = 0
        
        /// Received packets
        var packetRxSuccessCount: UInt16 = 0
        
        /// Number of packet drops
        var packetRxDropCount: UInt16 = 0
    }
    
    /// MAVLink Packets and States buffers
    let channelBuffers = (0 ... Channel.max).map({ _ in Packet() })
    let channelStatuses = (0 ... Channel.max).map({ _ in Status() })
    
    /// Object to pass received packets, messages, errors, finalized data to.
    public weak var delegate: MAVLinkDelegate?
    
    /// Enable this option to check the length of each message. This allows
    /// invalid messages to be caught much sooner. Use it if the transmission
    /// medium is prone to missing (or extra) characters (e.g. a radio that
    /// fades in and out). Use only if the channel will contain message
    /// types listed in the headers.
    public var checkMessageLength = true
    
    /// Use one extra CRC that is added to the message CRC to detect mismatches
    /// in the message specifications. This is to prevent that two devices using
    /// different message versions incorrectly decode a message with the same
    /// length. Defined as `let` as we support only the latest version (1.0) of
    /// the MAVLink wire protocol.
    public let crcExtra = true
    /// Select the transmit wire version. Reception accepts either version.
    public var mavlink2 = defaultMavlink2
    public private(set) var unsupportedFrames = 0
    public private(set) var badSignatures = 0
    public var signing: Signing?
    
    public init() { }
    
    /// This is a convenience function which handles the complete MAVLink
    /// parsing. The function will parse one byte at a time and return the
    /// complete packet once it could be successfully decoded. Checksum and
    /// other failures will be delegated to `delegate`.
    ///
    /// - parameter char:    The char to parse.
    /// - parameter channel: Id of the current channel. This allows to parse
    /// different channels with this function. A channel is not a physical
    /// message channel like a serial port, but a logic partition of the
    /// communication streams in this case.
    ///
    /// - returns: Returns `nil` if packet could be decoded at the moment,
    /// the `Packet` structure else.
    public func parse(char: UInt8, channel: Channel) -> Packet? {

        let status = channelStatuses[Int(channel)]
        if status.frame.isEmpty && char != 0xfe && char != 0xfd { return nil }
        status.frame.append(char)
        guard status.frame.count >= 3 else { return nil }
        let v2 = status.frame[0] == 0xfd
        let flags: UInt8 = v2 ? status.frame[2] : 0
        let headerLength = v2 ? 10 + (flags & 2 != 0 ? 3 : 0) + (flags & 4 != 0 ? 4 : 0) : 6
        let payloadLength = Int(status.frame[1])
        let crcOffset = headerLength + payloadLength
        let frameLength = crcOffset + 2 + (flags & 1 != 0 ? 13 : 0)
        guard status.frame.count == frameLength else { return nil }
        let bytes = status.frame
        status.frame = Data()
        guard flags & ~UInt8(7) == 0 else { unsupportedFrames += 1; return nil }
        let packet = Packet()
        packet.magic = bytes[0]
        packet.channel = channel
        packet.length = bytes[1]
        packet.incompatFlags = flags
        packet.compatFlags = v2 ? bytes[3] : 0
        packet.sequence = bytes[v2 ? 4 : 2]
        var offset = v2 ? 5 : 3
        func read(_ length: Int) -> UInt32 {
            var value: UInt32 = 0
            for i in 0..<length { value |= UInt32(bytes[offset + i]) << (i * 8) }
            offset += length
            return value
        }
        packet.systemId = read(flags & 2 != 0 ? 4 : 1)
        packet.componentId = UInt8(read(1))
        packet.messageId = read(v2 ? 3 : 1)
        if flags & 4 != 0 { packet.targetSystem = read(4) }
        packet.payload = bytes.subdata(in: headerLength..<crcOffset)
        guard let extra = messageCRCsExtra[packet.messageId] else {
            delegate?.didFailToParseMessage(from: packet, with: ParseError.unknownMessageId(messageId: packet.messageId), on: channel, via: self)
            return nil
        }
        packet.checksum.accumulate(bytes[1..<crcOffset])
        packet.checksum.accumulate(extra)
        guard bytes[crcOffset] == packet.checksum.lowByte && bytes[crcOffset + 1] == packet.checksum.highByte else {
            delegate?.didFailToReceive(packet: packet, with: ParseError.badCRC(messageId: packet.messageId), on: channel, via: self)
            return nil
        }
        if checkMessageLength && !v2 && packet.length != messageMinimumLengths[packet.messageId] {
            delegate?.didFailToReceive(packet: packet, with: ParseError.invalidPayloadLength(messageId: packet.messageId, receivedLength: packet.length, expectedLength: messageMinimumLengths[packet.messageId]!), on: channel, via: self)
            return nil
        }
        if flags & 1 != 0 {
            guard let sign = signing, sign.secretKey.count == 32 else { badSignatures += 1; return nil }
            let sigOffset = crcOffset + 2
            var timestamp: UInt64 = 0
            for i in 0..<6 { timestamp |= UInt64(bytes[sigOffset + 1 + i]) << (i * 8) }
            let stream = "\(packet.systemId):\(packet.componentId):\(bytes[sigOffset])"
            let previous = sign.streams[stream]
            let tag = sign.signature(Data(bytes.prefix(sigOffset + 7)))
            var difference: UInt8 = 0
            for i in 0..<6 { difference |= tag[i] ^ bytes[sigOffset + 7 + i] }
            guard difference == 0 && (previous == nil || timestamp > previous!) &&
                (previous != nil || sign.timestamp <= timestamp || sign.timestamp - timestamp <= 6000000) else {
                badSignatures += 1; return nil
            }
            sign.streams[stream] = timestamp
            sign.timestamp = max(sign.timestamp, timestamp)
        } else if signing?.requireSigned == true { badSignatures += 1; return nil }
        status.currentRxSeq = packet.sequence
        status.packetRxSuccessCount = status.packetRxSuccessCount &+ 1
        delegate?.didReceive(packet: packet, on: channel, via: self)
        if let messageClass = messageIdToClass[packet.messageId] {
            do {
                var payload = packet.payload
                let expected = Int(messageClass.payloadLength)
                if payload.count < expected { payload.append(Data(count: expected - payload.count)) }
                var message = try messageClass.init(data: payload)
                if let target = packet.targetSystem { message.setMavlinkTargetSystem(target) }
                packet.message = message
                delegate?.didParse(message: message, from: packet, on: channel, via: self)
            } catch {
                delegate?.didFailToParseMessage(from: packet, with: error as! MAVLinkError, on: channel, via: self)
            }
        }
        return packet
    }
    
    /// Parse new portion of data, then call `messageHandler` if new message
    /// is available.
    ///
    /// - parameter data:           Data to be parsed.
    /// - parameter channel:        Id of the current channel. This allows to
    /// parse different channels with this function. A channel is not a physical
    /// message channel like a serial port, but a logic partition of the
    /// communication streams in this case.
    /// - parameter messageHandler: The message handler to call when the
    /// provided data is enough to complete message parsing. Unless you have
    /// provided a custom delegate, this parameter must not be `nil`, because
    /// there is no other way to retrieve the parsed message and packet.
    public func parse(data: Data, channel: Channel, messageHandler: ((Message, Packet) -> Void)? = nil) {
        data.forEach { byte in
            if let packet = parse(char: byte, channel: channel), let message = packet.message, let messageHandler = messageHandler {
                messageHandler(message, packet)
            }
        }
    }
    
    /// Prepare `message` bytes for sending, pass to `delegate` for further
    /// processing and increase sequence counter.
    ///
    /// - parameter message:     Message to be compiled into bytes and sent.
    /// - parameter systemId:    Id of the sending (this) system.
    /// - parameter componentId: Id of the sending component.
    /// - parameter channel:     Id of the current channel.
    ///
    /// - throws: Throws `PackError`.
    public func dispatch(message: Message, systemId: UInt32, componentId: UInt8, channel: Channel) throws {
        let channelStatus = channelStatuses[Int(channel)]
        let packet = Packet(message: message, systemId: systemId, componentId: componentId, channel: channel)
        let data = try packet.finalize(sequence: channelStatus.currentTxSeq, mavlink2: mavlink2, signing: signing)
        delegate?.didFinalize(message: message, from: packet, to: data, on: channel, in: self)
        channelStatus.currentTxSeq = channelStatus.currentTxSeq &+ 1
    }
}

/// MAVLink Packet structure to store received data that is not full message yet.
/// Contains additional to Message info like channel, system id, component id
/// and raw payload data, etc. Also used to store and transfer received data of
/// unknown or corrupted Messages.
/// [More details](https://mavlink.io/en).
public class Packet {
    
    /// MAVlink Packet constants
    struct Constant {
        
        /// Maximum packets payload length
        static let maxPayloadLength = UInt8.max
        
        static let numberOfChecksumBytes = 2
        
        /// Length of core header (of the comm. layer): message length
        /// (1 byte) + message sequence (1 byte) + message system id (1 byte) +
        /// message component id (1 byte) + message type id (1 byte).
        static let coreHeaderLength = 5
        
        /// Length of all header bytes, including core and checksum
        static let numberOfHeaderBytes = Constant.numberOfChecksumBytes + Constant.coreHeaderLength + 1
        
        /// Packet start sign. Indicates the start of a new packet. v1.0.
        static let packetStx: UInt8 = 0xFE
    }
    
    /// Channel on which packet was received
    public internal(set) var channel: UInt8 = 0
    
    /// Sent at the end of packet
    public internal(set) var checksum = Checksum()
    
    /// Protocol magic marker (PacketStx value)
    public internal(set) var magic: UInt8 = 0
    
    /// Length of payload
    public internal(set) var length: UInt8 = 0
    
    /// Sequence of packet
    public internal(set) var sequence: UInt8 = 0
    
    /// Id of message sender system/aircraft
    public internal(set) var systemId: UInt32 = 0
    
    /// Id of the message sender component
    public internal(set) var componentId: UInt8 = 0
    
    /// Id of message type in payload
    public internal(set) var messageId: UInt32 = 0
    
    public internal(set) var incompatFlags: UInt8 = 0
    public internal(set) var compatFlags: UInt8 = 0
    public internal(set) var targetSystem: UInt32?

    /// Message bytes
    public internal(set) var payload = Data(capacity: Int(Constant.maxPayloadLength) + Constant.numberOfChecksumBytes)
    
    /// Received Message structure if available
    public internal(set) var message: Message?
    
    /// Initialize copy of provided Packet.
    ///
    /// - parameter packet: Packet to copy
    init(packet: Packet) {
        channel = packet.channel
        checksum = packet.checksum
        magic = packet.magic
        length = packet.length
        sequence = packet.sequence
        systemId = packet.systemId
        componentId = packet.componentId
        messageId = packet.messageId
        incompatFlags = packet.incompatFlags
        compatFlags = packet.compatFlags
        targetSystem = packet.targetSystem
        payload = packet.payload
        message = packet.message
    }
    
    /// Initialize packet with provided `message` for sending.
    ///
    /// - parameter message:     Message to send.
    /// - parameter systemId:    Id of the sending (this) system.
    /// - parameter componentId: Id of the sending component.
    /// - parameter channel:     Id of the current channel.
    public init(message: Message, systemId: UInt32, componentId: UInt8, channel: Channel) {
        self.magic = Constant.packetStx
        self.systemId = systemId
        self.componentId = componentId
        self.messageId = type(of: message).id
        self.length = type(of: message).payloadLength
        self.message = message
        self.channel = channel
    }
    
    init() { }
    
    /// Finalize a MAVLink packet with sequence assignment. Returns data that
    /// could be sent to the aircraft. This function calculates the checksum and
    /// sets length and aircraft id correctly. It assumes that the packet is
    /// already correctly initialized with appropriate `message`, `length`,
    /// `systemId`, `componentId`.
    /// Could be used to send packets without `MAVLink` object, in this case you
    /// should take care of `sequence` counter manually.
    ///
    /// - parameter sequence: Each channel counts up its send sequence. It allows
    /// to detect packet loss.
    ///
    /// - throws: Throws `PackError`.
    ///
    /// - returns: Data
    public func finalize(sequence: UInt8, mavlink2: Bool? = nil, signing: Signing? = nil) throws -> Data {
        guard let message = message else { throw PackError.messageNotSet }
        guard let crcExtra = messageCRCsExtra[messageId] else { throw PackError.crcExtraNotFound(messageId: messageId) }
        let v2 = mavlink2 ?? defaultMavlink2
        let target = message.mavlinkTargetSystem
        if !v2 && (systemId > 255 || target > 255 || messageId > 255) { throw PackError.unsupportedProtocol }
        self.sequence = sequence
        magic = v2 ? 0xfd : 0xfe
        incompatFlags = v2 ? (systemId > 255 ? 2 : 0) | (target > 255 ? 4 : 0) : 0
        if let sign = signing, sign.signOutgoing {
            guard v2 && sign.secretKey.count == 32 && sign.timestamp < (1 << 48) else { throw PackError.unsupportedProtocol }
            incompatFlags |= 1
        }
        targetSystem = target > 255 ? target : nil
        payload = try message.pack()
        if v2 {
            while payload.count > 1 && payload.last == 0 { payload.removeLast() }
        } else {
            payload = Data(payload.prefix(Int(messageMinimumLengths[messageId]!)))
        }
        length = UInt8(payload.count)
        var bytes = Data([magic, length])
        if v2 { bytes.append(contentsOf: [incompatFlags, 0]) }
        bytes.append(sequence)
        func append(_ value: UInt32, _ count: Int) {
            for i in 0..<count { bytes.append(UInt8(truncatingIfNeeded: value >> (i * 8))) }
        }
        append(systemId, incompatFlags & 2 != 0 ? 4 : 1)
        bytes.append(componentId)
        append(messageId, v2 ? 3 : 1)
        if incompatFlags & 4 != 0 { append(target, 4) }
        bytes.append(payload)
        checksum.start()
        checksum.accumulate(bytes.dropFirst())
        checksum.accumulate(crcExtra)
        bytes.append(contentsOf: [checksum.lowByte, checksum.highByte])
        if incompatFlags & 1 != 0, let sign = signing {
            bytes.append(sign.linkId)
            for i in 0..<6 { bytes.append(UInt8(truncatingIfNeeded: sign.timestamp >> (i * 8))) }
            bytes.append(sign.signature(bytes))
            sign.timestamp += 1
        }
        return bytes
    }

}

/// Struct for storing and calculating checksum.
public struct Checksum {
    
    struct Constants {
        static let x25InitCRCValue: UInt16 = 0xFFFF
    }
    
    public var lowByte: UInt8 {
        return UInt8(truncatingIfNeeded: value)
    }
    
    public var highByte: UInt8 {
        return UInt8(truncatingIfNeeded: value >> 8)
    }
    
    public private(set) var value: UInt16 = 0
    
    init() {
        start()
    }
    
    /// Initialize the buffer for the MCRF4XX CRC.
    mutating func start() {
        value = Constants.x25InitCRCValue
    }
    
    /// Accumulate the MCRF4XX CRC by adding one char at a time. The checksum
    /// function adds the hash of one char at a time to the 16 bit checksum
    /// `value` (`UInt16`).
    ///
    /// - parameter char: New char to hash
    mutating func accumulate(_ char: UInt8) {
        var tmp: UInt8 = char ^ UInt8(truncatingIfNeeded: value)
        tmp ^= (tmp << 4)
        value = (UInt16(value) >> 8) ^ (UInt16(tmp) << 8) ^ (UInt16(tmp) << 3) ^ (UInt16(tmp) >> 4)
    }
    
    /// Accumulate the MCRF4XX CRC by adding `buffer` bytes.
    ///
    /// - parameter buffer: Sequence of bytes to hash
    mutating func accumulate<T: Sequence>(_ buffer: T) where T.Iterator.Element == UInt8 {
        buffer.forEach { accumulate($0) }
    }
}

// MARK: - CF independent host system byte order determination

public enum ByteOrder: UInt32 {
    case unknown
    case littleEndian
    case bigEndian
}

public func hostByteOrder() -> ByteOrder {
    var bigAndLittleEndian: UInt32 = (ByteOrder.bigEndian.rawValue << 24) | ByteOrder.littleEndian.rawValue
    
    let firstByte: UInt8 = withUnsafePointer(to: &bigAndLittleEndian) { numberPointer in
        let bufferPointer = numberPointer.withMemoryRebound(to: UInt8.self, capacity: 4) { pointer in
            return UnsafeBufferPointer(start: pointer, count: 4)
        }
        return bufferPointer[0]
    }
    
    return ByteOrder(rawValue: UInt32(firstByte)) ?? .unknown
}

// MARK: - Data extensions

protocol MAVLinkNumber { }

extension UInt8: MAVLinkNumber { }

extension Int8: MAVLinkNumber { }

extension UInt16: MAVLinkNumber { }

extension Int16: MAVLinkNumber { }

extension UInt32: MAVLinkNumber { }

extension Int32: MAVLinkNumber { }

extension UInt64: MAVLinkNumber { }

extension Int64: MAVLinkNumber { }

extension Float: MAVLinkNumber { }

extension Double: MAVLinkNumber { }

/// Methods for getting properly typed field values from received data.
extension Data {
    
    /// Returns number value (integer or floating point) from receiver's data.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter byteOrder: Current system endianness.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: Returns `MAVLinkNumber` (UInt8, Int8, UInt16, Int16, UInt32,
    /// Int32, UInt64, Int64, Float, Double).
    func number<T: MAVLinkNumber>(at offset: Data.Index, byteOrder: ByteOrder = hostByteOrder()) throws -> T {
        let size = MemoryLayout<T>.stride
        let range: Range<Int> = offset ..< offset + size
        
        guard range.upperBound <= count else {
            throw ParseError.valueSizeOutOfBounds(offset: offset, size: size, upperBound: count)
        }
        
        var bytes = subdata(in: range)
        if byteOrder != .littleEndian {
            bytes.reverse()
        }
        
        return bytes.withUnsafeBytes { $0.pointee }
    }
    
    /// Returns typed array from receiver's data.
    ///
    /// - parameter offset:   Offset in receiver's bytes.
    /// - parameter capacity: Expected number of elements in array.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: `Array<T>`
    func array<T: MAVLinkNumber>(at offset: Data.Index, capacity: Int) throws -> [T] {
        var offset = offset
        var array = [T]()
        
        for _ in 0 ..< capacity {
            array.append(try number(at: offset))
            offset += MemoryLayout<T>.stride
        }
        
        return array
    }
    
    /// Returns ASCII String from receiver's data.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter length: Expected length of string to read.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: `String`
    func string(at offset: Data.Index, length: Int) throws -> String {
        let range: Range<Int> = offset ..< offset + length
        
        guard range.upperBound <= count else {
            throw ParseError.valueSizeOutOfBounds(offset: offset, size: length, upperBound: count)
        }
        
        let bytes = subdata(in: range)
        let emptySubSequence = Data.SubSequence(capacity: 0)
        let firstSubSequence = bytes.split(separator: 0x0, maxSplits: 1, omittingEmptySubsequences: false).first ?? emptySubSequence
        
        guard let string = String(bytes: firstSubSequence, encoding: .ascii) else {
            throw ParseError.invalidStringEncoding(offset: offset, length: length)
        }
        
        return string
    }
    
    /// Returns proper typed `Enumeration` subtype value from data or throws
    /// `ParserEnumError` or `ParseError` error.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    ///
    /// - throws: Throws `ParserEnumError`, `ParseError`.
    ///
    /// - returns: Properly typed `Enumeration` subtype value.
    func enumeration<T: Enumeration>(at offset: Data.Index) throws -> T where T.RawValue: MAVLinkNumber {
        let rawValue: T.RawValue = try number(at: offset)
        
        guard let enumerationCase = T(rawValue: rawValue) else {
            throw ParseEnumError.unknownValue(enumType: T.self, rawValue: rawValue, valueOffset: offset)
        }
        
        return enumerationCase
    }

    /// Returns a bitmask that is based on enumeration field. Throws ParseError.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: Bitmask subtype value.
    func bitmask<T: MAVLinkBitmask>(at offset: Data.Index) throws -> T where T.RawValue: MAVLinkNumber {
        let rawValue: T.RawValue = try number(at: offset)
        return T(rawValue: rawValue)
    }
}

/// Methods for filling `Data` with properly formatted field values.
extension Data {
    
    /// Sets properly swapped `number` bytes starting from `offset` in
    /// receiver's bytes.
    ///
    ///
    /// - parameter number: Number value to set.
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter byteOrder: Current system endianness.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: MAVLinkNumber>(_ number: T, at offset: Data.Index, byteOrder: ByteOrder = hostByteOrder()) throws {
        let size = MemoryLayout<T>.stride
        let range = offset ..< offset + size
        
        guard range.endIndex <= count else {
            throw PackError.valueSizeOutOfBounds(offset: offset, size: size, upperBound: count)
        }
        
        var number = number
        var bytes: Data = withUnsafePointer(to: &number) { numberPointer in
            let bufferPointer = numberPointer.withMemoryRebound(to: UInt8.self, capacity: size) { pointer in
                return UnsafeBufferPointer(start: pointer, count: size)
            }
            return Data(bufferPointer)
        }
        
        if byteOrder != .littleEndian {
            bytes.reverse()
        }
        
        replaceSubrange(range, with: bytes)
    }
    
    /// Sets `array` of `MAVLinkNumber` values at `offset` with `capacity` validation.
    ///
    /// - parameter array:    Array of values to set.
    /// - parameter offset:   Offset in receiver's bytes.
    /// - parameter capacity: Maximum allowed count of elements in `array`.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: MAVLinkNumber>(_ array: [T], at offset: Data.Index, capacity: Int) throws {
        guard array.count <= capacity else {
            throw PackError.invalidValueLength(offset: offset, providedValueLength: array.count, allowedLength: capacity)
        }
        
        let elementSize = MemoryLayout<T>.stride
        let arraySize = elementSize * array.count
        
        guard offset + arraySize <= count else {
            throw PackError.valueSizeOutOfBounds(offset: offset, size: arraySize, upperBound: count)
        }
        
        for (index, item) in array.enumerated() {
            try set(item, at: offset + index * elementSize)
        }
    }
    
    /// Sets correctly encoded `string` value at `offset` limited to `length` or
    /// throws `PackError`.
    ///
    /// - precondition: `string` value must be ASCII compatible.
    ///
    /// - parameter string: Value to set.
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter length: Maximum allowed length of `string`.
    ///
    /// - throws: Throws `PackError`.
    mutating func set(_ string: String, at offset: Data.Index, length: Int) throws {
        var bytes = string.data(using: .ascii) ?? Data()
        
        if bytes.isEmpty && string.unicodeScalars.count > 0 {
            throw PackError.invalidStringEncoding(offset: offset, string: string)
        }
        
        // Add optional null-termination if provided string is shorter than
        // expectedlength
        if bytes.count < length {
            bytes.append(0x0)
        }
        
        let asciiCharacters = bytes.withUnsafeBytes { Array(UnsafeBufferPointer<UInt8>(start: $0, count: bytes.count)) }
        try set(asciiCharacters, at: offset, capacity: length)
    }
    
    /// Sets correctly formatted `enumeration` raw value at `offset` or throws
    /// `PackError`.
    ///
    /// - parameter enumeration: Value to set.
    /// - parameter offset:      Offset in receiver's bytes.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: Enumeration>(_ enumeration: T, at offset: Data.Index) throws where T.RawValue: MAVLinkNumber {
        try set(enumeration.rawValue, at: offset)
    }

    /// Sets correctly formatted `bitmask` raw value at `offset` or throws
    /// `PackError`.
    ///
    /// - parameter enumeration: Value to set.
    /// - parameter offset:      Offset in receiver's bytes.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: MAVLinkBitmask>(_ enumeration: T, at offset: Data.Index) throws where T.RawValue: MAVLinkNumber {
        try set(enumeration.rawValue, at: offset)
    }
}

// MARK: - Additional MAVLink service info
