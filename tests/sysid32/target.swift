import Foundation
for source: UInt32 in [42, 0xABCDEF12] {
    for target: UInt32 in [0, 7, 255, 256, 0xffffffff] {
      for signed in [false, true] {
        var payload = Data(count: 33)
        for i in 0..<7 { try payload.set(Float(i + 1), at: i * 4) }
        try payload.set(UInt16(300), at: 28)
        payload[30] = UInt8(min(target,255)); payload[31] = 250; payload[32] = 1
        var message = try CommandLong(data: payload)
        message.targetSystem = target
        let packet = Packet(message: message, systemId: source, componentId: 11, channel: 255)
        let signing = Signing(secretKey: Data(repeating: 42, count: 32), linkId: 3, timestamp: 1000)
        signing.signOutgoing = signed
        let bytes = try packet.finalize(sequence: 0, signing: signing)
        print(bytes.map { String(format: "%02x", $0) }.joined())
        let link = MAVLink()
        link.signing = Signing(secretKey: Data(repeating: 42, count: 32))
        var received: Packet?
        for (i, byte) in bytes.enumerated() {
            if let p = link.parse(char: byte, channel: 255) { precondition(i == bytes.count - 1); received = p }
        }
        precondition(received?.systemId == source)
        if signed {
            for byte in bytes { precondition(link.parse(char: byte, channel: 255) == nil) }
            precondition(link.badSignatures == 1)
            let bad = MAVLink(); bad.signing = Signing(secretKey: signing.secretKey)
            var tampered = bytes; tampered[tampered.count - 1] ^= 1
            for byte in tampered { precondition(bad.parse(char: byte, channel: 0) == nil) }
            precondition(bad.badSignatures == 1)
        }
        var decoded = received!.message as! CommandLong
        precondition(decoded.targetSystem == target && decoded.param7 == 7 && decoded.targetComponent == 250)
        decoded.targetSystem = 7
        let forwarded = try Packet(message: decoded, systemId: source, componentId: 11, channel: 0).finalize(sequence: 0)
        precondition(forwarded[2] & 4 == 0)
        decoded.targetSystem = 256
        do {
            _ = try Packet(message: decoded, systemId: source, componentId: 11, channel: 0).finalize(sequence: 0, mavlink2: false)
            fatalError("MAVLink1 must reject wide IDs")
        } catch PackError.unsupportedProtocol { }
    }
}
}
