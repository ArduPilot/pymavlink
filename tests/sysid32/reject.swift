import Foundation
let root = CommandLine.arguments[1]
for file in try FileManager.default.contentsOfDirectory(atPath: root).filter({ $0.hasSuffix(".v1") }) {
    let data = try Data(contentsOf: URL(fileURLWithPath: root + "/" + file))
    let parser = MAVLink()
    var count = 0
    for byte in data {
        if let packet = parser.parse(char: byte, channel: 0) {
            precondition(packet.messageId == 0 && packet.systemId == 42, file)
            count += 1
        }
    }
    precondition(count == 1, file)
}
