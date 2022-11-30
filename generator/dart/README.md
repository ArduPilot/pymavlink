# MAVLink Dart SDK

A web-friendly SDK to encode and decode MAVLink1 and 2 packets.

## Features

- MAVLink1 and MAVLink2 packet encoding and decoding
- MAVLink2 signed packet support
- Null safe

## Getting started

Import the `package:mavlink/mavlink.dart` file. This automatically includes
any enums and messages in the `minimal` dialect. Additional dialects can
be included by importing `package:mavlink/<dialect_name>.dart` in addition
to `mavlink.dart`.

`package:mavlink/crc.dart` provides access to a CRC-16/MCRF4XX generator
and an abstract class used to implement dialect-specific CRC Extra bytes.

## Usage

### Type System

A typing system has been implemented to allow for type safety when
working with MAVLink messages. These types are `MAVChar`, `MAVUint8`,
`MAVInt8`, `MAVUint16`, `MAVInt16`, `MAVUint32`, `MAVInt32`, `MAVUint64`,
`MAVInt64`, `MAVFloat`, and `MAVDouble`.

To use these types, simply wrap the native value in the type constructor.
For example, to create a `MAVUint8` with the value 5:

```dart
MAVUint8(5)
```

To get the value out of the type, use the `value` getter.
For example, to get the integer value of 5 from the `MAVUint8` above:

```dart
MAVUint8(5).value
```

#### 64-bit types

64-bit native types are not supported on web platforms, so instead of using
an integer type for the 64-bit `MAVInt64` and `MAVUint64` types, pass
in a `BigInt` instead.
For example, to create a `MAVUint64` with the value 5:

```dart
MAVUint64(BigInt.from(5))
```

To get the value out of the type, use the `value` getter.
For example, to get the `BigInt` value of the `MAVUint64` above:

```dart
MAVUint64(BigInt.from(5)).value
```

### Parsing a MAVLink packet

In this example, `packetBytes` is a `Uint8List` containing the bytes of a MAVLink packet.
Optionally, the parser can be directed to only use specific dialects when calculating CRCs
by passing `dialectCRCs` as a named parameter. MAVLink2 signed packets can be parsed by
optionally providing the secret key (32 byte `Uint8List`) as a named parameter `signatureKey`.

```dart
var parser = MAVLinkParser()
// also valid: MAVLinkParser(dialectCRCs: <DialectCRC>[ardupilotmega_CRC()], signatureKey: secretKey);

MAVLinkPacket? packet;
for (var byte in packetBytes) {
  packet = parser.mavlinkParseChar(byte);
  if (packet != null) {
    // We have a packet and can obtain the message with packet.unpack()
  }
}
```

### Creating a MAVLink1 packet

This example uses the `MSG_AP_ADC` message from the `ardupilotmega` dialect.
`compID` is the MAVLink component ID and `sysID` is the MAVLink system ID.
The CRC, including CRC extra, is automatically applied to the packet on creation.

```dart
final mav1Message = MSG_AP_ADC(
  compID: compID,
  sysID: sysID,
);

mav1Message.adc1 = MAVUint16(1);
mav1Message.adc2 = MAVUint16(2);
mav1Message.adc3 = MAVUint16(3);
mav1Message.adc4 = MAVUint16(4);
mav1Message.adc5 = MAVUint16(5);
mav1Message.adc6 = MAVUint16(6);

MAVLinkPacket mav1Packet = mav1Message.pack(packetSeq);
Uint8List encoded = mav1Packet.encodePacket();
```

### Creating a MAVLink2 packet

```dart
final mav2Message = MSG_AP_ADC.MAVLink2(
  compID: compID,
  sysID: sysID,
);

mav2Message.adc1 = MAVUint16(1);
mav2Message.adc2 = MAVUint16(2);
mav2Message.adc3 = MAVUint16(3);
mav2Message.adc4 = MAVUint16(4);
mav2Message.adc5 = MAVUint16(5);
mav2Message.adc6 = MAVUint16(6);

MAVLinkPacket mav2Packet = mav2Message.pack(packetSeq);
Uint8List encoded = mav2Packet.encodePacket();
```

### Creating a signed MAVLink2 packet

`linkID` is the MAVLink2 packet signature's link ID. Signature keys can be
created by calling `MAVPacketSignature.generateKey()`.

```dart
Uint8List secretKey = MAVPacketSignature.generateKey();

final mav2Message = MSG_AP_ADC(
  compID: compID,
  sysID: sysID,
  isMavlink2: true,
);

mav2Message.adc1 = MAVUint16(1);
mav2Message.adc2 = MAVUint16(2);
mav2Message.adc3 = MAVUint16(3);
mav2Message.adc4 = MAVUint16(4);
mav2Message.adc5 = MAVUint16(5);
mav2Message.adc6 = MAVUint16(6);

MAVLinkPacket mav2Packet = mav2Message.pack(packetSeq);
mav2Packet.incompatFlags = MAVLinkIncompatFlags.SIGNED;
mav2Packet.signature = MAVPacketSignature.builder(mav2Packet, secretKey, linkID: linkID);

Uint8List encoded = mav2Packet.encodePacket();
```
