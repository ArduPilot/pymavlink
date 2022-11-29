import 'dart:math';
import 'dart:typed_data';

import 'package:mavlink/mavlink.dart';
import 'package:mavlink/ardupilotmega.dart';
import 'package:mavlink/test.dart';

import 'package:test/test.dart';

void main() {
  group('MAVLink Payload Tests', () {
    final payload = MAVLinkPayload();

    setUp(() {
      payload.resetIndex();
    });

    test('Payload can parse signed integers', () {
      final signedInteger = MAVInt32(-2147483648);
      payload.putInt(signedInteger);
      payload.resetIndex();
      expect(payload.getInt(), signedInteger);
    });

    test('Payload can parse unsigned integers', () {
      final unsignedInteger = MAVUint32(4294967295);
      payload.putUnsignedInt(unsignedInteger);
      payload.resetIndex();
      expect(payload.getUnsignedInt(), unsignedInteger);
    });

    test('Payload can parse signed shorts', () {
      final signedShort = MAVInt16(-32768);
      payload.putShort(signedShort);
      payload.resetIndex();
      expect(payload.getShort(), signedShort);
    });

    test('Payload can parse unsigned shorts', () {
      final unsignedShort = MAVUint16(65535);
      payload.putUnsignedShort(unsignedShort);
      payload.resetIndex();
      expect(payload.getUnsignedShort(), unsignedShort);
    });

    test('Payload can parse signed bytes', () {
      final signedByte = MAVInt8(-128);
      payload.putByte(signedByte);
      payload.resetIndex();
      expect(payload.getByte(), signedByte);
    });

    test('Payload can parse unsigned bytes', () {
      final unsignedByte = MAVUint8(255);
      payload.putUnsignedByte(unsignedByte);
      payload.resetIndex();
      expect(payload.getUnsignedByte(), unsignedByte);
    });

    test('Payload can parse double values', () {
      final doubleValue = MAVDouble(double.maxFinite);
      payload.putDouble(doubleValue);
      payload.resetIndex();
      expect(payload.getDouble(), doubleValue);
    });

    test('Payload can parse float values', () {
      final floatValue = MAVFloat(3.40282346638528859811704183484516925439e38);
      payload.putFloat(floatValue);
      payload.resetIndex();
      expect(payload.getFloat(), floatValue);
    });

    test('Payload can parse signed long values', () {
      final longValue = MAVInt64(BigInt.parse("-9223372036854775808", radix: 10).toSigned(64));
      payload.putLong(longValue);
      payload.resetIndex();
      expect(payload.getLong(), longValue);
    });

    test('Payload can parse unsigned long values', () {
      final longValue = MAVUint64(BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64));
      payload.putUnsignedLong(longValue);
      payload.resetIndex();
      expect(payload.getUnsignedLong(), longValue);
    });
  });

  group('MAVLink Types Tests', () {
    test('MAVInt8 will reject bad integers', () {
      expect(() => MAVInt8(-129), throwsA(isA<ArgumentError>()));
      expect(() => MAVInt8(128), throwsA(isA<ArgumentError>()));
    });
    test('MAVUint8 will reject bad integers', () {
      expect(() => MAVUint8(-1), throwsA(isA<ArgumentError>()));
      expect(() => MAVUint8(256), throwsA(isA<ArgumentError>()));
    });

    test('MAVInt16 will reject bad integers', () {
      expect(() => MAVInt16(-32769), throwsA(isA<ArgumentError>()));
      expect(() => MAVInt16(32768), throwsA(isA<ArgumentError>()));
    });

    test('MAVUint16 will reject bad integers', () {
      expect(() => MAVUint16(-1), throwsA(isA<ArgumentError>()));
      expect(() => MAVUint16(65536), throwsA(isA<ArgumentError>()));
    });

    test('MAVInt32 will reject bad integers', () {
      expect(() => MAVInt32(-2147483649), throwsA(isA<ArgumentError>()));
      expect(() => MAVInt32(2147483648), throwsA(isA<ArgumentError>()));
    });

    test('MAVUint32 will reject bad integers', () {
      expect(() => MAVUint32(-1), throwsA(isA<ArgumentError>()));
      expect(() => MAVUint32(4294967296), throwsA(isA<ArgumentError>()));
    });

    test('MAVInt64 will reject bad integers', () {
      expect(() => MAVInt64(BigInt.parse("-9223372036854775809", radix: 10)), throwsA(isA<ArgumentError>()));
      expect(() => MAVInt64(BigInt.parse("9223372036854775808", radix: 10)), throwsA(isA<ArgumentError>()));
    });

    test('MAVUint64 will reject bad integers', () {
      expect(() => MAVUint64(BigInt.parse("-1", radix: 10)), throwsA(isA<ArgumentError>()));
      expect(() => MAVUint64(BigInt.parse("18446744073709551616", radix: 10)), throwsA(isA<ArgumentError>()));
    });

    test('MAVUint64 will take both explicitly and implicitly unsigned numbers', () {
      expect(MAVUint64(BigInt.parse("1", radix: 10)), isA<MAVUint64>());
      expect(MAVUint64(BigInt.parse("1", radix: 10).toUnsigned(64)), isA<MAVUint64>());
      expect(MAVUint64(BigInt.parse("18446744073709551615", radix: 10)), isA<MAVUint64>());
      expect(MAVUint64(BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64)), isA<MAVUint64>());
    });

    test('MAVInt64 will take both explicitly and implicitly signed numbers', () {
      expect(MAVInt64(BigInt.parse("-9223372036854775808", radix: 10)), isA<MAVInt64>());
      expect(MAVInt64(BigInt.parse("-9223372036854775808", radix: 10).toSigned(64)), isA<MAVInt64>());
      expect(MAVInt64(BigInt.parse("9223372036854775807", radix: 10)), isA<MAVInt64>());
      expect(MAVInt64(BigInt.parse("9223372036854775807", radix: 10).toSigned(64)), isA<MAVInt64>());
    });

    test('MAVFloat will reject bad floats', () {
      expect(() => MAVFloat(double.nan), throwsA(isA<ArgumentError>()));
      expect(() => MAVFloat(double.infinity), throwsA(isA<ArgumentError>()));
      expect(() => MAVFloat(double.negativeInfinity), throwsA(isA<ArgumentError>()));
    });

    test('MAVDouble will reject bad doubles', () {
      expect(() => MAVDouble(double.nan), throwsA(isA<ArgumentError>()));
      expect(() => MAVDouble(double.infinity), throwsA(isA<ArgumentError>()));
      expect(() => MAVDouble(double.negativeInfinity), throwsA(isA<ArgumentError>()));
    });

    test('MAVChar will reject bad strings', () {
      expect(() => MAVChar(-1), throwsA(isA<ArgumentError>()));
      expect(() => MAVChar.fromString(''), throwsA(isA<ArgumentError>()));
      expect(() => MAVChar.fromString('ab'), throwsA(isA<ArgumentError>()));
      expect(MAVChar.fromString('a').toString(), 'a');
      expect(MAVChar(97).toString(), 'a');
    });
  });

  group('MAVLink Message Tests', () {
    final sysID = 1;
    final compID = MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
    final packetSeq = 0;
    final incompatFlags = 0;
    final compatFlags = 0;

    test('MAVLink1 message can be created', () {
      final message = MSG_AP_ADC(
        compID: compID,
        sysID: sysID,
      );
      message.adc1 = MAVUint16(1);
      message.adc2 = MAVUint16(2);
      message.adc3 = MAVUint16(3);
      message.adc4 = MAVUint16(4);
      message.adc5 = MAVUint16(5);
      message.adc6 = MAVUint16(6);
      var packet = message.pack(packetSeq);
      expect(packet.payloadIsFilled(), true);
      var encoded = packet.encodePacket();

      // Check STX
      expect(encoded[0], MAVLinkPacket.MAVLINK_STX_MAVLINK1);
      // Length matches actual payload length
      expect(encoded[1], packet.payload.size);

      // Check sequence number
      expect(encoded[2], packetSeq);

      // Check system ID
      expect(encoded[3], sysID);

      // Component ID
      expect(encoded[4], compID);

      // Message ID
      expect(encoded[5], message.msgID);

      // TODO: Check checksum
    });

    test('MAVLink2 message can be created', () {
      final message = MSG_TEST_TYPES.MAVLink2(
        compID: compID,
        sysID: sysID,
      );
      message.u8 = MAVUint8(255);
      message.u8_array = List<MAVUint8>.filled(MSG_TEST_TYPES.u8_array_max_length, MAVUint8(255));
      message.s8 = MAVInt8(-128);
      message.s8_array = List<MAVInt8>.filled(MSG_TEST_TYPES.s8_array_max_length, MAVInt8(-128));
      message.u16 = MAVUint16(65535);
      message.u16_array = List<MAVUint16>.filled(MSG_TEST_TYPES.u16_array_max_length, MAVUint16(65535));
      message.s16 = MAVInt16(-32768);
      message.s16_array = List<MAVInt16>.filled(MSG_TEST_TYPES.s16_array_max_length, MAVInt16(-32768));
      message.u32 = MAVUint32(4294967295);
      message.u32_array = List<MAVUint32>.filled(MSG_TEST_TYPES.u32_array_max_length, MAVUint32(4294967295));
      message.s32 = MAVInt32(-2147483648);
      message.s32_array = List<MAVInt32>.filled(MSG_TEST_TYPES.s32_array_max_length, MAVInt32(-2147483648));
      message.u64 = MAVUint64(BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64));
      message.u64_array = List<MAVUint64>.filled(MSG_TEST_TYPES.u64_array_max_length, MAVUint64(BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64)));
      message.s64 = MAVInt64(BigInt.parse("-9223372036854775808", radix: 10).toSigned(64));
      message.s64_array = List<MAVInt64>.filled(MSG_TEST_TYPES.s64_array_max_length, MAVInt64(BigInt.parse("-9223372036854775808", radix: 10).toSigned(64)));
      message.d = MAVDouble(double.maxFinite);
      message.d_array = List<MAVDouble>.filled(MSG_TEST_TYPES.d_array_max_length, MAVDouble(double.maxFinite));
      message.f = MAVFloat(3.40282346638528859811704183484516925439e38);
      message.f_array = List<MAVFloat>.filled(MSG_TEST_TYPES.f_array_max_length, MAVFloat(3.40282346638528859811704183484516925439e38));
      message.c = "A";
      message.s = "Test";

      var packet = message.pack(packetSeq);
      expect(packet.payloadIsFilled(), true);
      packet.incompatFlags = incompatFlags;
      packet.compatFlags = compatFlags;
      var encoded = packet.encodePacket();

      // Check STX
      expect(encoded[0], MAVLinkPacket.MAVLINK_STX_MAVLINK2);
      // Length matches actual payload length
      expect(encoded[1], MAVLinkPacket.mavTrimPayload(packet.payload.getData()));

      // Check incompat flags
      expect(encoded[2], packet.incompatFlags);

      // Check compat flags
      expect(encoded[3], packet.compatFlags);

      // Check packet sequence
      expect(encoded[4], packetSeq);

      // Check system ID
      expect(encoded[5], sysID);

      // Component ID
      expect(encoded[6], compID);

      // Message ID
      expect(encoded[7] | (encoded[8] << 8*1) | (encoded[9] << 8*2), message.msgID);

      // TODO: Check checksum
    });

    test('MAVLink2 message can be signed', () {
      final linkID = 1;
      final secretKey = MAVPacketSignature.generateKey();
      final message = MSG_AP_ADC.MAVLink2(
        compID: compID,
        sysID: sysID,
      );
      message.adc1 = MAVUint16(1);
      message.adc2 = MAVUint16(2);
      message.adc3 = MAVUint16(3);
      message.adc4 = MAVUint16(4);
      message.adc5 = MAVUint16(5);
      message.adc6 = MAVUint16(6);
      var packet = message.pack(packetSeq);
      packet.incompatFlags = MAVLinkIncompatFlags.SIGNED;
      packet.compatFlags = compatFlags;
      packet.signature = MAVPacketSignature.builder(packet, secretKey, linkID: linkID);
      var encoded = packet.encodePacket();

      // Check STX
      expect(encoded[0], MAVLinkPacket.MAVLINK_STX_MAVLINK2);
      // Length matches actual payload length
      expect(encoded[1], MAVLinkPacket.mavTrimPayload(packet.payload.getData()));

      // Check incompat flags
      expect(encoded[2], packet.incompatFlags);

      // Check compat flags
      expect(encoded[3], packet.compatFlags);

      // Check packet sequence
      expect(encoded[4], packetSeq);

      // Check system ID
      expect(encoded[5], sysID);

      // Component ID
      expect(encoded[6], compID);

      // Message ID
      expect(encoded[7] | (encoded[8] << 8*1) | (encoded[9] << 8*2), message.msgID);

      // TODO: Check checksum

      expect(encoded[encoded.length - 13], linkID);

      var strBuf = StringBuffer();
      for (int i = 7; i < 13; i++) {
        strBuf.write(encoded[encoded.length - i].toRadixString(16).padLeft(2, '0'));
      }
      var rxTimestamp = int.parse(strBuf.toString(), radix: 16);
      expect(rxTimestamp, packet.signature!.timestamp);

      strBuf = StringBuffer();
      for (int i = 1; i < 7; i++) {
        strBuf.write(encoded[encoded.length - i].toRadixString(16).padLeft(2, '0'));
      }
      var signature = int.parse(strBuf.toString(), radix: 16);
      expect(signature, packet.signature!.signature);

      expect(MAVPacketSignature.from(packet, secretKey, timestamp: rxTimestamp, linkID: linkID, signature: signature).isValid(), true);
    });

    test('MAVLink2 signed message secret keys are rejected when not 32 bytes', () {
      final linkID = 1;
      final message = MSG_AP_ADC.MAVLink2(
        compID: compID,
        sysID: sysID,
      );
      message.adc1 = MAVUint16(1);
      message.adc2 = MAVUint16(2);
      message.adc3 = MAVUint16(3);
      message.adc4 = MAVUint16(4);
      message.adc5 = MAVUint16(5);
      message.adc6 = MAVUint16(6);
      var packet = message.pack(packetSeq);
      packet.incompatFlags = MAVLinkIncompatFlags.SIGNED;
      packet.compatFlags = compatFlags;

      expect(() => MAVPacketSignature.builder(packet, Uint8List(4), linkID: linkID), throwsA(isA<ArgumentError>()));
      expect(() => MAVPacketSignature.builder(packet, Uint8List(64), linkID: linkID), throwsA(isA<ArgumentError>()));
      expect(MAVPacketSignature.builder(packet, Uint8List(32), linkID: linkID), isA<MAVPacketSignature>());
    });
  });
}
