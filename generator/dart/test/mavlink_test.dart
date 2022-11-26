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

  group('MAVLink Message Tests', () {
    test('MAVLink1 message can be created', () {
      final message = MSG_AP_ADC(
        compID: MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER,
        sysID: 1,
      );
      message.adc1 = MAVUint16(1);
      message.adc2 = MAVUint16(2);
      message.adc3 = MAVUint16(3);
      message.adc4 = MAVUint16(4);
      message.adc5 = MAVUint16(5);
      message.adc6 = MAVUint16(6);
      StringBuffer buf = StringBuffer("[");
      message.pack(0).encodePacket().forEach((element) {
        buf.write("0x");
        buf.write(element.toRadixString(16).padLeft(2, "0").toUpperCase());
        buf.write(",");
      });
      buf.write("]");
      expect(buf.toString(), "ASDF");
    });

    test('MAVLink2 message can be created', () {
      final message = MSG_TEST_TYPES.MAVLink2(
        compID: MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER,
        sysID: 1,
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
      message.s64 = MAVInt64(BigInt.parse("-9223372036854775808", radix: 10).toUnsigned(64));
      message.s64_array = List<MAVInt64>.filled(MSG_TEST_TYPES.s64_array_max_length, MAVInt64(BigInt.parse("-9223372036854775808", radix: 10).toUnsigned(64)));
      message.d = MAVDouble(double.maxFinite);
      message.d_array = List<MAVDouble>.filled(MSG_TEST_TYPES.d_array_max_length, MAVDouble(double.maxFinite));
      message.f = MAVFloat(3.40282346638528859811704183484516925439e38);
      message.f_array = List<MAVFloat>.filled(MSG_TEST_TYPES.f_array_max_length, MAVFloat(3.40282346638528859811704183484516925439e38));
      message.c = "A";
      message.s = "Test";

      StringBuffer buf = StringBuffer("[");
      message.pack(0).encodePacket().forEach((element) {
        buf.write("0x");
        buf.write(element.toRadixString(16).padLeft(2, "0").toUpperCase());
        buf.write(",");
      });
      buf.write("]");
      expect(buf.toString(), "ASDF");
    });
  });
}
