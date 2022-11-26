import 'dart:typed_data';

import 'package:binary/binary.dart';
import 'package:mavlink/ardupilotmega.dart';
import 'package:mavlink/mavlink.dart';
import 'package:mavlink/src/enums/mav_component.dart';
import 'package:mavlink/src/test/export.dart';
import 'package:test/test.dart';

void main() {
  group('MAVLink Payload Tests', () {
    final payload = MAVLinkPayload();

    setUp(() {
      payload.resetIndex();
    });

    test('Payload can parse signed integers', () {
      final int signedInteger = -2147483648;
      payload.putInt(signedInteger);
      payload.resetIndex();
      expect(payload.getInt(), signedInteger);
    });

    test('Payload can parse unsigned integers', () {
      final int unsignedInteger = 4294967295;
      payload.putUnsignedInt(unsignedInteger);
      payload.resetIndex();
      expect(payload.getUnsignedInt(), unsignedInteger);
    });

    test('Payload can parse signed shorts', () {
      final int signedShort = -32768;
      payload.putShort(signedShort);
      payload.resetIndex();
      expect(payload.getShort(), signedShort);
    });

    test('Payload can parse unsigned shorts', () {
      final int unsignedShort = 65535;
      payload.putUnsignedShort(unsignedShort);
      payload.resetIndex();
      expect(payload.getUnsignedShort(), unsignedShort);
    });

    test('Payload can parse signed bytes', () {
      final int signedByte = -128;
      payload.putByte(signedByte);
      payload.resetIndex();
      expect(payload.getByte(), signedByte);
    });

    test('Payload can parse unsigned bytes', () {
      final int unsignedByte = 255;
      payload.putUnsignedByte(unsignedByte);
      payload.resetIndex();
      expect(payload.getUnsignedByte(), unsignedByte);
    });

    test('Payload can parse double values', () {
      final double doubleValue = double.maxFinite;
      payload.putDouble(doubleValue);
      payload.resetIndex();
      expect(payload.getDouble(), doubleValue);
    });

    test('Payload can parse float values', () {
      final double floatValue = 340282346638528859811704183484516925440;
      payload.putFloat(floatValue);
      payload.resetIndex();
      expect(payload.getFloat(), floatValue);
    });

    test('Payload can parse signed long values', () {
      final BigInt longValue = BigInt.parse("-9223372036854775808", radix: 10).toSigned(64);
      payload.putLong(longValue);
      payload.resetIndex();
      expect(payload.getLong(), longValue);
    });

    test('Payload can parse unsigned long values', () {
      final BigInt longValue = BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64);
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
      message.adc1 = 1;
      message.adc2 = 2;
      message.adc3 = 3;
      message.adc4 = 4;
      message.adc5 = 5;
      message.adc6 = 6;
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
      message.u8 = 255;
      message.u8_array = List<int>.filled(3, 255);
      message.s8 = -128;
      message.s8_array = List<int>.filled(3, -128);
      message.u16 = 65535;
      message.u16_array = List<int>.filled(3, 65535);
      message.s16 = -32768;
      message.s16_array = List<int>.filled(3, -32768);
      message.u32 = 4294967295;
      message.u32_array = List<int>.filled(3, 4294967295);
      message.s32 = -2147483648;
      message.s32_array = List<int>.filled(3, -2147483648);
      message.u64 = BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64);
      message.u64_array = List<BigInt>.filled(3, BigInt.parse("18446744073709551615", radix: 10).toUnsigned(64));
      message.s64 = BigInt.parse("-9223372036854775808", radix: 10).toUnsigned(64);
      message.s64_array = List<BigInt>.filled(3, BigInt.parse("-9223372036854775808", radix: 10).toUnsigned(64));
      message.d = double.maxFinite;
      message.d_array = List<double>.filled(3, double.maxFinite);
      message.f = 340282346638528859811704183484516925440;
      message.f_array = List<double>.filled(3, 340282346638528859811704183484516925440);
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
