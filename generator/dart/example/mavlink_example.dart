import 'package:mavlink/mavlink.dart';
import 'package:mavlink/test.dart';

void main() {
  int seq = 0;
  final message = MSG_TEST_TYPES(
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

  print(message.pack(seq).payload.actualSize);
  print(MSG_TEST_TYPES.MAVLINK_MSG_LENGTH);
}
