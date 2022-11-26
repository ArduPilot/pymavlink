import 'package:mavlink/src/enums/mav_component.dart';
import 'package:mavlink/test.dart';

void main() {
  int seq = 0;
  final message = MSG_TEST_TYPES(
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

  print(message.pack(seq).payload.size());
  print(MSG_TEST_TYPES.MAVLINK_MSG_LENGTH);
}
