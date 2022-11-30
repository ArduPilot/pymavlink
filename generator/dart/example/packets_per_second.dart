import 'dart:typed_data';

import 'package:mavlink/mavlink.dart';
import 'package:mavlink/test.dart';

void main() {
  int seq = 0;
  final stopwatch = Stopwatch();
  final totalStopwatch = Stopwatch();
  Uint8List key = MAVPacketSignature.generateKey();

  final times = List.empty(growable: true);
  int lastStatusUpdate = 0;
  int lastSeq = 0;

  totalStopwatch.start();
  print('Starting test...');
  lastStatusUpdate = totalStopwatch.elapsedMicroseconds;
  while (totalStopwatch.elapsedMilliseconds < 60 * 1000) {
    if (totalStopwatch.elapsedMicroseconds - lastStatusUpdate > 1000000) {
      int diffSeq = seq - lastSeq;
      double diffTime = (totalStopwatch.elapsedMicroseconds - lastStatusUpdate)/1000;
      print("Created $diffSeq packets in ${diffTime/1000}s, total $seq in ${totalStopwatch.elapsedMilliseconds/1000}s, average ${diffSeq/diffTime*1000} packets per second");
      lastStatusUpdate = totalStopwatch.elapsedMicroseconds;
      lastSeq = seq;
    }
    stopwatch.start();
    final message = MSG_TEST_TYPES.MAVLink2(
      compID: MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER,
      sysID: 1,
    );
    message.u8 = MAVUint8(255);
    message.u8_array =
        List<MAVUint8>.filled(MSG_TEST_TYPES.u8_array_max_length, MAVUint8(255));
    message.s8 = MAVInt8(-128);
    message.s8_array =
        List<MAVInt8>.filled(MSG_TEST_TYPES.s8_array_max_length, MAVInt8(-128));
    message.u16 = MAVUint16(65535);
    message.u16_array = List<MAVUint16>.filled(
        MSG_TEST_TYPES.u16_array_max_length, MAVUint16(65535));
    message.s16 = MAVInt16(-32768);
    message.s16_array = List<MAVInt16>.filled(
        MSG_TEST_TYPES.s16_array_max_length, MAVInt16(-32768));
    message.u32 = MAVUint32(4294967295);
    message.u32_array = List<MAVUint32>.filled(
        MSG_TEST_TYPES.u32_array_max_length, MAVUint32(4294967295));
    message.s32 = MAVInt32(-2147483648);
    message.s32_array = List<MAVInt32>.filled(
        MSG_TEST_TYPES.s32_array_max_length, MAVInt32(-2147483648));
    message.u64 =
        MAVUint64(BigInt.parse("18446744073709551615", radix: 10));
    message.u64_array = List<MAVUint64>.filled(
        MSG_TEST_TYPES.u64_array_max_length,
        MAVUint64(
            BigInt.parse("18446744073709551615", radix: 10)));
    message.s64 =
        MAVInt64(BigInt.parse("-9223372036854775808", radix: 10));
    message.s64_array = List<MAVInt64>.filled(MSG_TEST_TYPES.s64_array_max_length,
        MAVInt64(BigInt.parse("-9223372036854775808", radix: 10)));
    message.d = MAVDouble(double.maxFinite);
    message.d_array = List<MAVDouble>.filled(
        MSG_TEST_TYPES.d_array_max_length, MAVDouble(double.maxFinite));
    message.f = MAVFloat(3.40282346638528859811704183484516925439e38);
    message.f_array = List<MAVFloat>.filled(MSG_TEST_TYPES.f_array_max_length,
        MAVFloat(3.40282346638528859811704183484516925439e38));
    message.c = "A";
    message.s = "Test";

    var packet = message.pack(seq);
    packet.incompatFlags = MAVLinkIncompatFlags.SIGNED;
    packet.signature = MAVPacketSignature.builder(packet, key, linkID: 1);

    final Uint8List encoded = packet.encodePacket();
    stopwatch.stop();
    times.add(stopwatch.elapsedMicroseconds);
    stopwatch.reset();
    final dummyList = List<int>.empty(growable: true);
    for (var i = 0; i < encoded.length; i++) {
      dummyList.add(encoded[i]);
    }
    dummyList.clear();
    seq++;
  }
  totalStopwatch.stop();
  // Caclulate how many packets can be generated per second from `times`, an array of the number of microseconds it took to generate each packet.
  double average = double.parse(times.reduce((value, element) => value + element).toString()) / times.length;
  print("Average time to generate packet: $average microseconds");
  print("Packets per second: ${1000000 / average}");
}