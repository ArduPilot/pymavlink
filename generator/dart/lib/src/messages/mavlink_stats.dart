/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * Dart mavlink generator tool. It should not be modified by hand.
 */

import 'dart:collection';

import 'package:mavlink/mavlink.dart';
import 'package:mavlink/common.dart';

class MAVLinkStats {
  MAVLinkStats();

  MAVLinkStats.withIgnoreRadioPackets(this._ignoreRadioPackets);

  int _receivedPacketCount = 0; // total received packet count for all sources
  int get receivedPacketCount => _receivedPacketCount;

  int _crcErrorCount = 0;
  int get crcErrorCount => _crcErrorCount;

  int _signatureErrorCount = 0;
  int get signatureErrorCount => _signatureErrorCount;

  int _lostPacketCount = 0; // total lost packet count for all sources
  int get lostPacketCount => _lostPacketCount;

  bool _ignoreRadioPackets = false;
  bool get ignoreRadioPackets => _ignoreRadioPackets;

  // stats are nil for a system id until a packet has been received from a system
  final LinkedHashMap<int, SystemStat> _systemStats = LinkedHashMap.of({}); // stats for each system that is known
  LinkedHashMap<int, SystemStat> get systemStats => _systemStats;

  /// Check the new received packet to see if has lost someone between this and
  /// the last packet
  /// 
  /// @param packet Packet that should be checked
  void newPacket(MAVLinkPacket packet) {
    if (_ignoreRadioPackets && packet.msgID == MSG_RADIO_STATUS.MAVLINK_MSG_ID_RADIO_STATUS) {
      return;
    }

    if (!_systemStats.containsKey(packet.sysID)) {
      // allocate stats for systems that exist on the network
      _systemStats[packet.sysID] = SystemStat();
    }
    _lostPacketCount += _systemStats[packet.sysID]!._newPacket(packet);
    _receivedPacketCount++;
  }

  /// Called when a CRC error happens on the parser
  void crcError() {
    _crcErrorCount++;
  }

  /// Called when a Signature error happens on the parser
  void signatureError() {
    _signatureErrorCount++;
  }

  void resetStats() {
    _crcErrorCount = 0;
    _signatureErrorCount = 0;
    _lostPacketCount = 0;
    _receivedPacketCount = 0;
    _systemStats.clear();
  }
}

/// Stat structure for every system id
class SystemStat {
  int _lostPacketCount = 0; // the lost count for this source
  int get lostPacketCount => _lostPacketCount;

  int _receivedPacketCount = 0;
  int get receivedPacketCount => _receivedPacketCount;

  // stats are nil for a component id until a packet has been received from a system
  final LinkedHashMap<int, ComponentStat> _componentStats = LinkedHashMap.of({}); // stats for each component that is known
  LinkedHashMap<int, ComponentStat> get componentStats => _componentStats;

  int _newPacket(MAVLinkPacket packet) {
    int newLostPackets = 0;
    if (!_componentStats.containsKey(packet.compID)) {
      // allocate stats for components that exist on the network
      _componentStats[packet.compID] = ComponentStat();
    }
    newLostPackets = componentStats[packet.compID]!._newPacket(packet);
    _lostPacketCount += newLostPackets;
    _receivedPacketCount++;
    return newLostPackets;
  }

  void resetStats() {
    _lostPacketCount = 0;
    _receivedPacketCount = 0;
    _componentStats.clear();
  }
}

/// stat structure for every system id
class ComponentStat {
  int _lastPacketSeq = -1;
  int get lastPacketSeq => _lastPacketSeq;

  int _lostPacketCount = 0; // the lost count for this source
  int get lostPacketCount => _lostPacketCount;

  int _receivedPacketCount = 0;
  int get receivedPacketCount => _receivedPacketCount;

  int _newPacket(MAVLinkPacket packet) {
    int newLostPackets = 0;
    if (hasLostPackets(packet)) {
      newLostPackets = _updateLostPacketCount(packet);
    }
    _lastPacketSeq = packet.seq;
    _advanceLastPacketSequence(packet.seq);
    _receivedPacketCount++;
    return newLostPackets;
  }

  void resetStats() {
    _lastPacketSeq = -1;
    _lostPacketCount = 0;
    _receivedPacketCount = 0;
  }

  int _updateLostPacketCount(MAVLinkPacket packet) {
    int lostPackets;
    if (packet.seq - _lastPacketSeq < 0) {
      lostPackets = (packet.seq - _lastPacketSeq) + 255;
    } else {
      lostPackets = (packet.seq - _lastPacketSeq);
    }
    _lostPacketCount += lostPackets;
    return lostPackets;
  }

  void _advanceLastPacketSequence(int lastSeq) {
    // wrap from 255 to 0 if necessary
    _lastPacketSeq = (lastSeq + 1) & 0xFF;
  }

  bool hasLostPackets(MAVLinkPacket packet) {
    return _lastPacketSeq >= 0 && packet.seq != _lastPacketSeq;
  }
}