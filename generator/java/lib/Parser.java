/* AUTO-GENERATED FILE. DO NOT MODIFY.
 * Stream framing for MAVLink 1 and MAVLink 2, including extended system IDs.
 */
package com.MAVLink;

import com.MAVLink.Messages.MAVLinkStats;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.security.MessageDigest;

public class Parser {
    public MAVLinkStats stats;
    /** Configure a 32-byte key to accept signed packets. Unsigned packets remain accepted. */
    public byte[] signingKey;
    public long signingTimestamp;
    public int badSignatureCount;
    public int unsupportedFrames;
    private final Map<String, Long> signingStreams = new HashMap<String, Long>();
    private final byte[] frame = new byte[287];
    private int used;
    private int expected;

    public Parser() { this(false); }
    public Parser(boolean ignoreRadioPacketStats) {
        stats = new MAVLinkStats(ignoreRadioPacketStats);
    }

    private static long unsignedLE(byte[] bytes, int offset, int count) {
        long value = 0;
        for (int i = 0; i < count; i++) value |= (bytes[offset + i] & 255L) << (8 * i);
        return value;
    }

    /** Consume a complete frame before rejecting it, so payload magic cannot resynchronize the parser. */
    public MAVLinkPacket mavlink_parse_char(int c) {
        c &= 255;
        if (used == 0 && c != 0xfd && c != 0xfe) return null;
        frame[used++] = (byte)c;
        if (used < 3) return null;
        final boolean v2 = (frame[0] & 255) == 0xfd;
        final int flags = v2 ? frame[2] & 255 : 0;
        final int payloadLength = frame[1] & 255;
        final int sourceLength = (flags & 2) != 0 ? 4 : 1;
        final int headerLength = v2 ? 9 + sourceLength + ((flags & 4) != 0 ? 4 : 0) : 6;
        if (used == 3) expected = headerLength + payloadLength + 2 + ((flags & 1) != 0 ? 13 : 0);
        if (used < expected) return null;
        used = 0;
        if ((flags & ~7) != 0) { unsupportedFrames++; return null; }

        MAVLinkPacket packet = new MAVLinkPacket(payloadLength, v2);
        packet.incompatFlags = flags;
        packet.compatFlags = v2 ? frame[3] & 255 : 0;
        packet.seq = frame[v2 ? 4 : 2] & 255;
        int offset = v2 ? 5 : 3;
        packet.sysid = unsignedLE(frame, offset, sourceLength);
        offset += sourceLength;
        packet.compid = frame[offset++] & 255;
        packet.msgid = (int)unsignedLE(frame, offset, v2 ? 3 : 1);
        offset += v2 ? 3 : 1;
        if ((flags & 4) != 0) packet.targetSysid = unsignedLE(frame, offset, 4);
        for (int i = 0; i < payloadLength; i++) packet.payload.add(frame[headerLength + i]);
        if (!packet.generateCRC(payloadLength) || packet.crc.getLSB() != (frame[headerLength + payloadLength] & 255)
                || packet.crc.getMSB() != (frame[headerLength + payloadLength + 1] & 255)) {
            stats.crcError();
            return null;
        }
        if ((flags & 1) != 0) {
            int signatureOffset = headerLength + payloadLength + 2;
            long timestamp = unsignedLE(frame, signatureOffset + 1, 6);
            int linkId = frame[signatureOffset] & 255;
            String stream = packet.sysid + ":" + packet.compid + ":" + linkId;
            Long previous = signingStreams.get(stream);
            if (signingKey == null || signingKey.length != 32
                    || !MessageDigest.isEqual(MAVLinkPacket.signature(signingKey, frame, signatureOffset + 7),
                        Arrays.copyOfRange(frame, signatureOffset + 7, expected))
                    || (previous != null && timestamp <= previous)
                    || (previous == null && timestamp + 6000000L < signingTimestamp)) {
                badSignatureCount++;
                return null;
            }
            signingStreams.put(stream, timestamp);
            signingTimestamp = Math.max(signingTimestamp, timestamp);
            packet.signingLinkId = linkId;
            packet.signingTimestamp = timestamp;
        }
        stats.newPacket(packet);
        return packet;
    }
}
