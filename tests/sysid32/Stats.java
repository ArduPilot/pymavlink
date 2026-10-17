import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkStats;

public class Stats {
    private static void packet(MAVLinkStats stats, long source, int sequence) {
        MAVLinkPacket packet = new MAVLinkPacket(0, true);
        packet.sysid = source;
        packet.compid = 11;
        packet.seq = sequence;
        stats.newPacket(packet);
    }

    public static void main(String[] args) {
        MAVLinkStats stats = new MAVLinkStats(false, 2);
        packet(stats, 70000, 0);
        packet(stats, 70001, 0);
        packet(stats, 70000, 2); // keep this active source; count one lost packet
        packet(stats, 70002, 0);
        if (stats.wideSystemStats.size() != 2 || stats.wideSystemStats.containsKey(70001L)
            || !stats.wideSystemStats.containsKey(70000L)) throw new AssertionError("LRU eviction");
        if (stats.lostPacketCount != 1 || stats.receivedPacketCount != 4)
            throw new AssertionError("aggregate counters");
        packet(stats, 70001, 200); // evicted source starts fresh sequence history
        if (stats.lostPacketCount != 1 || stats.receivedPacketCount != 5)
            throw new AssertionError("evicted source history");
        packet(stats, 255, 0);
        packet(stats, 255, 2);
        if (stats.systemStats[255].lostPacketCount != 1 || stats.lostPacketCount != 2)
            throw new AssertionError("legacy stats");
        stats.crcError();
        stats.resetStats();
        if (!stats.wideSystemStats.isEmpty() || stats.systemStats[255] != null
            || stats.receivedPacketCount != 0 || stats.lostPacketCount != 0 || stats.crcErrorCount != 0)
            throw new AssertionError("reset");
        for (long id = 100000; id < 100010; id++) packet(stats, id, 0);
        if (stats.wideSystemStats.size() != 2) throw new AssertionError("reset lost configured limit");

        stats = new MAVLinkStats();
        for (long id = 100000; id < 200000; id++) packet(stats, id, 0);
        if (stats.wideSystemStats.size() != MAVLinkStats.DEFAULT_MAX_WIDE_SYSTEMS
            || stats.receivedPacketCount != 100000 || stats.lostPacketCount != 0)
            throw new AssertionError("bounded history under changing IDs");
        try {
            new MAVLinkStats(false, 0);
            throw new AssertionError("zero limit accepted");
        } catch (IllegalArgumentException expected) { }
    }
}
