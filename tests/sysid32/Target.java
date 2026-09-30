import com.MAVLink.Parser;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.common.msg_command_long;
import java.util.Arrays;
public class Target {
    public static void main(String[] args) {
        for (long source : new long[]{42, 0xABCDEF12L}) for (long target : new long[]{0, 7, 255, 256, 0xFFFFFFFFL}) {
            for (boolean signed : new boolean[]{false,true}) {
                msg_command_long command = new msg_command_long();
                command.isMavlink2 = true;
                command.sysid = source; command.compid = 11;
                command.target_system = target; command.target_component = 250;
                command.command = 300; command.confirmation = 1;
                command.param1=1; command.param2=2; command.param3=3;
                command.param4=4; command.param5=5; command.param6=6; command.param7=7;
                MAVLinkPacket packet = command.pack();
                Parser parser = new Parser();
                if (signed) {
                    packet.signingKey = new byte[32]; Arrays.fill(packet.signingKey, (byte)42);
                    packet.signingLinkId = 3; packet.signingTimestamp = 1000;
                    parser.signingKey = packet.signingKey;
                }
                byte[] bytes = packet.encodePacket();
                MAVLinkPacket received = null;
                for (int i=0;i<bytes.length;i++) {
                    MAVLinkPacket result = parser.mavlink_parse_char(bytes[i]);
                    if (result != null) { if (i!=bytes.length-1) throw new AssertionError(); received=result; }
                    System.out.printf("%02x",bytes[i]);
                }
                System.out.println();
                if (received == null || received.sysid != source) throw new AssertionError("source");
                msg_command_long decoded = (msg_command_long)received.unpack();
                if (decoded.target_system != target || decoded.param7 != 7 || decoded.target_component != 250)
                    throw new AssertionError("payload/target");
                decoded.target_system = 7;
                if ((decoded.pack().encodePacket()[2] & 4) != 0) throw new AssertionError("stale target");
                if (signed) {
                    for (byte b : bytes) if (parser.mavlink_parse_char(b) != null) throw new AssertionError("replay");
                    if (parser.badSignatureCount != 1) throw new AssertionError("signature counter");
                    bytes[bytes.length-1] ^= 1;
                    Parser corrupt = new Parser(); corrupt.signingKey=packet.signingKey;
                    for (byte b : bytes) if (corrupt.mavlink_parse_char(b) != null) throw new AssertionError("bad signature");
                }
                decoded.target_system = 256; decoded.isMavlink2=false;
                try { decoded.pack().encodePacket(); throw new AssertionError("MAVLink1 wide target"); }
                catch (IllegalArgumentException expected) { }
            }
        }
    }
}
