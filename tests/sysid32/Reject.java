import com.MAVLink.Parser;
import com.MAVLink.MAVLinkPacket;
import java.nio.file.*;
public class Reject {
    public static void main(String[] args) throws Exception {
        try (DirectoryStream<Path> files = Files.newDirectoryStream(Paths.get(args[0]), "*.v2")) {
            for (Path file : files) {
                byte[] original = Files.readAllBytes(file);
                String frameName = file.getFileName().toString().replaceFirst("\\.v2$", ".frame");
                int rejectedLength = Files.readAllBytes(file.resolveSibling(frameName)).length;
                for (boolean magicTrailer : new boolean[]{false, true}) {
                    byte[] bytes = original.clone();
                    // A discard ending one byte early must not interpret the
                    // rejected frame's final CRC/signature byte as a new STX.
                    if (magicTrailer) bytes[rejectedLength - 1] = (byte)0xfd;
                    Parser parser = new Parser();
                    int count = 0;
                    for (byte b : bytes) {
                        MAVLinkPacket packet = parser.mavlink_parse_char(b & 255);
                        if (packet != null) {
                            if (packet.msgid != 0 || packet.sysid != 42) throw new AssertionError(file);
                            count++;
                        }
                    }
                    if (count != 1) throw new AssertionError(file + ": " + count + " magicTrailer=" + magicTrailer);
                }
            }
        }
    }
}
