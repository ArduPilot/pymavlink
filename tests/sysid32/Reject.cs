using System;
using System.IO;
class Reject {
    static void Main(string[] args) {
        foreach (string path in Directory.GetFiles(args[0], "*.v2")) {
            byte[] original = File.ReadAllBytes(path);
            int rejectedLength = File.ReadAllBytes(Path.ChangeExtension(path, "frame")).Length;
            foreach (bool magicTrailer in new[] { false, true }) {
                byte[] bytes = (byte[])original.Clone();
                // An early discard must not treat the final CRC/signature byte as STX.
                if (magicTrailer) bytes[rejectedLength - 1] = 0xfd;
                var stream = new MemoryStream(bytes);
                var parser = new MAVLink.MavlinkParse();
                if (parser.ReadPacket(stream) != null) throw new Exception("accepted " + path);
                if (parser.badCRC != 0 || parser.badIncompatFlags != 1)
                    throw new Exception("unsupported frame reported as corrupt " + path);
                if (stream.Position != rejectedLength)
                    throw new Exception("wrong discard length " + path + " magicTrailer=" + magicTrailer);
                var packet = parser.ReadPacket(stream);
                if (packet == null || packet.msgid != 0 || packet.sysid != 42 || stream.Position != stream.Length)
                    throw new Exception("lost synchronization " + path + " magicTrailer=" + magicTrailer);
            }
            // Packet-oriented transports must reject the same header too.
            bool rejected = false;
            try { new MAVLink.MAVLinkMessage(File.ReadAllBytes(Path.ChangeExtension(path, "frame"))); }
            catch (NotSupportedException) { rejected = true; }
            if (!rejected) throw new Exception("constructor accepted " + path);
        }
        byte[] corrupt = File.ReadAllBytes(Path.Combine(args[0], "legacy"));
        corrupt[corrupt.Length - 1] ^= 1;
        var crcParser = new MAVLink.MavlinkParse();
        if (crcParser.ReadPacket(new MemoryStream(corrupt)) != null ||
            crcParser.badCRC != 1 || crcParser.badIncompatFlags != 0)
            throw new Exception("CRC error misclassified");
    }
}
