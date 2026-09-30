using System;
using System.IO;
class Target {
    static void Main() {
        foreach (uint source in new uint[]{42, 0xABCDEF12}) foreach (uint target in new uint[]{0,7,255,256,0xFFFFFFFF}) foreach (bool signed in new bool[]{false,true}) {
            var parser = new MAVLink.MavlinkParse();
            parser.signingKey = new byte[32]; for (int i=0;i<32;i++) parser.signingKey[i]=42;
            var message = new MAVLink.mavlink_command_long_t();
            message.param1=1; message.param2=2; message.param3=3; message.param4=4;
            message.param5=5; message.param6=6; message.param7=7;
            message.command=300; message.confirmation=1;
            var bytes = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, message, signed, source, 11, 0, target, 250);
            Console.WriteLine(BitConverter.ToString(bytes).Replace("-", "").ToLowerInvariant());
            var received = new MAVLink.MavlinkParse().ReadPacket(new MemoryStream(bytes));
            if (received == null || received.sysid != source || received.GetTargetSystem() != target || received.GetTargetComponent() != 250) throw new Exception("IDs");
            var decoded = (MAVLink.mavlink_command_long_t)received.data;
            if (decoded.param7 != 7 || decoded.target_system != Math.Min(target,255)) throw new Exception("payload");
            var padded = new byte[bytes.Length + 16]; Array.Copy(bytes, padded, bytes.Length);
            if (new MAVLink.MAVLinkMessage(padded).GetTargetSystem() != target) throw new Exception("padded buffer");
            var preserved = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, decoded, targetSystem: target);
            if (new MAVLink.MAVLinkMessage(preserved).GetTargetComponent() != 250) throw new Exception("component clobbered");
            bytes = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, decoded, false, source, 11, 0, 7, 19);
            received = new MAVLink.MavlinkParse().ReadPacket(new MemoryStream(bytes));
            if (received.GetTargetSystem()!=7 || (bytes[2]&4)!=0) throw new Exception("stale target");
            try { parser.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, decoded, 256); throw new Exception("MAVLink1 truncation"); }
            catch (ArgumentOutOfRangeException) { }
        }
    }
}
