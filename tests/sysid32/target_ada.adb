with Ada.Text_IO;
with Interfaces; use Interfaces;
with MAVLink.Raw_Floats; use MAVLink.Raw_Floats;
with MAVLink.V2; use MAVLink.V2;
with MAVLink.V2.Common.Types; use MAVLink.V2.Common.Types;
with MAVLink.V2.Common.Command_Longs; use MAVLink.V2.Common.Command_Longs;
procedure Target_Ada is
   type Ids is array (Positive range <>) of System_Id_Type;
   Sources : constant Ids := [42, 16#ABCDEF12#];
   Targets : constant Ids := [0, 7, 255, 256, 16#FFFFFFFF#];
   Hex : constant String := "0123456789abcdef";
   Outgoing : Out_Connection;
   Incoming : In_Connection;
   Sign : Signature;
   Buffer : Data_Buffer (1 .. Maximum_Buffer_Len);
   Last : Positive;
   Link : Link_Id_Type;
   Time : Timestamp_Type;
   Valid : Three_Boolean;
   CRC_Valid : Boolean;
   Message : Command_Long := (Target_System => 0, Target_Component => 250,
       Command => Mission_Start, Confirmation => 1,
       Param1 => To_Raw(1.0), Param2 => To_Raw(2.0), Param3 => To_Raw(3.0),
       Param4 => To_Raw(4.0), Param5 => To_Raw(5.0), Param6 => To_Raw(6.0), Param7 => To_Raw(7.0));
   Decoded : Command_Long;
begin
   for Source of Sources loop
      for Target of Targets loop
         for Signed in Boolean loop
            Clear (Incoming);
            Set_System_Id (Outgoing, Source);
            Set_Component_Id (Outgoing, 11);
            Set_Sequency_Id (Outgoing, 0);
            Initialize (Sign, 3, [1 .. 32 => 42], 1000);
            if Signed then Encode (Message, Outgoing, Sign, Buffer, Last, Target);
            else Encode (Message, Outgoing, Buffer, Last, Target); end if;
            for I in 1 .. Last loop
               Ada.Text_IO.Put (Hex (Integer (Buffer(I) / 16) + 1));
               Ada.Text_IO.Put (Hex (Integer (Buffer(I) mod 16) + 1));
               if Parse_Byte (Incoming, Buffer (I)) then
                  pragma Assert (I = Last);
                  pragma Assert (Get_Message_System_Id (Incoming) = Source);
                  pragma Assert (Get_Message_Id (Incoming) = 76);
                  Decode (Decoded, Incoming, CRC_Valid);
                  pragma Assert (CRC_Valid);
                  pragma Assert (Get_Target_System (Decoded, Incoming) = Target);
                  pragma Assert (Decoded.Param7 = Message.Param7);
                  if Signed then
                     Check_Message_Signature (Incoming, Sign, Link, Time, Valid);
                     pragma Assert (Valid = MAVLink.V2.True and Link = 3 and Time = 1000);
                     pragma Assert (Get_Message_Link_Id (Incoming) = 3);
                  end if;
               else pragma Assert (I /= Last);
               end if;
            end loop;
            Ada.Text_IO.New_Line;
         end loop;
      end loop;
   end loop;
end Target_Ada;
