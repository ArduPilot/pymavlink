-- Mavlink connection
-- Copyright Fil Andrii root.fi36@gmail.com 2022

with Ada.Assertions;

package body Mavlink.Connection is

   function Parse_Byte (Conn : in out Connection;
                        Val : Interfaces.Unsigned_8) return Boolean is
      use type Interfaces.Unsigned_8;
   begin
      Conn.In_Buf (Conn.In_Ptr) := Val;

      if Conn.In_Ptr = 0 then
         if Val /= Packet_Marker then
            return False;
         end if;
         X25CRC.Reset (Conn.Checksum);
      elsif Conn.In_Ptr = 1 then
         Conn.Len := Natural (Val) + Packet_Header_Length + Packet_Marker_Length;
         X25CRC.Reset (Conn.Checksum);
         X25CRC.Update (Conn.Checksum, Val);
      elsif Conn.In_Ptr > Conn.Len then
         X25CRC.Update (Conn.Checksum, Mavlink.Messages.CRC_Extras (Conn.In_Buf (Pos_Msg_id)));

         Conn.In_Ptr := 0;
         return Conn.Checksum.High = Conn.In_Buf (Conn.Len) and Conn.Checksum.Low = Conn.In_Buf (Conn.Len + 1);
      elsif Conn.In_Ptr /= Conn.Len then
         X25CRC.Update (Conn.Checksum, Val);
      end if;

      Conn.In_Ptr := Conn.In_Ptr + 1;
      return False;
   end Parse_Byte;

   function Get_Msg_Id (Conn : Connection) return Msg_Id is
   begin
      return Conn.In_Buf (Pos_Msg_id);
   end Get_Msg_Id;

   function Pack (Conn : in out Connection;
                  Msg : Mavlink.Messages.Message'Class) return Byte_Arrray is
      use type Interfaces.Unsigned_8;

      Buf : Byte_Arrray (0 .. Natural(Msg.Payload_Length) + Packet_Control_Info_Size - 1);
      Buf_Ptr : Natural;
      Checksum : X25CRC.Checksum;

      Fake_array : Byte_Arrray (0 .. Msg'Size / 8 - 1)
        with Address => Msg'Address, Import => True, Convention => Ada;

   begin

      Buf (0 .. 5) := (Packet_Marker, Msg.Payload_Length, Conn.Out_Sequency,
                      Conn.System_Id, Conn.Component_Id, Msg.Message_Id);
      Buf_Ptr := Natural (Msg.Payload_Length) + 5;
      Buf (6 .. Buf_Ptr) := Fake_array (Message_Size .. Message_Size + Natural (Msg.Payload_Length) - 1);

      Conn.Out_Sequency := Conn.Out_Sequency + 1;

      for B of Buf (1 .. Buf_Ptr) loop
         X25CRC.Update (Checksum, B);
      end loop;
      X25CRC.Update (Checksum, Messages.CRC_Extras (Msg.Message_Id));

      Buf (Buf_Ptr + 1) := Checksum.High;
      Buf (Buf_Ptr + 2) := Checksum.Low;

      return Buf;

   end Pack;

   procedure Unpack (Conn : in out Connection;
                     Msg  : in out Mavlink.Messages.Message'Class) is
      use type Interfaces.Unsigned_8;

      Fake_array : Byte_Arrray (0 .. Msg'Size / 8 - 1)
        with Address => Msg'Address, Import => True, Convention => Ada;
   begin

      Ada.Assertions.Assert (Msg.Payload_Length = Conn.In_Buf (1) and Msg.Message_Id = Conn.In_Buf (Pos_Msg_Id));

      Fake_array (Tag_Length + 1 .. Tag_Length + 3) := Conn.In_Buf (2 .. 4);
      Fake_array (Message_Size .. Message_Size - 1 + Natural (Msg.Payload_Length)) :=
        Conn.In_Buf (Packet_Payload_First .. Packet_Payload_First - 1 + Natural (Msg.Payload_Length));

   end Unpack;

end Mavlink.Connection;
