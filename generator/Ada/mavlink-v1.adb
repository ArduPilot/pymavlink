--  MAVLink connection
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

package body MAVLink.V1 is

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Conn : in out Connection; Val : Interfaces.Unsigned_8) return Boolean
   is
      use type Interfaces.Unsigned_8;
   begin
      if Conn.In_Ptr = 0
        and then Val /= Version_1_Code
      then
         return False;
      end if;

      Conn.In_Ptr := Conn.In_Ptr + 1;
      Conn.In_Buf (Conn.In_Ptr) := Val;

      if Conn.In_Ptr = Pos_Len then
         Conn.Len := Conn.In_Buf'First +
           Packet_Payload_First + --  header
             Natural (Conn.In_Buf (Pos_Len)) + --  data len
           1; --  x25crc checksum -1

         X25CRC.Reset (Conn.Checksum);
         X25CRC.Update (Conn.Checksum, Val);
         Conn.Extras_Added := False;

      elsif Conn.In_Ptr = Conn.Len then
         Conn.In_Ptr := 0;
         return True;

      elsif Conn.In_Ptr <= Conn.Len - 2 then
         X25CRC.Update (Conn.Checksum, Val);
      end if;

      return False;
   end Parse_Byte;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Conn   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural)
   is
      Len : constant Natural := Natural'Min
        (Buffer'Length,
         (if Conn.In_Ptr = 0 then Conn.Len else Conn.In_Ptr));
   begin
      Last := Buffer'First + Len - 1;
      Buffer (Buffer'First .. Last) :=
        Conn.In_Buf (Conn.In_Buf'First .. Conn.In_Buf'First + Len - 1);
   end Get_Buffer;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Conn   : in out Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean is
   begin
      if not Conn.Extras_Added then
         X25CRC.Update (Conn.Checksum, Extras);
         Conn.Extras_Added := True;
      end if;

      return Conn.Checksum.High = Conn.In_Buf (Conn.Len - 1)
        and Conn.Checksum.Low = Conn.In_Buf (Conn.Len);
   end Is_CRC_Valid;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Conn   : Connection;
      Buffer : out Data_Buffer)
   is
      Header    : constant V1_Header with Import,
        Address => Conn.In_Buf'Address;
      Last_Data : constant Positive := Conn.In_Buf'First +
        Packet_Payload_First +
          Natural (Header.Len - 1);
      Last : constant Natural := Buffer'First + Natural (Header.Len - 1);
   begin
      Buffer (Buffer'First .. Last) :=
        Conn.In_Buf (Conn.In_Buf'First + Packet_Payload_First .. Last_Data);
   end Get_Message_Data;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Conn   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode (Conn.System_Id, Conn.Component_Id, Conn.Out_Sequency,
              Id, Extras, Buffer, Last);
      Conn.Out_Sequency := Conn.Out_Sequency + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Conn   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode (Conn.System_Id, Conn.Component_Id, Conn.Out_Sequency,
              Id, Extras, Buffer, Last);
      Conn.Out_Sequency := Conn.Out_Sequency + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8;
      Sequency     : Interfaces.Unsigned_8;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive)
   is
      Header   : V1_Header with Import,
        Address => Buffer (Buffer'First)'Address;
      Checksum : X25CRC.Checksum;
   begin
      Header.Stx     := Version_1_Code;
      Header.Len     := Interfaces.Unsigned_8 (Last - Packet_Payload_First);
      Header.Seq     := Sequency;
      Header.Sys_Id  := System_Id;
      Header.Comp_Id := Component_Id;
      Header.Msg_Id  := Id;

      for B of Buffer (Buffer'First + 1 .. Last) loop
         X25CRC.Update (Checksum, B);
      end loop;
      X25CRC.Update (Checksum, Extras);

      Buffer (Last + 1) := Checksum.High;
      Buffer (Last + 2) := Checksum.Low;
      Last := Last + 2;
   end Encode;

end MAVLink.V1;
