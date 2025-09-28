pragma Ada_2022;

with MAVLink.X25CRC;

package body MAVLink.V2 is

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize
     (Self      : out Signature;
      Link_Id   : Link_Id_Type;
      Key       : Signature_Key;
      Timestamp : Timestamp_Type) is
   begin
      Self.Link_Id   := Link_Id;
      Self.Timestamp := Timestamp;

      Self.Key := SHA_256.Initial_Context;
      SHA_256.Update (Self.Key, SHA_256.Data (Key));
   end Initialize;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8)
      return Boolean
   is
      Header : V2_Header with Import, Address => Self.Income_Buffer'Address;
   begin
      if Self.Position = 0
        and then Value /= Version_2_Code
      then
         return False;
      end if;

      Self.Position := Self.Position + 1;
      Self.Income_Buffer (Self.Position) := Value;

      if Self.Position < Packet_Payload_First then
         --  no header yet
         return False;
      end if;

      if Self.Last = 0 then
         Self.Last := Self.Income_Buffer'First +
           Packet_Payload_First + --  header
             Natural (Header.Len - 1) + --  data len
           2; --  x25crc checksum

         if (Header.Inc_Flags and 1) > 0 then
            Self.Last := Self.Last + 13; --  SHA256 signature
         end if;
      end if;

      return Self.Position = Self.Last
        or else Self.Position = Self.Income_Buffer'Last;
   end Parse_Byte;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Self    : Connection;
      Seq     : out Sequence_Id_Type;
      Sys_Id  : out System_Id_Type;
      Comp_Id : out Component_Id_Type;
      Id      : out Msg_Id)
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      Seq     := Header.Seq;
      Sys_Id  := Header.Sys_Id;
      Comp_Id := Header.Comp_Id;
      Id      := Get_Message_Id (Self);
   end Get_Message_Information;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Self      : Connection;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) is
   begin
      Get_Message_Information (Self, Seq, Sys_Id, Comp_Id, Id);
      Check_Message_Signature (Self, Sign, Link_Id, Timestamp, Signature);
   end Get_Message_Information;

   --------------------
   -- Get_Message_Id --
   --------------------

   function Get_Message_Id (Self : Connection) return Msg_Id
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;

   begin
      return Msg_Id (Shift_Left (Unsigned_64 (Header.Id_High), 16) +
                       Shift_Left (Unsigned_64 (Header.Id_Mid), 8) +
                         Unsigned_64 (Header.Id_Low));
   end Get_Message_Id;

   -------------------------
   -- Get_Message_Sequnce --
   -------------------------

   function Get_Message_Sequnce
     (Self : Connection) return Sequence_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      return Header.Seq;
   end Get_Message_Sequnce;

   ---------------------------
   -- Get_Message_System_Id --
   ---------------------------

   function Get_Message_System_Id
     (Self : Connection) return System_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      return Header.Sys_Id;
   end Get_Message_System_Id;

   ------------------------------
   -- Get_Message_Component_Id --
   ------------------------------

   function Get_Message_Component_Id
     (Self : Connection) return Component_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      return Header.Comp_Id;
   end Get_Message_Component_Id;

   -------------------------
   -- Get_Message_Link_Id --
   -------------------------

   function Get_Message_Link_Id
     (Self : Connection) return Link_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
      Sig    : constant MAVLink.V2.MAV_Signature with Import,
        Address => Self.Income_Buffer
          (Self.Income_Buffer'First +
             Packet_Payload_First +
               Natural (Header.Len - 1) + 3)'Address;
   begin
      if (Header.Inc_Flags and 1) > 0 then
         return Sig.Link_Id;
      else
         return 0;
      end if;
   end Get_Message_Link_Id;

   -----------------------------
   -- Check_Message_Signature --
   -----------------------------

   procedure Check_Message_Signature
     (Self      : Connection;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean)
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      if (Header.Inc_Flags and 1) > 0 then
         declare
            Last_Data   : constant Positive := Self.Income_Buffer'First +
              Packet_Payload_First +
                Natural (Header.Len - 1);
            Sig         : constant MAVLink.V2.MAV_Signature with Import,
              Address => Self.Income_Buffer (Last_Data + 3)'Address;
            Message_SHA : SHA_Digest;
         begin
            Link_Id   := Sig.Link_Id;

            Timestamp := 0;
            for Index in reverse Sig.Timestamp'First .. Sig.Timestamp'Last loop
               Timestamp := Timestamp + Timestamp_Type (Sig.Timestamp (Index));
               if Index > Sig.Timestamp'First then
                  Timestamp := Timestamp_Type
                    (Shift_Left (Unsigned_64 (Timestamp), 8));
               end if;
            end loop;

            Calc_SHA (Sign.Key, Self.Income_Buffer
                      (Self.Income_Buffer'First .. Last_Data + 2 + 7),
                      Message_SHA);

            Signature := (if Sig.Sig = Message_SHA then True else False);
         end;

      else
         Link_Id   := 0;
         Timestamp := 0;
         Signature := Unknown;
      end if;
   end Check_Message_Signature;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Self   : Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean
   is
      Header    : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
      Last_Data : constant Positive := Self.Income_Buffer'First +
        Packet_Payload_First +
          Natural (Header.Len) - 1;

      CRC : X25CRC.Checksum;
   begin
      for B of Self.Income_Buffer
        (Self.Income_Buffer'First + 1 .. Last_Data)
      loop
         X25CRC.Update (CRC, B);
      end loop;
      X25CRC.Update (CRC, Extras);

      return Self.Income_Buffer (Last_Data + 1) = CRC.High
        and then Self.Income_Buffer (Last_Data + 2) = CRC.Low;
   end Is_CRC_Valid;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural)
   is
      Len : constant Natural := Natural'Min
        (Buffer'Length, Self.Income_Buffer'First + Self.Position - 1);
   begin
      Last := Buffer'First + Len - 1;
      Buffer (Buffer'First .. Last) := Self.Income_Buffer
        (Self.Income_Buffer'First .. Self.Income_Buffer'First + Len - 1);
   end Get_Buffer;

   ------------------
   -- Drop_Message --
   ------------------

   procedure Drop_Message (Self : in out Connection)
   is
      Len : constant Natural := Self.Position - Self.Last;
   begin
      Self.Income_Buffer
        (Self.Income_Buffer'First .. Self.Income_Buffer'First + Len - 1) :=
        Self.Income_Buffer (Self.Last + 1 .. Self.Position);

      Self.Position := Self.Income_Buffer'First + Len - 1;
      Self.Last     := 0;
   end Drop_Message;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Buffer, Last);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Sign   : in out Signature;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Sign, Buffer, Last);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Buffer, Last);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Sign   : in out Signature;
      Buffer : in out Data_Buffer;
      Last   : in out Positive) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Sign, Buffer, Last);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type;
      Sequence_Id  : Sequence_Id_Type;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive;
      Inc_Flags    : Interfaces.Unsigned_8 := 0)
   is
      Header : V2_Header with Import,
        Address => Buffer (Buffer'First)'Address;
      L_Id   : Unsigned_64 := Unsigned_64 (Id);
      CRC    : X25CRC.Checksum;
   begin
      --  Truncate the message
      while Last > Buffer'First + Packet_Payload_First loop
         exit when Buffer (Last) /= 0;
         Last := Last - 1;
      end loop;

      Header.Stx       := Version_2_Code;
      Header.Len       := Unsigned_8
        (Last - (Buffer'First + Packet_Payload_First - 1));
      Header.Inc_Flags := Inc_Flags;
      Header.Cmp_Flags := 0;
      Header.Seq       := Sequence_Id;
      Header.Sys_Id    := System_Id;
      Header.Comp_Id   := Component_Id;

      Header.Id_Low := Unsigned_8 (L_Id and 16#FF#);
      L_Id := Shift_Right (L_Id, 8);
      Header.Id_Mid := Unsigned_8 (L_Id and 16#FF#);
      L_Id := Shift_Right (L_Id, 8);
      Header.Id_High := Unsigned_8 (L_Id and 16#FF#);

      for B of Buffer (Buffer'First + 1 .. Last) loop
         X25CRC.Update (CRC, B);
      end loop;
      X25CRC.Update (CRC, Extras);

      Buffer (Last + 1) := CRC.High;
      Buffer (Last + 2) := CRC.Low;
      Last := Last + 2;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type;
      Sequence_Id  : Sequence_Id_Type;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Sign         : in out Signature;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive) is
   begin
      Encode
        (System_Id, Component_Id, Sequence_Id, Id, Extras, Buffer, Last, 1);

      declare
         Sig  : MAV_Signature with Import,
           Address => Buffer (Last + 1)'Address;
         Time : Timestamp_Type := Sign.Timestamp;
      begin
         Sig.Link_Id := Sign.Link_Id;

         for Index in Sig.Timestamp'Range loop
            Sig.Timestamp (Index) := Unsigned_8 (Time and 16#FF#);
            Time := Timestamp_Type (Shift_Right (Unsigned_64 (Time), 8));
         end loop;

         Calc_SHA (Sign.Key, Buffer (Buffer'First .. Last + 7), Sig.Sig);
      end;

      Sign.Timestamp := Sign.Timestamp + 1;
      Last := Last + 13;
   end Encode;

   --------------
   -- Calc_SHA --
   --------------

   procedure Calc_SHA
     (Key    : SHA_256.Context;
      Buffer : Data_Buffer;
      Result : out SHA_Digest)
   is
      use SHA_256;
      SHA : Context := Key;
      D   : Digest_Type;
   begin
      Update (SHA, Data (Buffer));
      Digest (SHA, D);
      Result := SHA_Digest (D (D'First .. D'First + 5));
   end Calc_SHA;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural)
   is
      Header    : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
      Last_Data : constant Positive := Self.Income_Buffer'First +
        Packet_Payload_First +
          Natural (Header.Len) - 1;
   begin
      Last := Buffer'First + Natural (Header.Len - 1);
      Buffer (Buffer'First .. Last) := Self.Income_Buffer
        (Self.Income_Buffer'First + Packet_Payload_First ..
           Last_Data);
   end Get_Message_Data;

end MAVLink.V2;
