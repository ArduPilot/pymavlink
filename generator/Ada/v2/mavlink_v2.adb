pragma Ada_2022;

with X25CRC;

package body Mavlink_v2 is

   --------------------------
   -- Initialize_Signature --
   --------------------------

   procedure Initialize_Signature
     (Self      : in out Connection;
      Link_Id   : Interfaces.Unsigned_8;
      Key       : Signature_Key;
      Timestamp : Interfaces.Unsigned_64) is
   begin
      Self.Link_Id := Link_Id;

      Self.Key := SHA_256.Initial_Context;
      SHA_256.Update (Self.Key, SHA_256.Data (Key));

      Self.Timestamp     := Timestamp;
      Self.Use_Signature := True;
   end Initialize_Signature;

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
      Self.Position := Self.Position + 1;
      Self.Income_Buffer (Self.Position) := Value;

      if Self.Position <= Message_Data_Position_In_Buffer then
         return False;
      end if;

      if Self.Last = 0 then
         Self.Last := Self.Income_Buffer'First +
           Message_Data_Position_In_Buffer + --  header
             Natural (Header.Len - 1) + --  data len
           2; --  x25crc checksum

         if (Header.Inc_Flags and 1) > 0 then
            Self.Last := Self.Last + 13; --  SHA256 signature
         end if;
      end if;

      return Self.Position >= Self.Last
        or else Self.Position = Self.Income_Buffer'Last;
   end Parse_Byte;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Self      : Connection;
      Seq       : out Interfaces.Unsigned_8;
      Sys_Id    : out Interfaces.Unsigned_8;
      Comp_Id   : out Interfaces.Unsigned_8;
      Id        : out Msg_Id;
      Link_Id   : out Interfaces.Unsigned_8;
      Timestamp : out Interfaces.Unsigned_64;
      Signature : out Three_Boolean)
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      Seq     := Header.Seq;
      Sys_Id  := Header.Sys_Id;
      Comp_Id := Header.Comp_Id;
      Id      := Get_Message_Id (Self);

      Check_Message_Signature (Self, Link_Id, Timestamp, Signature);
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
     (Self : Connection) return Interfaces.Unsigned_8
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
     (Self : Connection) return Interfaces.Unsigned_8
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
     (Self : Connection) return Interfaces.Unsigned_8
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
     (Self : Connection) return Interfaces.Unsigned_8
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
      Sig    : constant Mavlink_v2.Signature with Import,
        Address => Self.Income_Buffer
          (Self.Income_Buffer'First +
             Message_Data_Position_In_Buffer +
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
      Link_Id   : out Interfaces.Unsigned_8;
      Timestamp : out Interfaces.Unsigned_64;
      Signature : out Three_Boolean)
   is
      Header : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
   begin
      if (Header.Inc_Flags and 1) > 0 then
         declare
            Last_Data : constant Positive := Self.Income_Buffer'First +
              Message_Data_Position_In_Buffer +
                Natural (Header.Len - 1);
            Sig       : constant Mavlink_v2.Signature with Import,
              Address => Self.Income_Buffer (Last_Data + 3)'Address;
         begin
            Link_Id   := Sig.Link_Id;
            Timestamp :=
              Interfaces.Unsigned_64 (Sig.Timestamp) and 16#FFFF_FFFF_FFFF#;

            Signature :=
              (if Sig.Sig = Calc_SHA (Self, Self.Income_Buffer
               (Self.Income_Buffer'First .. Last_Data + 2 + 7))
               then True
               else False);
         end;

      else
         Link_Id   := 0;
         Timestamp := 0;
         Signature := Unknown;
      end if;
   end Check_Message_Signature;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   function Get_Message_Data (Self : Connection) return Data_Buffer
   is
      Header    : constant V2_Header with Import,
        Address => Self.Income_Buffer'Address;
      Last_Data : constant Positive := Self.Income_Buffer'First +
        Message_Data_Position_In_Buffer +
          Natural (Header.Len - 1);
   begin
      return Self.Income_Buffer
        (Self.Income_Buffer'First + Message_Data_Position_In_Buffer ..
           Last_Data);
   end Get_Message_Data;

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
        Message_Data_Position_In_Buffer +
          Natural (Header.Len - 1);

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
      Last   : out Natural) is
   begin
      Last := Natural'Min
        (Buffer'Length,
         Self.Income_Buffer'First + Self.Position - 1);

      Buffer (Buffer'First .. Buffer'First + Last - 1) := Self.Income_Buffer
        (Self.Income_Buffer'First .. Self.Income_Buffer'First + Last - 1);
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
      Last   : in out Positive)
   is
      Local : V2_Header with Import,
        Address => Buffer (Buffer'First)'Address;
      L_Id  : Unsigned_64 := Unsigned_64 (Id);
      CRC   : X25CRC.Checksum;
   begin
      --  Truncate the message
      while Last > Buffer'First + Message_Data_Position_In_Buffer loop
         exit when Buffer (Last) /= 0;
         Last := Last - 1;
      end loop;

      Local.Stx       := 16#FD#;
      Local.Len       := Unsigned_8
        (Last - (Buffer'First + Message_Data_Position_In_Buffer - 1));
      Local.Inc_Flags := (if Self.Use_Signature then 1 else 0);
      Local.Cmp_Flags := 0;
      Local.Seq       := Self.Sequence_Id;
      Local.Sys_Id    := Self.System_Id;
      Local.Comp_Id   := Self.Component_Id;

      Self.Sequence_Id := Self.Sequence_Id + 1;

      Local.Id_Low := Unsigned_8 (L_Id and 16#FF#);
      L_Id := Shift_Right (L_Id, 8);
      Local.Id_Mid := Unsigned_8 (L_Id and 16#FF#);
      L_Id := Shift_Right (L_Id, 8);
      Local.Id_High := Unsigned_8 (L_Id and 16#FF#);

      for B of Buffer (Buffer'First + 1 .. Last) loop
         X25CRC.Update (CRC, B);
      end loop;
      X25CRC.Update (CRC, Extras);

      Buffer (Last + 1) := CRC.High;
      Buffer (Last + 2) := CRC.Low;
      Last := Last + 2;

      if Self.Use_Signature then
         declare
            S : Signature with Import, Address => Buffer (Last + 1)'Address;
         begin
            S.Link_Id   := Self.Link_Id;
            S.Timestamp := Timestamp_Type
              ((Self.Timestamp and 16#FFFF_FFFF_FFFF#));
            S.Sig       := Calc_SHA (Self, Buffer (Buffer'First .. Last + 7));
         end;

         Self.Timestamp := Self.Timestamp + 1;
         Last := Last + 13;
      end if;
   end Encode;

   --------------
   -- Calc_SHA --
   --------------

   function Calc_SHA
     (Self   : Connection;
      Buffer : Data_Buffer)
      return Data_Buffer
   is
      use SHA_256;
      SHA    : Context := Self.Key;
      Result : Digest_Type;
   begin
      Update (SHA, Data (Buffer));
      Result := Digest (SHA);
      return Data_Buffer (Result (Result'First .. Result'First + 5));
   end Calc_SHA;

end Mavlink_v2;
