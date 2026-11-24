pragma Ada_2022;

with MAVLink.X25CRC;

package body MAVLink.V2 is

   function System_Add_Length (Data : Incoming_Data) return Natural is
     (if (Data.Income_Buffer (Inc_Flags_Pos) and Inc_Flags_System32) /= 0
      then 3
      else 0)
     with Inline;

   -------------------
   -- Header_Length --
   -------------------

   function Header_Length (Data : Incoming_Data) return Positive is
     (10 + System_Add_Length (Data) +
        (if (Data.Income_Buffer (Inc_Flags_Pos) and Inc_Flags_Target32) /= 0
         then 4
         else 0));

   -----------------------
   -- Last_Message_Data --
   -----------------------

   function Last_Message_Data (Data : Incoming_Data) return Positive
   is
      Header : constant V2_Header with Import,
        Address => Data.Income_Buffer'Address;   
   begin
     return Data.Income_Buffer'First +
       Header_Length (Data) +
       Natural (Header.Len) - 1;
   end Last_Message_Data;

   ---------------------
   -- Read_Low_Endian --
   ---------------------

   function Read_Low_Endian
     (Incoming : Incoming_Data;
      First    : Positive;
      Length   : Positive)
      return Unsigned_32
   is
      Value : Unsigned_32 := 0;
   begin
      for I in 0 .. Length - 1 loop
         Value := Value or Shift_Left
           (Unsigned_32 (Incoming.Income_Buffer (First + I)), I * 8);
      end loop;
      return Value;
   end Read_Low_Endian;

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

   -------------------
   -- Set_System_Id --
   -------------------

   procedure Set_System_Id
     (Self  : in out Connection;
      Value : System_Id_Type) is
   begin
      Self.System_Id := Value;
   end Set_System_Id;

   ----------------------
   -- Set_Component_Id --
   ----------------------

   procedure Set_Component_Id
     (Self  : in out Connection;
      Value : Component_Id_Type) is
   begin
      Self.Component_Id := Value;
   end Set_Component_Id;

   ---------------------
   -- Set_Sequency_Id --
   ---------------------

   procedure Set_Sequency_Id
     (Self  : in out Connection;
      Value : Sequence_Id_Type) is
   begin
      Self.Sequence_Id := Value;
   end Set_Sequency_Id;

   -------------------
   -- Set_System_Id --
   -------------------

   procedure Set_System_Id
     (Self  : in out Out_Connection;
      Value : System_Id_Type) is
   begin
      Self.System_Id := Value;
   end Set_System_Id;

   ----------------------
   -- Set_Component_Id --
   ----------------------

   procedure Set_Component_Id
     (Self  : in out Out_Connection;
      Value : Component_Id_Type) is
   begin
      Self.Component_Id := Value;
   end Set_Component_Id;

   ---------------------
   -- Set_Sequency_Id --
   ---------------------

   procedure Set_Sequency_Id
     (Self  : in out Out_Connection;
      Value : Sequence_Id_Type) is
   begin
      Self.Sequence_Id := Value;
   end Set_Sequency_Id;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Incoming : in out Incoming_Data;
      Value    : Interfaces.Unsigned_8)
      return Boolean
   is
      Header : V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      if Incoming.Unsupported then
         Incoming.Unsupported := False;

         --  the V1 message length byte, claculate how many
         --  bytes to skip (length + 8 - 2[STX,LEN])
         Incoming.Discard_Remaining := Natural (Value) + 6;
         Clear (Incoming);
         return False;
      end if;

      if Incoming.Discard_Remaining > 0 then
         --  skip bytes of discarded message (like v1 message)
         Incoming.Discard_Remaining := @ - 1;
         return False;
      end if;

      if Incoming.Last > 0
        and then Incoming.Position >= Incoming.Last
      then
         --  the first byte of new message
         Clear (Incoming);
      end if;

      if Incoming.Position = 0 then
         if Value = 16#FE# then
            --  V1 message, skip it
            Incoming.Unsupported := True;
            return False;

         elsif Value /= Version_2_Code then
            --  not the STX or incorrect protocol version
            return False;
         end if;
      end if;

      Incoming.Position := Incoming.Position + 1;
      Incoming.Income_Buffer (Incoming.Position) := Value;

      if Incoming.Position = Inc_Flags_Pos then
         if (Value and Inc_Flags_Unsupported) /= 0 then
            --  Skip the entire unsupported frame, including any signature.
            --  Payload bytes may contain valid-looking frames and must
            --  not be parsed again.
            --  Bytes to skip by default (length + 10 - 3[STX,LEN,INC])
            Incoming.Discard_Remaining := Natural (Header.Len) + 7;

            if (Value and Inc_Flags_Signed) /= 0 then
               Incoming.Discard_Remaining := @ + 13;
            end if;

            if (Value and Inc_Flags_System32) /= 0 then
               Incoming.Discard_Remaining := @ + 3;
            end if;

            if (Value and Inc_Flags_Target32) /= 0 then
               Incoming.Discard_Remaining := @ + 4;
            end if;

            Clear (Incoming);
            return False;

         else
            --  we have LEN+INC and can calculate the full size
            --  of the message, do this
            Incoming.Last := Incoming.Income_Buffer'First +
              Header_Length (Incoming) + --  variable header
              Natural (Header.Len) + --  data len
              1; --  x25crc checksum - 1

            if (Header.Inc_Flags and Inc_Flags_Signed) > 0 then
               Incoming.Last := Incoming.Last + 13; --  SHA256 signature
            end if;
         end if;
      end if;

      --  check we have all message's bytes or just before the overflow
      return (Incoming.Last > 0
              and then Incoming.Position >= Incoming.Last)
        or else Incoming.Position = Incoming.Income_Buffer'Last;
   end Parse_Byte;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Parse_Byte (Self.Incoming, Value);
   end Parse_Byte;

   ----------------
   -- Parse_Byte --
   ----------------

   function Parse_Byte
     (Self  : in out In_Connection;
      Value : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Parse_Byte (Self.Incoming, Value);
   end Parse_Byte;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Incoming : Incoming_Data;
      Seq      : out Sequence_Id_Type;
      Sys_Id   : out System_Id_Type;
      Comp_Id  : out Component_Id_Type;
      Id       : out Msg_Id)
   is
      Header : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      Seq     := Header.Seq;
      Sys_Id  := Get_Message_System_Id (Incoming);
      Comp_Id := Get_Message_Component_Id (Incoming);
      Id      := Get_Message_Id (Incoming);
   end Get_Message_Information;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Self    : Connection;
      Seq     : out Sequence_Id_Type;
      Sys_Id  : out System_Id_Type;
      Comp_Id : out Component_Id_Type;
      Id      : out Msg_Id) is
   begin
      Get_Message_Information (Self.Incoming, Seq, Sys_Id, Comp_Id, Id);
   end Get_Message_Information;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Self    : In_Connection;
      Seq     : out Sequence_Id_Type;
      Sys_Id  : out System_Id_Type;
      Comp_Id : out Component_Id_Type;
      Id      : out Msg_Id) is
   begin
      Get_Message_Information (Self.Incoming, Seq, Sys_Id, Comp_Id, Id);
   end Get_Message_Information;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Incoming  : Incoming_Data;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) is
   begin
      Get_Message_Information (Incoming, Seq, Sys_Id, Comp_Id, Id);
      Check_Message_Signature (Incoming, Sign, Link_Id, Timestamp, Signature);
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
      Get_Message_Information
        (Self.Incoming, Sign, Seq, Sys_Id, Comp_Id, Id,
         Link_Id, Timestamp, Signature);
   end Get_Message_Information;

   -----------------------------
   -- Get_Message_Information --
   -----------------------------

   procedure Get_Message_Information
     (Self      : In_Connection;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) is
   begin
      Get_Message_Information
        (Self.Incoming, Sign, Seq, Sys_Id, Comp_Id, Id,
         Link_Id, Timestamp, Signature);
   end Get_Message_Information;

   --------------------
   -- Get_Message_Id --
   --------------------

   function Get_Message_Id (Incoming : Incoming_Data) return Msg_Id is
   begin
      return Msg_Id
        (Read_Low_Endian
          (Incoming, Msg_Id_Pos + System_Add_Length (Incoming), 3));
   end Get_Message_Id;

   --------------------
   -- Get_Message_Id --
   --------------------

   function Get_Message_Id (Self : Connection) return Msg_Id is
   begin
      return Get_Message_Id (Self.Incoming);
   end Get_Message_Id;

   --------------------
   -- Get_Message_Id --
   --------------------

   function Get_Message_Id (Self : In_Connection) return Msg_Id is
   begin
      return Get_Message_Id (Self.Incoming);
   end Get_Message_Id;

   -------------------------
   -- Get_Message_Sequnce --
   -------------------------

   function Get_Message_Sequnce
     (Incoming : Incoming_Data) return Sequence_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      return Header.Seq;
   end Get_Message_Sequnce;

   -------------------------
   -- Get_Message_Sequnce --
   -------------------------

   function Get_Message_Sequnce
     (Self : Connection) return Sequence_Id_Type is
   begin
      return Get_Message_Sequnce (Self.Incoming);
   end Get_Message_Sequnce;

   -------------------------
   -- Get_Message_Sequnce --
   -------------------------

   function Get_Message_Sequnce
     (Self : In_Connection) return Sequence_Id_Type is
   begin
      return Get_Message_Sequnce (Self.Incoming);
   end Get_Message_Sequnce;

   ---------------------------
   -- Get_Message_System_Id --
   ---------------------------

   function Get_Message_System_Id
     (Incoming : Incoming_Data) return System_Id_Type is
   begin
      return System_Id_Type
        (Read_Low_Endian
          (Incoming, Sys_Id_Pos, 1 + System_Add_Length (Incoming)));
   end Get_Message_System_Id;

   ---------------------------
   -- Get_Message_System_Id --
   ---------------------------

   function Get_Message_System_Id
     (Self : Connection) return System_Id_Type is
   begin
      return Get_Message_System_Id (Self.Incoming);
   end Get_Message_System_Id;

   ---------------------------
   -- Get_Message_System_Id --
   ---------------------------

   function Get_Message_System_Id
     (Self : In_Connection) return System_Id_Type is
   begin
      return Get_Message_System_Id (Self.Incoming);
   end Get_Message_System_Id;

   ------------------------------
   -- Get_Message_Component_Id --
   ------------------------------

   function Get_Message_Component_Id
     (Incoming : Incoming_Data) return Component_Id_Type is
   begin
      return Component_Id_Type
        (Incoming.Income_Buffer
          (Comp_Id_Pos + System_Add_Length (Incoming)));
   end Get_Message_Component_Id;

   ------------------------------
   -- Get_Message_Component_Id --
   ------------------------------

   function Get_Message_Component_Id
     (Self : Connection) return Component_Id_Type is
   begin
      return Get_Message_Component_Id (Self.Incoming);
   end Get_Message_Component_Id;

   ------------------------------
   -- Get_Message_Component_Id --
   ------------------------------

   function Get_Message_Component_Id
     (Self : In_Connection) return Component_Id_Type is
   begin
      return Get_Message_Component_Id (Self.Incoming);
   end Get_Message_Component_Id;

   -------------------------
   -- Get_Message_Link_Id --
   -------------------------

   function Get_Message_Link_Id
     (Incoming : Incoming_Data) return Link_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      if (Header.Inc_Flags and Inc_Flags_Signed) > 0 then
         declare
            Sig : constant MAVLink.V2.MAV_Signature with Import,
            Address => Incoming.Income_Buffer
               (Last_Message_Data (Incoming) + 3)'Address;
            --  last data + 1 + 2(crc)
         begin
            return Sig.Link_Id;
         end;
      else
         return 0;
      end if;
   end Get_Message_Link_Id;

   -------------------------
   -- Get_Message_Link_Id --
   -------------------------

   function Get_Message_Link_Id
     (Self : Connection) return Link_Id_Type is
   begin
      return Get_Message_Link_Id (Self.Incoming);
   end Get_Message_Link_Id;

   -------------------------
   -- Get_Message_Link_Id --
   -------------------------

   function Get_Message_Link_Id
     (Self : In_Connection) return Link_Id_Type is
   begin
      return Get_Message_Link_Id (Self.Incoming);
   end Get_Message_Link_Id;

   ----------------------------------
   -- Get_Message_Target_System_Id --
   ----------------------------------

   function Get_Message_Target_System_Id
     (Incoming       : Incoming_Data;
      Payload_Target : Target_Id_Type := 0)
      return Target_Id_Type
   is
      Header : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      if (Header.Inc_Flags and Inc_Flags_Target32) /= 0 then
         return Target_Id_Type
           (Read_Low_Endian
             (Incoming, Target_Id_Pos + System_Add_Length (Incoming), 4));
      end if;
      return Payload_Target;
   end Get_Message_Target_System_Id;

   ----------------------------------
   -- Get_Message_Target_System_Id --
   ----------------------------------

   function Get_Message_Target_System_Id
     (Self           : Connection;
      Payload_Target : Target_Id_Type := 0)
      return Target_Id_Type is
   begin
      return Get_Message_Target_System_Id (Self.Incoming, Payload_Target);
   end Get_Message_Target_System_Id;

   ----------------------------------
   -- Get_Message_Target_System_Id --
   ----------------------------------

   function Get_Message_Target_System_Id
     (Self           : In_Connection;
      Payload_Target : Target_Id_Type := 0)
      return Target_Id_Type is
   begin
      return Get_Message_Target_System_Id (Self.Incoming, Payload_Target);
   end Get_Message_Target_System_Id;

   -----------------------------
   -- Check_Message_Signature --
   -----------------------------

   procedure Check_Message_Signature
     (Incoming  : Incoming_Data;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean)
   is
      Header : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      if (Header.Inc_Flags and Inc_Flags_Signed) > 0 then
         declare
            Last_Data   : constant Positive := Last_Message_Data (Incoming);
            Sig         : constant MAVLink.V2.MAV_Signature with Import,
              Address => Incoming.Income_Buffer (Last_Data + 3)'Address;
            --  last data + 1 + 2(crc)

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

            Calc_SHA (Sign.Key, Incoming.Income_Buffer
                      (Incoming.Income_Buffer'First .. Last_Data + 2 + 7),
                      Message_SHA);

            Signature := (if Sig.Sig = Message_SHA then True else False);
         end;

      else
         Link_Id   := 0;
         Timestamp := 0;
         Signature := Unknown;
      end if;
   end Check_Message_Signature;

   -----------------------------
   -- Check_Message_Signature --
   -----------------------------

   procedure Check_Message_Signature
     (Self      : Connection;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) is
   begin
      Check_Message_Signature
        (Self.Incoming, Sign, Link_Id, Timestamp, Signature);
   end Check_Message_Signature;

   -----------------------------
   -- Check_Message_Signature --
   -----------------------------

   procedure Check_Message_Signature
     (Self      : In_Connection;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) is
   begin
      Check_Message_Signature
        (Self.Incoming, Sign, Link_Id, Timestamp, Signature);
   end Check_Message_Signature;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Incoming : Incoming_Data;
      Extras   : Interfaces.Unsigned_8)
      return Boolean
   is
      Header    : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
      Last_Data : constant Positive := Last_Message_Data (Incoming);

      CRC : X25CRC.Checksum;
   begin
      for B of Incoming.Income_Buffer
        (Incoming.Income_Buffer'First + 1 .. Last_Data)
      loop
         X25CRC.Update (CRC, B);
      end loop;
      X25CRC.Update (CRC, Extras);

      return Incoming.Income_Buffer (Last_Data + 1) = CRC.High
        and then Incoming.Income_Buffer (Last_Data + 2) = CRC.Low;
   end Is_CRC_Valid;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Self   : Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Is_CRC_Valid (Self.Incoming, Extras);
   end Is_CRC_Valid;

   ------------------
   -- Is_CRC_Valid --
   ------------------

   function Is_CRC_Valid
     (Self   : In_Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean is
   begin
      return Is_CRC_Valid (Self.Incoming, Extras);
   end Is_CRC_Valid;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer;
      Last     : out Natural)
   is
      Len : constant Natural := Natural'Min
        (Buffer'Length, Incoming.Income_Buffer'First + Incoming.Position - 1);
   begin
      Last := Buffer'First + Len - 1;
      Buffer (Buffer'First .. Last) := Incoming.Income_Buffer
        (Incoming.Income_Buffer'First ..
           Incoming.Income_Buffer'First + Len - 1);
   end Get_Buffer;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) is
   begin
      Get_Buffer (Self.Incoming, Buffer, Last);
   end Get_Buffer;

   ----------------
   -- Get_Buffer --
   ----------------

   procedure Get_Buffer
     (Self   : In_Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) is
   begin
      Get_Buffer (Self.Incoming, Buffer, Last);
   end Get_Buffer;

   -----------
   -- Clear --
   -----------

   procedure Clear (Incoming : in out Incoming_Data) is
   begin
      Incoming.Position := 0;
      Incoming.Last     := 0;
      Incoming.Discard_Remaining := 0;
   end Clear;

   -----------
   -- Clear --
   -----------

   procedure Clear (Self : in out Connection) is
   begin
      Clear (Self.Incoming);
   end Clear;

   -----------
   -- Clear --
   -----------

   procedure Clear (Self : in out In_Connection) is
   begin
      Clear (Self.Incoming);
   end Clear;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer;
      Last     : out Natural)
   is
      Header     : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
      First_Data : constant Natural :=
        Incoming.Income_Buffer'First +
          Header_Length (Incoming);
      Len        : Natural;
   begin
      Len  := Natural'Min (Natural (Header.Len), Buffer'Length);
      Last := Buffer'First + Len - 1;

      Buffer (Buffer'First .. Last) := Incoming.Income_Buffer
        (First_Data .. First_Data + Len - 1);
   end Get_Message_Data;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) is
   begin
      Get_Message_Data (Self.Incoming, Buffer, Last);
   end Get_Message_Data;

   ----------------------
   -- Get_Message_Data --
   ----------------------

   procedure Get_Message_Data
     (Self   : In_Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) is
   begin
      Get_Message_Data (Self.Incoming, Buffer, Last);
   end Get_Message_Data;

   --------------------
   -- Get_Msg_Length --
   --------------------

   function Get_Msg_Length
     (Incoming : Incoming_Data) return Interfaces.Unsigned_8
   is
      Header : constant V2_Header with Import,
        Address => Incoming.Income_Buffer'Address;
   begin
      return Header.Len;
   end Get_Msg_Length;

   --------------------
   -- Get_Msg_Length --
   --------------------

   function Get_Msg_Length (Self : Connection) return Interfaces.Unsigned_8 is
   begin
      return Get_Msg_Length (Self.Incoming);
   end Get_Msg_Length;

   --------------------
   -- Get_Msg_Length --
   --------------------

   function Get_Msg_Length
     (Self : In_Connection) return Interfaces.Unsigned_8 is
   begin
      return Get_Msg_Length (Self.Incoming);
   end Get_Msg_Length;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self          : in out Connection;
      Id            : Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Data, Buffer, Last, 0, Target_System);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self          : in out Connection;
      Id            : Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Sign          : in out Signature;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Sign, Data, Buffer, Last, Target_System);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self          : in out Out_Connection;
      Id            : Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Data, Buffer, Last, 0, Target_System);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (Self          : in out Out_Connection;
      Id            : Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Sign          : in out Signature;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0) is
   begin
      Encode
        (Self.System_Id, Self.Component_Id, Self.Sequence_Id,
         Id, Extras, Sign, Data, Buffer, Last, Target_System);
      Self.Sequence_Id := Self.Sequence_Id + 1;
   end Encode;

   ------------
   -- Encode --
   ------------

   procedure Encode
     (System_Id     : System_Id_Type;
      Component_Id  : Component_Id_Type;
      Sequence_Id   : Sequence_Id_Type;
      Id            : Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Inc_Flags     : Interfaces.Unsigned_8 := 0;
      Target_System : Target_Id_Type := 0)
   is
      System_Size  : constant Positive := (if System_Id > 255 then 4 else 1);
      Header_Size  : constant Positive :=
        9 + System_Size + (if Target_System > 255 then 4 else 0);
      Length       : Natural;
      First_Data   : Natural;
      Last_Data    : Natural := Data'Last;
      CRC          : X25CRC.Checksum;
      Variable_Pos : Positive := Buffer'First + Sys_Id_Pos - 1;

      ---------
      -- Put --
      ---------

      procedure Put (Value : Unsigned_32; Size : Positive) is
      begin
         for I in 0 .. Size - 1 loop
            Buffer (Variable_Pos) :=
              Unsigned_8 (Shift_Right (Value, I * 8) and 255);
            Variable_Pos := @ + 1;
         end loop;
      end Put;

      Header : V2_Header with Import,
        Address => Buffer (Buffer'First)'Address;      
   begin
      while Last_Data >= Data'First loop
         exit when Data (Last_Data) /= 0;
         Last_Data := Last_Data - 1;
      end loop;
      Length := Data'First + Last_Data - 1;

      Header.Stx       := Version_2_Code;
      Header.Len       := Unsigned_8 (Length);
      Header.Inc_Flags := Inc_Flags or
        (if System_Size = 4 then Inc_Flags_System32 else 0) or
        (if Target_System > 255 then Inc_Flags_Target32 else 0);
      Header.Cmp_Flags := 0;
      Header.Seq       := Sequence_Id;

      Put (Unsigned_32 (System_Id), System_Size);
      Put (Unsigned_32 (Component_Id), 1);
      Put (Unsigned_32 (Id), 3);
      if Target_System > 255 then
         Put (Unsigned_32 (Target_System), 4);
      end if;

      First_Data := Buffer'First + Header_Size;
      Last := First_Data + Length - 1;

      Buffer (First_Data .. Last) :=
        Data (Data'First .. Last_Data);

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
     (System_Id     : System_Id_Type;
      Component_Id  : Component_Id_Type;
      Sequence_Id   : Sequence_Id_Type;
      Id            : Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Sign          : in out Signature;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0) is
   begin
      Encode
        (System_Id, Component_Id, Sequence_Id, Id,
         Extras, Data, Buffer, Last, 1, Target_System);

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

end MAVLink.V2;
