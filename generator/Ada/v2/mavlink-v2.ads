
pragma Ada_2022;

with Interfaces;               use Interfaces;
with Mavlink.Raw_Floats;       use Mavlink.Raw_Floats;
with Mavlink.Raw_Long_Floats;  use Mavlink.Raw_Long_Floats;

private with Mavlink.SHA_256;

package Mavlink.V2 is

   pragma Preelaborate;

   type Msg_Id is mod 2 ** 24 with Size => 24;
   --  Message ID has 3 bytes

   type Data_Buffer is array (Positive range <>) of
     aliased Interfaces.Unsigned_8;

   Maximum_Buffer_Len : constant Positive := 280;

   type Timestamp_Type is mod 2 ** 48 with Size => 48;

   subtype Signature_Index is Positive range 1 .. 32;
   type Signature_Key is array (Signature_Index range <>) of
     Interfaces.Unsigned_8;

   type Three_Boolean is (False, True, Unknown);

   Version_2_Code : constant Interfaces.Unsigned_8 := 16#FD#;

   -- Arrays --

   type Unsigned_8_Array is array (Natural range <>) of Interfaces.Unsigned_8
     with Component_Size => 8;
   type Unsigned_16_Array is array (Natural range <>) of Interfaces.Unsigned_16
     with Component_Size => 16;
   type Unsigned_32_Array is array (Natural range <>) of Interfaces.Unsigned_32
     with Component_Size => 32;
   type Unsigned_64_Array is array (Natural range <>) of Interfaces.Unsigned_64
     with Component_Size => 64;
   type Integer_8_Array is array (Natural range <>) of Interfaces.Integer_8
     with Component_Size => 8;
   type Integer_16_Array is array (Natural range <>) of Interfaces.Integer_16
     with Component_Size => 16;
   type Integer_32_Array is array (Natural range <>) of Interfaces.Integer_32
     with Component_Size => 32;
   type Integer_64_Array is array (Natural range <>) of Interfaces.Integer_64
     with Component_Size => 64;
   type Short_Float_Array is array (Natural range <>) of Raw_Float
     with Component_Size => 32;
   type Long_Float_Array is array (Natural range <>) of Raw_Long_Float
     with Component_Size => 64;

   ----------------
   -- Connection --
   ----------------

   type Connection
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8)
   is private;

   procedure Initialize_Signature
     (Self      : in out Connection;
      Link_Id   : Interfaces.Unsigned_8;
      Key       : Signature_Key;
      Timestamp : Timestamp_Type);
   --  Initialize signature that will be used to generate SHA256 signature
   --  for packets

   function Parse_Byte
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8)
      return Boolean;
   --  Add the Value to the buffer and return True if a message has been read
   --  (collected all message data)

   --  Get the message's information that is in the connection's buffer --
   procedure Get_Message_Information
     (Self      : Connection;
      Seq       : out Interfaces.Unsigned_8;
      Sys_Id    : out Interfaces.Unsigned_8;
      Comp_Id   : out Interfaces.Unsigned_8;
      Id        : out Msg_Id;
      Link_Id   : out Interfaces.Unsigned_8;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean);
   --  Returns information about the current message in the buffer.
   --  Should be called only after Parse_Byte returned True.
   --  Timestamp contains 48 bits of the timestamp (others set ot 0)
   --  Signature is set to Unknown if SHA256 is not used. In another case it is
   --  set to True/False depends on whether the checksum is valid for the
   --  message.

   function Get_Message_Id (Self : Connection) return Msg_Id;
   --  Returns message's ID

   function Get_Message_Sequnce
     (Self : Connection) return Interfaces.Unsigned_8;
   --  Returns message's Seq

   function Get_Message_System_Id
     (Self : Connection) return Interfaces.Unsigned_8;
   --  Returns message's Sys_Id

   function Get_Message_Component_Id
     (Self : Connection) return Interfaces.Unsigned_8;
   --  Returns message's Comp_Id

   function Get_Message_Link_Id
     (Self : Connection) return Interfaces.Unsigned_8;
   --  Returns message's Link_Id. Returns 0 if the message does not have
   --  the Signature.

   procedure Check_Message_Signature
     (Self      : Connection;
      Link_Id   : out Interfaces.Unsigned_8;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean);
   --  Returns the message's Signature. See Get_Message_Information.

   procedure Get_Buffer
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural);
   --  Returns data from the internal buffer filled with Parse_Byte.

   procedure Drop_Message (Self : in out Connection);
   --  Delete current message from the buffer.
   --  Should be called only after Parse_Byte returned True

   --------------------
   -- Out_Connection --
   --------------------

   type Out_Connection
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8)
   is private;

   procedure Initialize_Signature
     (Self      : in out Out_Connection;
      Link_Id   : Interfaces.Unsigned_8;
      Key       : Signature_Key;
      Timestamp : Timestamp_Type);
   --  Initialize signature that will be used to generate SHA256 signature
   --  for packets

private

   type V2_Header is record
      Stx       : Interfaces.Unsigned_8;
      Len       : Interfaces.Unsigned_8;
      Inc_Flags : Interfaces.Unsigned_8;
      Cmp_Flags : Interfaces.Unsigned_8;
      Seq       : Interfaces.Unsigned_8;
      Sys_Id    : Interfaces.Unsigned_8;
      Comp_Id   : Interfaces.Unsigned_8;
      Id_Low    : Interfaces.Unsigned_8;
      Id_Mid    : Interfaces.Unsigned_8;
      Id_High   : Interfaces.Unsigned_8;
   end record;

   for V2_Header use record
      Stx       at 0 range 0 .. 7;
      Len       at 1 range 0 .. 7;
      Inc_Flags at 2 range 0 .. 7;
      Cmp_Flags at 3 range 0 .. 7;
      Seq       at 4 range 0 .. 7;
      Sys_Id    at 5 range 0 .. 7;
      Comp_Id   at 6 range 0 .. 7;
      Id_Low    at 7 range 0 .. 7;
      Id_Mid    at 8 range 0 .. 7;
      Id_High   at 9 range 0 .. 7;
   end record;

   type SHA_Digest is array (1 .. 6) of Interfaces.Unsigned_8;

   type Signature is record
      Link_Id   : Interfaces.Unsigned_8;
      Timestamp : Data_Buffer (1 .. 6);
      Sig       : SHA_Digest;
   end record;

   for Signature use record
      Link_Id   at 0 range 0 .. 7;
      Timestamp at 1 range 0 .. 47;
      Sig       at 7 range 0 .. 47;
   end record;

   -- Settings_Data --

   type Settings_Data is record
      Sequence_Id   : Interfaces.Unsigned_8  := 0;
      Use_Signature : Boolean                := False;
      Link_Id       : Interfaces.Unsigned_8  := 0;
      Key           : SHA_256.Context;
      Timestamp     : Timestamp_Type         := 0;
   end record;

   -- Connection --

   type Connection
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8)
   is record
      Settings      : Settings_Data;

      -- Income
      Income_Buffer : Data_Buffer (1 .. Maximum_Buffer_Len);
      Position      : Natural := 0;
      Last          : Natural := 0;
   end record;

   procedure Encode
     (Self   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);

   function Is_CRC_Valid
     (Self   : Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Message_Data
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural);

   -- Out_Connection --

   type Out_Connection
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8)
   is record
      Settings : Settings_Data;
   end record;

   procedure Encode
     (Self   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);

   -- Utils --

   procedure Encode
     (System_Id    : Interfaces.Unsigned_8;
      Component_Id : Interfaces.Unsigned_8;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Settings     : in out Settings_Data;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive);

   procedure Calc_SHA
     (Settings : Settings_Data;
      Buffer   : Data_Buffer;
      Result   : out SHA_Digest);

   --  Positions of the messages parts minus 1, to use in construction like
   --  Buff'First + Packet_Payload_First
   Packet_Payload_First : constant Positive := 10;

end Mavlink.V2;
