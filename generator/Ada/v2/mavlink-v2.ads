
pragma Ada_2022;

with Interfaces;               use Interfaces;
with MAVLink.Raw_Floats;       use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats;  use MAVLink.Raw_Long_Floats;

private with MAVLink.SHA_256;

package MAVLink.V2 is

   pragma Preelaborate;

   type Msg_Id is mod 2 ** 24 with Size => 24;
   --  Message ID has 3 bytes

   type Data_Buffer is array (Positive range <>) of
     aliased Interfaces.Unsigned_8;

   Maximum_Buffer_Len : constant Positive := 280;

   type System_Id_Type is new Interfaces.Unsigned_8;
   type Component_Id_Type is new Interfaces.Unsigned_8;
   type Sequence_Id_Type is new Interfaces.Unsigned_8;
   type Link_Id_Type is new Interfaces.Unsigned_8;

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

   ---------------
   -- Signature --
   ---------------

   type Signature is private;

   procedure Initialize
     (Self      : out Signature;
      Link_Id   : Link_Id_Type;
      Key       : Signature_Key;
      Timestamp : Timestamp_Type);
   --  Initialize signature that will be used to generate SHA256 signature
   --  for packets

   ----------------
   -- Connection --
   ----------------

   type Connection
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type)
   is private;

   function Parse_Byte
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8)
      return Boolean;
   --  Add the Value to the buffer and return True if a message has been read
   --  (collected all message data)

   --  Get the message's information that is in the connection's buffer --
   procedure Get_Message_Information
     (Self      : Connection;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean);
   --  Returns information about the current message in the buffer.
   --  Should be called only after Parse_Byte returned True.
   --  Timestamp contains 48 bits of the timestamp (others set ot 0)
   --  Signature is set to Unknown if SHA256 is not used. In another case it is
   --  set to True/False depends on whether the checksum is valid for the
   --  message.

   --  Get the message's information that is in the connection's buffer --
   procedure Get_Message_Information
     (Self    : Connection;
      Seq     : out Sequence_Id_Type;
      Sys_Id  : out System_Id_Type;
      Comp_Id : out Component_Id_Type;
      Id      : out Msg_Id);
   --  Same as above but do not check V2 signature

   function Get_Message_Id (Self : Connection) return Msg_Id;
   --  Returns message's ID

   function Get_Message_Sequnce
     (Self : Connection) return Sequence_Id_Type;
   --  Returns message's Seq

   function Get_Message_System_Id
     (Self : Connection) return System_Id_Type;
   --  Returns message's Sys_Id

   function Get_Message_Component_Id
     (Self : Connection) return Component_Id_Type;
   --  Returns message's Comp_Id

   function Get_Message_Link_Id
     (Self : Connection) return Link_Id_Type;
   --  Returns message's Link_Id. Returns 0 if the message does not have
   --  the Signature.

   procedure Check_Message_Signature
     (Self      : Connection;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
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
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type)
   is private;

private

   type V2_Header is record
      Stx       : Interfaces.Unsigned_8;
      Len       : Interfaces.Unsigned_8;
      Inc_Flags : Interfaces.Unsigned_8;
      Cmp_Flags : Interfaces.Unsigned_8;
      Seq       : Sequence_Id_Type;
      Sys_Id    : System_Id_Type;
      Comp_Id   : Component_Id_Type;
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

   type MAV_Signature is record
      Link_Id   : Link_Id_Type;
      Timestamp : Data_Buffer (1 .. 6);
      Sig       : SHA_Digest;
   end record;

   for MAV_Signature use record
      Link_Id   at 0 range 0 .. 7;
      Timestamp at 1 range 0 .. 47;
      Sig       at 7 range 0 .. 47;
   end record;

   type Signature is record
      Link_Id   : Link_Id_Type;
      Key       : SHA_256.Context;
      Timestamp : Timestamp_Type := 0;
   end record;

   -- Connection --

   type Connection
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type)
   is record
      Sequence_Id   : Sequence_Id_Type := 0;

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

   procedure Encode
     (Self   : in out Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Sign   : in out Signature;
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
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type)
   is record
      Sequence_Id : Sequence_Id_Type := 0;
   end record;

   procedure Encode
     (Self   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);

   procedure Encode
     (Self   : in out Out_Connection;
      Id     : Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Sign   : in out Signature;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);

   -- Utils --

   procedure Encode
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type;
      Sequence_Id  : Sequence_Id_Type;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive;
      Inc_Flags    : Interfaces.Unsigned_8 := 0);

   procedure Encode
     (System_Id    : System_Id_Type;
      Component_Id : Component_Id_Type;
      Sequence_Id  : Sequence_Id_Type;
      Id           : Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Sign         : in out Signature;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive);

   procedure Calc_SHA
     (Key    : SHA_256.Context;
      Buffer : Data_Buffer;
      Result : out SHA_Digest);

   --  Positions of the messages parts minus 1, to use in construction like
   --  Buff'First + Packet_Payload_First
   Packet_Payload_First : constant Positive := 10;

end MAVLink.V2;
