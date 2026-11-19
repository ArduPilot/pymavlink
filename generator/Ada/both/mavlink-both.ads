
pragma Ada_2022;

with Interfaces;               use Interfaces;
with MAVLink.Raw_Floats;       use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats;  use MAVLink.Raw_Long_Floats;
with MAVLink.V1;               use MAVLink.V1;
with MAVLink.V2;               use MAVLink.V2;

private with MAVLink.SHA_256;
private with MAVLink.X25CRC;

package MAVLink.Both is

   pragma Pure;

   type Three_Boolean is (False, True, Unknown);

   type Parse_Result is (None, Version_1, Version_2, Unknown);

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

   type Connection is private;

   procedure Set_System_Id
     (Self  : in out Connection;
      Value : System_Id_Type);

   procedure Set_Component_Id
     (Self  : in out Connection;
      Value : Component_Id_Type);

   procedure Set_Sequency_Id
     (Self  : in out Connection;
      Value : Sequence_Id_Type);

   function Parse_Byte
     (Self  : in out Connection;
      Value : Interfaces.Unsigned_8)
      return Parse_Result with Inline;
   --  Add the Value to the buffer and return Version_1/Version_2 if
   --  a V1 or V2 message has been read (collected all message data)
   --  All methods that work with incoming data (like Get_*, Check_*)
   --   should be called only when this method not returned None.
      
   function Get_Message_Protocol (Self : Connection) return Parse_Result;

   --  Get the message's information that is in the connection's buffer --
   procedure Get_Message_Information
     (Self      : Connection;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out MAVLink.V2.Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) with Inline;
   --  Returns information about the current message in the buffer.
   --  Should be called only after Parse_Byte returned True.
   --  Timestamp contains 48 bits of the timestamp (others set ot 0)
   --  Signature is set to Unknown if SHA256 is not used. In another case
   --  it is set to True/False depends on whether the checksum is valid
   --  for the message.

   --  Get the message's information that is in the connection's buffer --
   procedure Get_Message_Information
     (Self    : Connection;
      Seq     : out Sequence_Id_Type;
      Sys_Id  : out System_Id_Type;
      Comp_Id : out Component_Id_Type;
      Id      : out MAVLink.V2.Msg_Id) with Inline;
   --  Same as above but do not check V2 signature

   function Get_Message_Length
     (Self : Connection) return Interfaces.Unsigned_8 with Inline;
   --  Returns length field from the packet header from the
   --  incoming buffer

   function Get_Message_Id
     (Self : Connection) return MAVLink.V1.Msg_Id with Inline;
   --  Returns message's ID for V1 message

   function Get_Message_Id
     (Self : Connection) return MAVLink.V2.Msg_Id with Inline;
   --  Returns message's ID for V2 message

   function Get_Message_Sequnce
     (Self : Connection) return Sequence_Id_Type with Inline;
   --  Returns message's Seq

   function Get_Message_System_Id
     (Self : Connection) return System_Id_Type with Inline;
   --  Returns message's Sys_Id

   function Get_Message_Target_System_Id
     (Self           : Connection;
      Payload_Target : Target_Id_Type := 0)
      return Target_Id_Type;
   --  Header destination, or the payload's target byte if not extended.

   function Get_Message_Component_Id
     (Self : Connection) return Component_Id_Type with Inline;
   --  Returns message's Comp_Id

   function Get_Message_Link_Id
     (Self : Connection) return Link_Id_Type with Inline;
   --  Returns message's Link_Id. Returns 0 if the message does not have
   --  the Signature.

   procedure Check_Message_Signature
     (Self      : Connection;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) with Inline;
   --  Returns the message's Signature. See Get_Message_Information.

   procedure Get_Buffer
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) with Inline;
   --  Returns data from the internal buffer filled with Parse_Byte.

   procedure Clear (Self : in out Connection) with Inline;
   --  Delete all data from the incoming buffer

   --  for internal usage by messages --

   function Is_CRC_Valid
     (Self   : Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Message_Data
     (Self   : Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural);
   --  Copy data from the internal incoming buffer to Buffer,
   --  and set Last to the index of the last copied byte.
   --  Truncate the data if Buffer is too small to hold it all.
   --  For V2

   procedure Get_Message_Data
     (Self   : Connection;
      Buffer : out Data_Buffer);
   -- For V1

   procedure Encode
     (Self          : in out Connection;
      Id            : MAVLink.V2.Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0);
   --  For V2 internal usage

   procedure Encode
     (Self          : in out Connection;
      Id            : MAVLink.V2.Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Sign          : in out Signature;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0);
   --  For V2 internal usage

   procedure Encode
     (Self   : in out Connection;
      Id     : MAVLink.V1.Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);
   --  For V1 internal usage

   -------------------
   -- In_Connection --
   -------------------

   type In_Connection is private;
   --  For incoming messages only

   function Parse_Byte
     (Self  : in out In_Connection;
      Value : Interfaces.Unsigned_8)
      return Parse_Result with Inline;

   function Get_Message_Protocol (Self : In_Connection) return Parse_Result;

   procedure Get_Message_Information
     (Self      : In_Connection;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out MAVLink.V2.Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) with Inline;

   procedure Get_Message_Information
     (Self    : In_Connection;
      Seq     : out Sequence_Id_Type;
      Sys_Id  : out System_Id_Type;
      Comp_Id : out Component_Id_Type;
      Id      : out MAVLink.V2.Msg_Id) with Inline;

   function Get_Message_Length
     (Self : In_Connection) return Interfaces.Unsigned_8 with Inline;
   --  Returns length field from the packet header from the
   --  incoming buffer

   function Get_Message_Id
     (Self : In_Connection) return MAVLink.V1.Msg_Id with Inline;

   function Get_Message_Id
     (Self : In_Connection) return MAVLink.V2.Msg_Id with Inline;

   function Get_Message_Sequnce
     (Self : In_Connection) return Sequence_Id_Type with Inline;

   function Get_Message_System_Id
     (Self : In_Connection) return System_Id_Type with Inline;

   function Get_Message_Target_System_Id
     (Self           : In_Connection;
      Payload_Target : Target_Id_Type := 0)
      return Target_Id_Type;
   --  Header destination, or the payload's target byte if not extended.

   function Get_Message_Component_Id
     (Self : In_Connection) return Component_Id_Type with Inline;

   function Get_Message_Link_Id
     (Self : In_Connection) return Link_Id_Type with Inline;

   procedure Check_Message_Signature
     (Self      : In_Connection;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean) with Inline;

   procedure Get_Buffer
     (Self   : In_Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural) with Inline;

   procedure Clear (Self : in out In_Connection) with Inline;

   --  for internal usage by messages --

   function Is_CRC_Valid
     (Self   : In_Connection;
      Extras : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Message_Data
     (Self   : In_Connection;
      Buffer : out Data_Buffer;
      Last   : out Natural);
   --  Copy data from the internal incoming buffer to Buffer,
   --  and set Last to the index of the last copied byte.
   --  Truncate the data if Buffer is too small to hold it all.
   --  For V2

   procedure Get_Message_Data
     (Self   : In_Connection;
      Buffer : out Data_Buffer);
   -- For V1

   --------------------
   -- Out_Connection --
   --------------------

   type Out_Connection is private;
   --  For outcoming messages only

   procedure Set_System_Id
     (Self  : in out Out_Connection;
      Value : System_Id_Type);

   procedure Set_Component_Id
     (Self  : in out Out_Connection;
      Value : Component_Id_Type);

   procedure Set_Sequency_Id
     (Self  : in out Out_Connection;
      Value : Sequence_Id_Type);

   --  for internal usage by messages --

   procedure Encode
     (Self          : in out Out_Connection;
      Id            : MAVLink.V2.Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0);
   --  For V2 internal usage

   procedure Encode
     (Self          : in out Out_Connection;
      Id            : MAVLink.V2.Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Sign          : in out Signature;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0);
   --  For V2 internal usage

   procedure Encode
     (Self   : in out Out_Connection;
      Id     : MAVLink.V1.Msg_Id;
      Extras : Interfaces.Unsigned_8;
      Buffer : in out Data_Buffer;
      Last   : in out Positive);
   --  For V1 internal usage

private

   type V1_Header is record
      Stx     : Interfaces.Unsigned_8;
      Len     : Interfaces.Unsigned_8;
      Seq     : Sequence_Id_Type;
      Sys_Id  : Short_System_Id_Type;
      Comp_Id : Component_Id_Type;
      Msg_Id  : Interfaces.Unsigned_8;
   end record;

   for V1_Header use record
      Stx       at 0 range 0 .. 7;
      Len       at 1 range 0 .. 7;
      Seq       at 2 range 0 .. 7;
      Sys_Id    at 3 range 0 .. 7;
      Comp_Id   at 4 range 0 .. 7;
      Msg_Id    at 5 range 0 .. 7;
   end record;

   Packet_Checksum_Length   : constant := 2;

   Packet_Control_Info_Size : constant :=
     Packet_Marker_Length + Packet_Header_Length + Packet_Checksum_Length;

   Pos_Len                 : constant Natural := 2;
   Pos_Sequence            : constant Natural := 3;
   Pos_Target_System_Id    : constant Natural := 4;
   Pos_Target_Component_Id : constant Natural := 5;
   Pos_Msg_Id              : constant Natural := 6;

   type V2_Header is record
      Stx       : Interfaces.Unsigned_8;
      Len       : Interfaces.Unsigned_8;
      Inc_Flags : Interfaces.Unsigned_8;
      Cmp_Flags : Interfaces.Unsigned_8;
      Seq       : Sequence_Id_Type;
      Sys_Id    : Interfaces.Unsigned_8;
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

   Inc_Flags_Pos : constant := 3;
   --  Position of the Inc_Flags in the header
   Sys_Id_Pos    : constant := 6;
   Comp_Id_Pos   : constant := 7;
   Msg_Id_Pos    : constant := 8;
   Target_Id_Pos : constant := 11;

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

   type Incoming_Data is record
      Income_Buffer     : Data_Buffer (1 .. Maximum_Buffer_Len);
      Position          : Natural := 0;
      Last              : Natural := 0;
      Checksum          : X25CRC.Checksum;
      Discard_Remaining : Natural := 0;
   end record;

   -- Connection --

   type Connection is record
      System_Id    : System_Id_Type := 1;
      Component_Id : Component_Id_Type := 1;
      Sequence_Id  : Sequence_Id_Type := 0;
      Incoming     : Incoming_Data;
      Out_Sequency : Interfaces.Unsigned_8 := 0;
   end record;

   -- In_Connection --

   type In_Connection is record
      Incoming : Incoming_Data;
   end record;

   -- Out_Connection --

   type Out_Connection is record
      System_Id    : System_Id_Type := 1;
      Component_Id : Component_Id_Type := 1;
      Sequence_Id  : Sequence_Id_Type := 0;
   end record;

   -- Utils --

   function Parse_Byte
     (Incoming : in out Incoming_Data;
      Value    : Interfaces.Unsigned_8)
      return Parse_Result;

   function Get_Message_Protocol
     (Incoming : Incoming_Data) return Parse_Result;

   procedure Get_Message_Information
     (Incoming : Incoming_Data;
      Seq      : out Sequence_Id_Type;
      Sys_Id   : out System_Id_Type;
      Comp_Id  : out Component_Id_Type;
      Id       : out MAVLink.V2.Msg_Id);

   procedure Get_Message_Information
     (Incoming  : Incoming_Data;
      Sign      : Signature;
      Seq       : out Sequence_Id_Type;
      Sys_Id    : out System_Id_Type;
      Comp_Id   : out Component_Id_Type;
      Id        : out MAVLink.V2.Msg_Id;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean);

   function Get_Message_Length
     (Incoming : Incoming_Data) return Interfaces.Unsigned_8;

   function Get_Message_Id
     (Incoming : Incoming_Data) return MAVLink.V2.Msg_Id;

   function Get_Message_Sequnce
     (Incoming : Incoming_Data) return Sequence_Id_Type;

   function Get_Message_System_Id
     (Incoming : Incoming_Data) return System_Id_Type;

   function Get_Message_Component_Id
     (Incoming : Incoming_Data) return Component_Id_Type;

   function Get_Message_Link_Id
     (Incoming : Incoming_Data) return Link_Id_Type;

   procedure Check_Message_Signature
     (Incoming  : Incoming_Data;
      Sign      : Signature;
      Link_Id   : out Link_Id_Type;
      Timestamp : out Timestamp_Type;
      Signature : out Three_Boolean);

   function Is_CRC_Valid
     (Incoming : Incoming_Data;
      Extras   : Interfaces.Unsigned_8)
      return Boolean;

   procedure Get_Buffer
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer;
      Last     : out Natural);

   procedure Clear (Incoming : in out Incoming_Data);

   procedure Get_Message_Data
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer;
      Last     : out Natural);

   procedure Encode_V2
     (System_Id     : System_Id_Type;
      Component_Id  : Component_Id_Type;
      Sequence_Id   : Sequence_Id_Type;
      Id            : MAVLink.V2.Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Inc_Flags     : Interfaces.Unsigned_8 := 0;
      Target_System : Target_Id_Type := 0);

   procedure Encode_V2
     (System_Id     : System_Id_Type;
      Component_Id  : Component_Id_Type;
      Sequence_Id   : Sequence_Id_Type;
      Id            : MAVLink.V2.Msg_Id;
      Extras        : Interfaces.Unsigned_8;
      Sign          : in out Signature;
      Data          : Data_Buffer;
      Buffer        : out Data_Buffer;
      Last          : out Positive;
      Target_System : Target_Id_Type := 0);

   procedure Calc_SHA
     (Key    : SHA_256.Context;
      Buffer : Data_Buffer;
      Result : out SHA_Digest);

   --  V1 support --

   function Get_Message_Id (Incoming : Incoming_Data) return MAVLink.V1.Msg_Id
   is (Incoming.Income_Buffer (Pos_Msg_Id));

   function Get_Message_Id (Self : Connection) return MAVLink.V1.Msg_Id
   is (Get_Message_Id (Self.Incoming));

   function Get_Message_Id (Self : In_Connection) return MAVLink.V1.Msg_Id
   is (Get_Message_Id (Self.Incoming));

   procedure Encode_V1
     (System_Id    : Short_System_Id_Type;
      Component_Id : Component_Id_Type;
      Sequence_Id  : Sequence_Id_Type;
      Id           : MAVLink.V1.Msg_Id;
      Extras       : Interfaces.Unsigned_8;
      Buffer       : in out Data_Buffer;
      Last         : in out Positive);

   procedure Get_Message_Data
     (Incoming : Incoming_Data;
      Buffer   : out Data_Buffer);

   Inc_Flags_Signed      : constant := 2#0000_0001#;
   --  Has 13 bytes of signature

   Inc_Flags_System32    : constant := 2#0000_0010#;
   --  3 additional bytes for 32 bit Sys_Id field

   Inc_Flags_Target32    : constant := 2#0000_0100#;
   --  4 additional bytes for Target_Id followed by the header

   Inc_Flags_Unsupported : constant := 2#1111_1000#;
   --  All other bits are unsupported

end MAVLink.Both;
