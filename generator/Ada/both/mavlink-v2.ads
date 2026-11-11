
pragma Ada_2022;

with Interfaces;              use Interfaces;
with MAVLink.Raw_Floats;      use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats; use MAVLink.Raw_Long_Floats;

package MAVLink.V2 is

   pragma Pure;

   Version_2_Code     : constant Interfaces.Unsigned_8 := 16#FD#;

   type Msg_Id is mod 2 ** 24 with Size => 24;
   --  Message ID has 3 bytes

   Maximum_Buffer_Len : constant Positive := 287;

   --  Positions of the messages parts minus 1, to use in construction like
   --  Buff'First + Packet_Payload_First
   Packet_Payload_First : constant Positive := 10;

   type System_Id_Type is new Interfaces.Unsigned_32;
   type Short_System_Id_Type is new Interfaces.Unsigned_8;
   type Component_Id_Type is new Interfaces.Unsigned_8;
   type Sequence_Id_Type is new Interfaces.Unsigned_8;
   type Link_Id_Type is new Interfaces.Unsigned_8;
   type Target_Id_Type is new Interfaces.Unsigned_32;

   type Timestamp_Type is mod 2 ** 48 with Size => 48;

   subtype Signature_Index is Positive range 1 .. 32;
   type Signature_Key is array (Signature_Index range <>) of
     Interfaces.Unsigned_8;

   type Short_Float_Array is array (Natural range <>) of Raw_Float
     with Component_Size => 32;
   type Long_Float_Array is array (Natural range <>) of Raw_Long_Float
     with Component_Size => 64;

end MAVLink.V2;
