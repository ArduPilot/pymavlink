--  MAVLink
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

pragma Ada_2022;

with Interfaces;              use Interfaces;
with MAVLink.Raw_Floats;      use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats; use MAVLink.Raw_Long_Floats;

package MAVLink.V1 is

   pragma Pure;

   Version_1_Code       : constant Interfaces.Unsigned_8 := 16#FE#;

   Packet_Marker_Length : constant := 1;
   Packet_Header_Length : constant := 5;

   --  Positions of the messages parts minus 1, to use in construction like
   --  Buff'First + Packet_Payload_First
   Packet_Payload_First : constant :=
     Packet_Header_Length + Packet_Marker_Length;

   subtype Msg_Id is Interfaces.Unsigned_8;

   type Short_Float_Array is array (Natural range <>) of Raw_Float
     with Component_Size => 32;
   type Long_Float_Array is array (Natural range <>) of Raw_Long_Float
     with Component_Size => 64;

end MAVLink.V1;
