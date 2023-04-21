-- Mavlink
-- Copyright Fil Andrii root.fi36@gmail.com 2022

with Interfaces;

package Mavlink is

   pragma Pure (Mavlink);

   Packet_Marker            : constant Interfaces.Unsigned_8 := 16#FE#;
   Packet_Marker_Length     : constant := 1;
   Packet_Checksum_Length   : constant := 2;
   Packet_Header_Length     : constant := 5;
   Packet_Payload_First     : constant := Packet_Header_Length + Packet_Marker_Length;
   Packet_Control_INfo_Size : constant := Packet_Marker_Length + Packet_Header_Length + Packet_Checksum_Length;

   Tag_Length : constant := Standard'Address_Size / 8;
   
   Message_Alignment : constant := 8;
   Message_Size      : constant := Positive (Float'Ceiling (Float (Tag_Length + Packet_Header_Length) /
                                               Float (Message_Alignment))) * Message_Alignment;

   subtype Msg_Id is Interfaces.Unsigned_8;

   type Byte_Arrray is array (Natural range <>) of Interfaces.Unsigned_8;

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
   type Short_Float_Array is array (Natural range <>) of Short_Float
     with Component_Size => 32;
   type Long_Float_Array is array (Natural range <>) of Long_Float
     with Component_Size => 64;

end Mavlink;

