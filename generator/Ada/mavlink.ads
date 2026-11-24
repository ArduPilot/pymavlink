--  MAVLink
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

pragma Ada_2022;

with Interfaces;

package MAVLink is

   pragma Pure;

   type Data_Buffer is array (Positive range <>) of Interfaces.Unsigned_8;

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

end MAVLink;

