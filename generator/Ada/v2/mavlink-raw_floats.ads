
pragma Ada_2022;

with Ada.Unchecked_Conversion;
with Interfaces;

package Mavlink.Raw_Floats is

   pragma Preelaborate;

   type Raw_Float is private;

   function Is_Valid (Value : Raw_Float) return Boolean;

   function To_Float (Value : Raw_Float) return Float
     with Pre => Is_Valid (Value);

   function To_Raw (Value : Float) return Raw_Float;

private
   type Raw_Float is new Interfaces.Unsigned_32;

   function Unchecked_To_Float is new Ada.Unchecked_Conversion
     (Raw_Float, Float);

   function Unchecked_To_Raw is new Ada.Unchecked_Conversion
     (Float, Raw_Float);

   function Is_Valid (Value : Raw_Float) return Boolean is
     (Unchecked_To_Float (Value)'Valid);

   function To_Float (Value : Raw_Float) return Float is
     (Unchecked_To_Float (Value));

   function To_Raw (Value : Float) return Raw_Float is
      (Unchecked_To_Raw (Value));

end Mavlink.Raw_Floats;
