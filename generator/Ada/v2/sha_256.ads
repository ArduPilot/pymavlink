
with Interfaces; use Interfaces;
with System;

package SHA_256 is

   pragma Preelaborate;

   Hash_Bit_Order : System.Bit_Order := System.Low_Order_First;

   type Context is private;

   Initial_Context : constant Context;

   type Data  is array (Positive range <>) of Interfaces.Unsigned_8;
   type State is array (Natural range <>) of Interfaces.Unsigned_32;

   procedure Update
     (Self  : in out Context;
      Input : Data);

   subtype Digest_Type is Data (1 .. 32);

   procedure Digest
     (Self : in out Context;
      Res  : out Digest_Type);

private

   SHA256_Init_State : constant State (0 .. 7) :=
     [0 => 16#6a09e667#,
      1 => 16#bb67ae85#,
      2 => 16#3c6ef372#,
      3 => 16#a54ff53a#,
      4 => 16#510e527f#,
      5 => 16#9b05688c#,
      6 => 16#1f83d9ab#,
      7 => 16#5be0cd19#];

   type Context is record
      H_State : State (0 .. 7) := SHA256_Init_State;
      Last    : Natural := 0;
      Length  : Interfaces.Unsigned_64 := 0;
      Buffer  : Data (1 .. 64) := (others => 0);
   end record;

   Initial_Context : constant Context := (others => <>);

   procedure Transform (Self : in out Context);

end SHA_256;
