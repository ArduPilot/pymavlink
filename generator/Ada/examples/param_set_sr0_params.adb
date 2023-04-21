-- Connects to ardupilot (baud rate 115200) via ttyUSB0 and set SR0_PARAMS to 11.
-- Copyright Fil Andrii root.fi36@gmail.com 2022

with Ada.Streams;
with GNAT.Serial_Communications;
with Ada.Text_IO;
with Interfaces;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Strings.Maps;

with Mavlink;
with Mavlink.Connection;
with Mavlink.Messages;
with Mavlink.Types;

procedure Param_Set_SR0_PARAMS is
   use type Ada.Streams.Stream_Element_Offset;
   use type Interfaces.Unsigned_8;
   package Short_Float_Text_IO is new Ada.Text_IO.Float_IO(Short_Float);

   Ser : GNAT.Serial_Communications.Serial_Port;
   Input : Ada.Streams.Stream_Element_Array(1..1024);
   Input_Last : Ada.Streams.Stream_Element_Offset;
   Output : Ada.Streams.Stream_Element_Array(1..1024);
   Output_Last : Ada.Streams.Stream_Element_Offset := Output'First;

   Mav_Conn : Mavlink.Connection.Connection (System_Id => 250);

   function Handler_Param_Value return Boolean is
      Param_Value : Mavlink.Messages.Param_Value;
   begin
      Mav_Conn.Unpack (Param_Value);
      if Ada.Strings.Fixed.Trim (Source => Param_Value.Param_Id,
                                 Left => Ada.Strings.Maps.Null_Set,
                                 Right => Ada.Strings.Maps.To_Set (ASCII.Nul)) = "SR0_PARAMS" then
         Ada.Text_IO.Put (Param_Value.Param_Id & " = ");
         Short_Float_Text_IO.Put (Param_Value.Param_Value, Aft => 4, Exp => 0);
         Ada.Text_IO.New_Line;

         return True;
      end if;
      return False;
   end Handler_Param_Value;

   Param_Set : Mavlink.Messages.Param_Set;

begin
   Ada.Text_IO.Put_Line ("Connects to ardupilot (baud rate 115200) via ttyUSB0 and set SR0_PARAMS to 11.");
   Ada.Text_IO.Put_Line ("Warning this app change param SR0_PARAMS to 11!");

   GNAT.Serial_Communications.Open (Port => Ser, Name => "/dev/ttyUSB0");
   GNAT.Serial_Communications.Set (Port => Ser, Rate => GNAT.Serial_Communications.B115200, Block => True, Timeout => 0.0);

   Param_Set.Target_System := 1;
   Param_Set.Target_Component := 0;
   Param_Set.Param_Id := Ada.Strings.Fixed.Head(Source => "SR0_PARAMS", Count => 16, Pad => ASCII.Nul);
   Param_Set.Param_Value := 11.0;
   Param_Set.Param_Type := Mavlink.Types.Mav_Param_Type_Real32;

   for B of Mav_Conn.Pack (Param_Set) loop
      Output (Output_Last) := Ada.Streams.Stream_Element (B);
      Output_Last := Output_Last + 1;
   end loop;
   GNAT.Serial_Communications.Write (Port => Ser, Buffer => Output (Output'First .. Output_Last));

   Main_Loop: loop
      GNAT.Serial_Communications.Read (Port => Ser, Buffer => Input, Last => Input_Last);
      for B of Input (Input'First .. Input_Last) loop
         if Mav_Conn.Parse_Byte(Interfaces.Unsigned_8(B)) then
            if Mav_Conn.Get_Msg_Id = Mavlink.Messages.Param_Value_Id then
               if Handler_Param_Value then
                  exit Main_Loop;
               end if;
            end if;
         end if;
      end loop;
   end loop Main_Loop;
end Param_Set_SR0_PARAMS;