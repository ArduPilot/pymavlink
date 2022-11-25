-- Connects to ardupilot (baud rate 115200) via ttyUSB0 and read all params
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

procedure Param_Request_List is
   use type Ada.Streams.Stream_Element_Offset;
   use type Interfaces.Unsigned_8;
   package Short_Float_Text_IO is new Ada.Text_IO.Float_IO(Short_Float);

   Ser : GNAT.Serial_Communications.Serial_Port;
   Input : Ada.Streams.Stream_Element_Array(1..1024);
   Input_Last : Ada.Streams.Stream_Element_Offset;
   Output : Ada.Streams.Stream_Element_Array(1..1024);
   Output_Last : Ada.Streams.Stream_Element_Offset := Output'First;

   Mav_Conn : Mavlink.Connection.Connection (System_Id => 250);

   procedure Handler_Param_Value is
      Param_Value : Mavlink.Messages.Param_Value;
   begin
      Mav_Conn.Unpack (Param_Value);
      Ada.Text_IO.Put (Param_Value.Param_Id & " = ");
      Short_Float_Text_IO.Put (Param_Value.Param_Value, Aft => 4, Exp => 0);
      Ada.Text_IO.New_Line;
   end Handler_Param_Value;

   Param_Request_List : Mavlink.Messages.Param_Request_List;

begin
   Ada.Text_IO.Put_Line ("Connects to ardupilot (baud rate 115200) via ttyUSB0 and read all params");

   GNAT.Serial_Communications.Open (Port => Ser, Name => "/dev/ttyUSB0");
   GNAT.Serial_Communications.Set (Port => Ser, Rate => GNAT.Serial_Communications.B115200, Block => True, Timeout => 0.0);

   Param_Request_List.Target_System := 1;
   Param_Request_List.Target_Component := 0;

   for B of Mav_Conn.Pack (Param_Request_List) loop
      Output (Output_Last) := Ada.Streams.Stream_Element (B);
      Output_Last := Output_Last + 1;
   end loop;
   GNAT.Serial_Communications.Write (Port => Ser, Buffer => Output (Output'First .. Output_Last));

   loop
      GNAT.Serial_Communications.Read (Port => Ser, Buffer => Input, Last => Input_Last);
      for B of Input (Input'First .. Input_Last) loop
         if Mav_Conn.Parse_Byte(Interfaces.Unsigned_8(B)) then
            if Mav_Conn.Get_Msg_Id = Mavlink.Messages.Param_Value_Id then
               Handler_Param_Value;
            end if;
         end if;
      end loop;
   end loop;
end Param_Request_List;