-- Connects to ardupilot (baud rate 115200) via ttyUSB0 and read all params
-- Copyright Fil Andrii root.fi36@gmail.com 2022s

with Ada.Streams;
with GNAT.Serial_Communications;
with Ada.Text_IO;
with Interfaces;
with Ada.Numerics.Generic_Elementary_Functions;

with Mavlink;
with Mavlink.Connection;
with Mavlink.Messages;
with Mavlink.Types;

procedure Attitude is
   use type Ada.Streams.Stream_Element_Offset;
   use type Interfaces.Unsigned_8;
   package Short_Float_Text_IO is new Ada.Text_IO.Float_IO(Short_Float);

   Ser : GNAT.Serial_Communications.Serial_Port;
   Input : Ada.Streams.Stream_Element_Array(1..1024);
   Input_Last : Ada.Streams.Stream_Element_Offset;

   Mav_Conn : Mavlink.Connection.Connection (System_Id => 250);

   procedure Handler_Attitude is
      Attitude : Mavlink.Messages.Attitude;
      K_Rad2Deg : Short_Float := 180.0 / Ada.Numerics.Pi;
   begin
      Mav_Conn.Unpack (Attitude);

      Ada.Text_IO.Put ("Pitch: ");
      Short_Float_Text_IO.Put(Attitude.Pitch * K_Rad2Deg, Aft => 4, Exp => 0);
      Ada.Text_IO.Put ("   Roll: ");
      Short_Float_Text_IO.Put(Attitude.Roll * K_Rad2Deg, Aft => 4, Exp => 0);
      Ada.Text_IO.Put ("   Yaw: ");
      Short_Float_Text_IO.Put(Attitude.Yaw * K_Rad2Deg, Aft => 4, Exp => 0);
      Ada.Text_IO.New_Line;
   end Handler_Attitude;

begin
   Ada.Text_IO.Put_Line ("Connects to ardupilot (baud rate 115200) via ttyUSB0 and reads attitude angles");

   GNAT.Serial_Communications.Open (Port => Ser, Name => "/dev/ttyUSB0");
   GNAT.Serial_Communications.Set (Port => Ser, Rate => GNAT.Serial_Communications.B115200, Block => True, Timeout => 0.0);

   loop
      GNAT.Serial_Communications.Read (Port => Ser, Buffer => Input, Last => Input_Last);
      for B of Input (Input'First .. Input_Last) loop
         if Mav_Conn.Parse_Byte(Interfaces.Unsigned_8(B)) then
            if Mav_Conn.Get_Msg_Id = Mavlink.Messages.Attitude_Id then
               Handler_Attitude;
            end if;
         end if;
      end loop;
   end loop;
end Attitude;