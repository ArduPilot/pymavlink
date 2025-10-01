--
-- Connects to ardupilot via UDP (IP = 127.0.0.1, Port = 14551) and reads the GPS Position
--
-- Author : Saiffullah Sabir Mohamed
-- Email  : saif25596@outlook.com
-- Github : https://github.com/TechnicalVillager
--

with Ada.Streams;
with Ada.Text_IO;
with Ada.Float_Text_IO;
with Interfaces;
with Ada.Numerics.Generic_Elementary_Functions;
with GNAT.Sockets; use GNAT.Sockets;
with Ada.Unchecked_Conversion;

with Mavlink;
with Mavlink.Connection;
with Mavlink.Messages;
with Mavlink.Types;

procedure GPS_Position is
   use type Ada.Streams.Stream_Element_Offset;
   use type Interfaces.Unsigned_8, Interfaces.Integer_32;

   Server        : Socket_Type;
   Address, From : Sock_Addr_Type;
   Input         : Ada.Streams.Stream_Element_Array(1..1024);
   Input_Last    : Ada.Streams.Stream_Element_Offset;
   Mav_Conn      : Mavlink.Connection.Connection (System_Id => 1);

   procedure Handler_Position is

      type Float_Digits_15 is new Interfaces.IEEE_Float_64;

      package Float_Digits_15_IO is new
         Ada.Text_IO.Float_IO (Float_Digits_15);

      type Geo_Location is record
         Latitude  : Float_Digits_15;
         Longitude : Float_Digits_15;
         Altitude  : Float;
      end record;

      Position : Mavlink.Messages.Global_Position_Int;
      Location : Geo_Location;
   begin
      Mav_Conn.Unpack (Position);

      Location := (Latitude  => Float_Digits_15 (Position.Lat) / 1.0E7,
                   Longitude => Float_Digits_15 (Position.Lon) / 1.0E7,
                   Altitude  => Float (Position.Relative_Alt) / 1.0E3);

      Ada.Text_IO.Put ("Latitude = ");
      Float_Digits_15_IO.Put (Item => Location.Latitude, Exp => 0);

      Ada.Text_IO.Put ("    Longitude = ");
      Float_Digits_15_IO.Put (Item => Location.Longitude, Exp => 0);

      Ada.Text_IO.Put ("    Altitude = ");
      Ada.Float_Text_IO.Put (Item => Location.Altitude, Exp => 0);
      Ada.Text_IO.New_Line;

   end Handler_Position;

begin
   Create_Socket (Server, Family_Inet, Socket_Datagram);
   Set_Socket_Option
     (Server,
      Socket_Level,
      (Reuse_Address, True));
   
   Address.Addr := Inet_Addr("127.0.0.1");
   Address.Port := 14551;

   Ada.Text_IO.Put_Line ("Connects to ardupilot via UDP (IP = 127.0.0.1, Port = 14551) and reads the GPS Position");

   Bind_Socket (Server, Address);

   loop
      Receive_Socket (Server, Input, Input_Last, From);
      for B of Input (Input'First .. Input_Last) loop
         if Mav_Conn.Parse_Byte(Interfaces.Unsigned_8(B)) then
            if Mav_Conn.Get_Msg_Id = Mavlink.Messages.Global_Position_Int_Id then
               Handler_Position;
            end if;
         end if;
      end loop;
   end loop;
end GPS_Position;