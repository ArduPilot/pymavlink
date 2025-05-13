--  MAVLink connection
--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

with Interfaces;
with MAVLink.Messages;
with X25CRC;

package MAVLink.Connection is

   pragma Pure (MAVLink.Connection);

   type Connection (System_Id : Interfaces.Unsigned_8) is tagged private;

   function Parse_Byte
     (Conn : in out Connection; Val : Interfaces.Unsigned_8) return Boolean;

   function Get_Target_System_Id
     (Conn : Connection) return Interfaces.Unsigned_8;

   function Get_Target_Component_Id
     (Conn : Connection) return Interfaces.Unsigned_8;

   function Get_Msg_Id (Conn : Connection) return Msg_Id;

   function Pack
     (Conn : in out Connection; Msg : MAVLink.Messages.Message'Class)
      return Byte_Array;

   procedure Unpack
     (Conn : in out Connection; Msg : in out MAVLink.Messages.Message'Class);

private

   Pos_Target_System_Id    : constant Natural := 3;
   Pos_Target_Component_Id : constant Natural := 4;
   Pos_Msg_Id              : constant Natural := 5;

   type Connection (System_Id : Interfaces.Unsigned_8) is tagged record
      Component_Id : Interfaces.Unsigned_8 := 1;
      In_Buf       : Byte_Array (0 .. 255 + 8) := (others => 0);
      In_Ptr       : Natural := 0;
      Len          : Natural := 0;
      Out_Sequency : Interfaces.Unsigned_8 := 0;
      Checksum     : X25CRC.Checksum;
   end record;

   function Get_Target_System_Id
     (Conn : Connection) return Interfaces.Unsigned_8
   is (Conn.In_Buf (Pos_Target_System_Id));

   function Get_Target_Component_Id
     (Conn : Connection) return Interfaces.Unsigned_8
   is (Conn.In_Buf (Pos_Target_Component_Id));

   function Get_Msg_Id (Conn : Connection) return Msg_Id
   is (Conn.In_Buf (Pos_Msg_id));

end MAVLink.Connection;
