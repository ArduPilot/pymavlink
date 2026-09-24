with Ada.Command_Line;
with Ada.Streams.Stream_IO;
with Interfaces;
with MAVLink.@VERSION@;
procedure Reject_Ada is
   package P renames MAVLink.@VERSION@;
   package IO renames Ada.Streams.Stream_IO;
   C : P.In_Connection;
   F : IO.File_Type;
   Data : Ada.Streams.Stream_Element_Array (1 .. 1);
   Last : Ada.Streams.Stream_Element_Offset;
   Count : Natural := 0;
begin
   IO.Open (F, IO.In_File, Ada.Command_Line.Argument (1));
   while not IO.End_Of_File (F) loop
      IO.Read (F, Data, Last);
      if P.Parse_Byte (C, Interfaces.Unsigned_8 (Data (1))) then
         pragma Assert (Integer (P.@MESSAGE_ID@ (C)) = 0);
         pragma Assert (Integer (P.@SYSTEM_ID@ (C)) = 42);
         Count := Count + 1;
      end if;
   end loop;
   IO.Close (F);
   pragma Assert (Count = 1);
end Reject_Ada;
