#!/usr/bin/env python3
"""
Module to test MAVXML
"""

import unittest
from importlib.resources import files as importlib_files

import contextlib
import importlib.util
import io
import os
import tempfile

from pymavlink.generator import mavgen
from pymavlink.generator.mavparse import MAVXML
from pymavlink.generator.mavparse import MAVParseError
from pymavlink.generator.mavparse import merge_enums

LOCATION_XML = '''<?xml version="1.0"?>
<mavlink>
  <version>3</version>
  <dialect>0</dialect>
  <enums>
    <enum name="MAV_CMD">
      <entry value="16" name="MAV_CMD_NAV_WAYPOINT" hasLocation="true" isDestination="true">
        <description>destination</description>
      </entry>
      <entry value="189" name="MAV_CMD_DO_LAND_START" hasLocation="true" isDestination="%s">
        <description>location, not a destination</description>
      </entry>
      <entry value="20" name="MAV_CMD_NAV_RETURN_TO_LAUNCH">
        <description>no location</description>
      </entry>
    </enum>
  </enums>
  <messages>
    <message id="0" name="HEARTBEAT">
      <description>heartbeat</description>
      <field type="uint8_t" name="type">type</field>
    </message>
  </messages>
</mavlink>
'''

class MAVXMLTest(unittest.TestCase):
    """
    Class to test MAVXML
    """

    def test_fields_number(self):
        """Test that a message can have at most 64 fields"""
        test_filename = "64-fields.xml"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        xml = MAVXML(test_filepath)
        count = len(xml.message[0].fields)
        self.assertEqual(count, 64)

        test_filename = "65-fields.xml"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        with self.assertRaises(MAVParseError):
            _ = MAVXML(test_filepath)


    def write_location_xml(self, directory, is_destination="false"):
        path = os.path.join(directory, "location.xml")
        with open(path, "w", encoding="utf-8") as f:
            f.write(LOCATION_XML % is_destination)
        return path

    def test_location_attributes(self):
        """Test that hasLocation and isDestination are parsed"""
        with tempfile.TemporaryDirectory() as directory:
            xml = MAVXML(self.write_location_xml(directory))
        entries = {e.name: e for e in xml.enum[0].entry}
        self.assertTrue(entries["MAV_CMD_NAV_WAYPOINT"].has_location)
        self.assertTrue(entries["MAV_CMD_NAV_WAYPOINT"].is_destination)
        self.assertTrue(entries["MAV_CMD_DO_LAND_START"].has_location)
        self.assertFalse(entries["MAV_CMD_DO_LAND_START"].is_destination)
        self.assertFalse(entries["MAV_CMD_NAV_RETURN_TO_LAUNCH"].has_location)
        self.assertFalse(entries["MAV_CMD_NAV_RETURN_TO_LAUNCH"].is_destination)

    def test_invalid_is_destination(self):
        """Test that an invalid isDestination value raises an exception"""
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_location_xml(directory, is_destination="maybe")
            with self.assertRaises(MAVParseError):
                _ = MAVXML(path)

    def test_python_location_attributes(self):
        """Test that generated Python carries has_location and is_destination"""
        with tempfile.TemporaryDirectory() as directory:
            xml_path = self.write_location_xml(directory)
            output = os.path.join(directory, "location.py")
            opts = mavgen.Opts(output, wire_protocol="2.0", language="Python3", validate=False)
            self.assertTrue(mavgen.mavgen(opts, [xml_path]))
            spec = importlib.util.spec_from_file_location("location", output)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        mav_cmd = module.enums["MAV_CMD"]
        self.assertTrue(mav_cmd[16].has_location)
        self.assertTrue(mav_cmd[16].is_destination)
        self.assertTrue(mav_cmd[189].has_location)
        self.assertFalse(mav_cmd[189].is_destination)
        self.assertFalse(mav_cmd[20].has_location)
        self.assertFalse(mav_cmd[20].is_destination)

    def test_isdestination_without_haslocation(self):
        """Test that an enum entry which is isDestination but not hasLocation
        is parsed with a warning, as older definitions mark some that way"""
        test_filename = "isdestination-without-haslocation.xml"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            xml = MAVXML(test_filepath)
        entry = xml.enum[0].entry[0]
        self.assertFalse(entry.has_location)
        self.assertTrue(entry.is_destination)
        self.assertIn("MAV_CMD_TEST is isDestination but not hasLocation", output.getvalue())

    def test_wire_protocol_version(self):
        """Test that an unknown MAVLink wire protocol version raises an exception"""
        with self.assertRaises(MAVParseError):
            _ = MAVXML(filename="", wire_protocol_version=42)


    def test_wip_deprecated_superseded(self):
        """Test that wip/deprecated/superseded are parsed for messages, enums and enum entries"""
        test_filename = "wip_deprecated_superseded.xml"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        xml = MAVXML(test_filepath)

        messages = {m.name: m for m in xml.message}
        self.assertTrue(messages["WIP_MESSAGE"].wip)
        self.assertIsNone(messages["WIP_MESSAGE"].deprecated)
        self.assertIsNone(messages["WIP_MESSAGE"].superseded)

        self.assertFalse(messages["DEPRECATED_MESSAGE"].wip)
        self.assertIsNotNone(messages["DEPRECATED_MESSAGE"].deprecated)
        self.assertEqual(messages["DEPRECATED_MESSAGE"].deprecated.since, "2024-01")
        self.assertEqual(messages["DEPRECATED_MESSAGE"].deprecated.replaced_by, "WIP_MESSAGE")

        self.assertFalse(messages["SUPERSEDED_MESSAGE"].wip)
        self.assertIsNotNone(messages["SUPERSEDED_MESSAGE"].superseded)
        self.assertEqual(messages["SUPERSEDED_MESSAGE"].superseded.since, "2024-01")
        self.assertEqual(messages["SUPERSEDED_MESSAGE"].superseded.replaced_by, "WIP_MESSAGE")

        enums = {e.name: e for e in xml.enum}
        self.assertTrue(enums["WIP_ENUM"].wip)
        self.assertIsNone(enums["WIP_ENUM"].deprecated)
        self.assertIsNone(enums["WIP_ENUM"].superseded)
        self.assertFalse(enums["DEPRECATED_ENUM"].wip)
        self.assertFalse(enums["SUPERSEDED_ENUM"].wip)
        self.assertFalse(enums["ENTRY_FLAGS_ENUM"].wip)
        self.assertIsNone(enums["ENTRY_FLAGS_ENUM"].deprecated)
        self.assertIsNone(enums["ENTRY_FLAGS_ENUM"].superseded)
        # an enum-level flag is not copied onto the parsed entries; the
        # generators apply it to unflagged entries themselves
        self.assertFalse(enums["WIP_ENUM"].entry[0].wip)

        self.assertIsNotNone(enums["DEPRECATED_ENUM"].deprecated)
        self.assertEqual(enums["DEPRECATED_ENUM"].deprecated.replaced_by, "SOME_OTHER_ENUM")
        self.assertIsNotNone(enums["SUPERSEDED_ENUM"].superseded)
        self.assertEqual(enums["SUPERSEDED_ENUM"].superseded.replaced_by, "SOME_OTHER_ENUM")

        entries = {e.name: e for e in enums["ENTRY_FLAGS_ENUM"].entry}
        self.assertTrue(entries["ENTRY_FLAGS_ENUM_WIP"].wip)
        self.assertIsNone(entries["ENTRY_FLAGS_ENUM_WIP"].deprecated)
        self.assertIsNone(entries["ENTRY_FLAGS_ENUM_WIP"].superseded)

        self.assertIsNotNone(entries["ENTRY_FLAGS_ENUM_DEPRECATED"].deprecated)
        self.assertEqual(entries["ENTRY_FLAGS_ENUM_DEPRECATED"].deprecated.replaced_by, "ENTRY_FLAGS_ENUM_WIP")

        self.assertIsNotNone(entries["ENTRY_FLAGS_ENUM_SUPERSEDED"].superseded)
        self.assertEqual(entries["ENTRY_FLAGS_ENUM_SUPERSEDED"].superseded.replaced_by, "ENTRY_FLAGS_ENUM_WIP")

        self.assertFalse(entries["ENTRY_FLAGS_ENUM_PLAIN"].wip)
        self.assertIsNone(entries["ENTRY_FLAGS_ENUM_PLAIN"].deprecated)
        self.assertIsNone(entries["ENTRY_FLAGS_ENUM_PLAIN"].superseded)

        # entries record the flags of the enum declaration they came from
        self.assertTrue(enums["WIP_ENUM"].entry[0].enum_wip)
        self.assertIsNotNone(enums["DEPRECATED_ENUM"].entry[0].enum_deprecated)
        self.assertIsNotNone(enums["SUPERSEDED_ENUM"].entry[0].enum_superseded)
        self.assertFalse(entries["ENTRY_FLAGS_ENUM_WIP"].enum_wip)
        self.assertIsNone(entries["ENTRY_FLAGS_ENUM_DEPRECATED"].enum_deprecated)
        self.assertIsNone(entries["ENTRY_FLAGS_ENUM_SUPERSEDED"].enum_superseded)


    def test_extended_enum_flags(self):
        """Test that merging an extended enum keeps each declaration's flags with its own entries"""
        dialect = MAVXML(importlib_files(__spec__.parent).joinpath("wip_deprecated_superseded.xml"))
        base = MAVXML(importlib_files(__spec__.parent).joinpath("wip_deprecated_superseded_base.xml"))
        # mavgen lists the including file before the files it includes
        merge_enums([dialect, base])

        enums = {e.name: e for e in dialect.enum}
        self.assertNotIn("EXTENDED_DEPRECATED_ENUM", [e.name for e in base.enum])
        self.assertNotIn("EXTENDED_WIP_ENUM", [e.name for e in base.enum])

        # flagged in the included file only: the merged enum and the
        # included file's entry are flagged, the extension's entry is not
        merged = enums["EXTENDED_DEPRECATED_ENUM"]
        self.assertIsNotNone(merged.deprecated)
        self.assertEqual(merged.deprecated.replaced_by, "SOME_OTHER_ENUM")
        entries = {e.name: e for e in merged.entry}
        self.assertIsNotNone(entries["EXTENDED_DEPRECATED_ENUM_FROM_FLAGGED_BASE"].enum_deprecated)
        self.assertIsNone(entries["EXTENDED_DEPRECATED_ENUM_FROM_UNFLAGGED_DIALECT"].enum_deprecated)
        self.assertIsNone(entries["EXTENDED_DEPRECATED_ENUM_ENUM_END"].enum_deprecated)

        # flagged in the including file only: the merged enum and the
        # including file's entry are flagged, the included file's entry is not
        merged = enums["EXTENDED_WIP_ENUM"]
        self.assertTrue(merged.wip)
        entries = {e.name: e for e in merged.entry}
        self.assertFalse(entries["EXTENDED_WIP_ENUM_FROM_UNFLAGGED_BASE"].enum_wip)
        self.assertTrue(entries["EXTENDED_WIP_ENUM_FROM_FLAGGED_DIALECT"].enum_wip)
        self.assertFalse(entries["EXTENDED_WIP_ENUM_ENUM_END"].enum_wip)

        # flagged in both files: the merged enum and both entries are flagged
        merged = enums["EXTENDED_BOTH_DEPRECATED_ENUM"]
        self.assertIsNotNone(merged.deprecated)
        entries = {e.name: e for e in merged.entry}
        self.assertIsNotNone(entries["EXTENDED_BOTH_DEPRECATED_ENUM_FROM_BASE"].enum_deprecated)
        self.assertIsNotNone(entries["EXTENDED_BOTH_DEPRECATED_ENUM_FROM_DIALECT"].enum_deprecated)


if __name__ == '__main__':
    unittest.main()
