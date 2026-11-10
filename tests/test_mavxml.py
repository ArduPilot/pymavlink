#!/usr/bin/env python3
"""
Module to test MAVXML
"""

import unittest
from importlib.resources import files as importlib_files

import importlib.util
import os
import tempfile

from pymavlink.generator import mavgen
from pymavlink.generator.mavparse import MAVXML
from pymavlink.generator.mavparse import MAVParseError

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

    def test_wire_protocol_version(self):
        """Test that an unknown MAVLink wire protocol version raises an exception"""
        with self.assertRaises(MAVParseError):
            _ = MAVXML(filename="", wire_protocol_version=42)


if __name__ == '__main__':
    unittest.main()
