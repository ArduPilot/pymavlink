#!/usr/bin/env python3
"""
Module to test MAVXML
"""

import os
import tempfile
import unittest
try:
    from importlib.resources import files as importlib_files
except ImportError:
    # importlib.resources.files() requires Python 3.9+; use backport for older versions
    from importlib_resources import files as importlib_files

from pymavlink.generator.mavparse import MAVXML
from pymavlink.generator.mavparse import MAVParseError
from pymavlink.generator.mavparse import check_duplicates

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


    def test_wire_protocol_version(self):
        """Test that an unknown MAVLink wire protocol version raises an exception"""
        with self.assertRaises(MAVParseError):
            _ = MAVXML(filename="", wire_protocol_version=42)


def enums_xml(enums, first_value=0):
    """Write a dialect holding only the given enums and parse it"""
    body = "".join(
        '<enum name="%s">%s</enum>' % (name, "".join(
            '<entry value="%u" name="%s"/>' % (value, entry)
            for value, entry in enumerate(entries, first_value)))
        for name, entries in enums)
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
        f.write("<mavlink><enums>%s</enums></mavlink>" % body)
    try:
        return MAVXML(f.name)
    finally:
        os.remove(f.name)


class DuplicateEnumEntryNameTest(unittest.TestCase):
    """
    Enum entry names share one namespace in the generated code, so the
    same name must not appear in two different enums
    """

    def test_distinct_names_pass(self):
        xml = enums_xml([("ENUM_A", ["A_ONE", "A_TWO"]), ("ENUM_B", ["B_ONE"])])
        self.assertFalse(check_duplicates([xml]))

    def test_same_name_in_two_enums_fails(self):
        xml = enums_xml([("ENUM_A", ["SAME"]), ("ENUM_B", ["B_ONE", "SAME"])])
        self.assertTrue(check_duplicates([xml]))

    def test_same_name_in_two_enums_in_different_files_fails(self):
        child = enums_xml([("ENUM_B", ["SAME"])])
        parent = enums_xml([("ENUM_A", ["SAME"])])
        self.assertTrue(check_duplicates([child, parent]))

    def test_enum_extended_in_another_file_passes(self):
        # A dialect adding entries to an enum from an included file (as
        # ardupilotmega.xml does with MAV_CMD) is merged into one enum.
        child = enums_xml([("ENUM_A", ["A_CHILD"])], first_value=10)
        parent = enums_xml([("ENUM_A", ["A_ONE", "A_TWO"])])
        self.assertFalse(check_duplicates([child, parent]))


if __name__ == '__main__':
    unittest.main()
