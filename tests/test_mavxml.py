#!/usr/bin/env python3
"""
Module to test MAVXML
"""

import unittest
try:
    from importlib.resources import files as importlib_files
except ImportError:
    # importlib.resources.files() requires Python 3.9+; use backport for older versions
    from importlib_resources import files as importlib_files

from pymavlink.generator.mavparse import MAVXML
from pymavlink.generator.mavparse import MAVParseError

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


if __name__ == '__main__':
    unittest.main()
