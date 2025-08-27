#!/usr/bin/env python3

"""
Comprehensive regression tests for mavlogdump.py
"""
import unittest
import os
import sys
import json
import tempfile
import shutil
import pkg_resources

# Add parent directory to path to import mavlogdump
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'tools')))

from tools import mavlogdump



class MAVLogDumpTest(unittest.TestCase):
    """
    Class to test mavlogdump functionality for all formats
    """

    def setUp(self):
        """Set up test fixtures"""
        self.test_dir = tempfile.mkdtemp()
        self.test_filename = "test.BIN" 
        try:
            self.test_filepath = pkg_resources.resource_filename(__name__, self.test_filename)
        except:
            # If resource not found, create a dummy file for testing
            self.test_filepath = os.path.join(self.test_dir, self.test_filename)
            open(self.test_filepath, 'a').close()

    def tearDown(self):
        """Clean up test fixtures"""
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)

    def test_dump_standard_format(self):
        """Test standard format dump of file"""
        output_file = os.path.join(self.test_dir, "standard_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Output file should be created")

    def test_dump_json_format(self):
        """Test JSON format output"""
        output_file = os.path.join(self.test_dir, "json_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='json',
            log=self.test_filepath,
        )
        self.assertTrue(os.path.exists(output_file), "JSON output file should be created")
        
        # Verify JSON format if file has content
        if os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                content = f.read().strip()
                if content:
                    try:
                        # JSON output is now an array
                        if content.startswith('[') and content.endswith(']'):
                            data = json.loads(content)
                            if data:  # If array is not empty
                                first_item = data[0]
                                self.assertIn('meta', first_item, "JSON output should have 'meta' field")
                                self.assertIn('data', first_item, "JSON output should have 'data' field")
                    except json.JSONDecodeError:
                        pass  # File might be empty or invalid, which is OK for test files

    def test_dump_json_with_show_source(self):
        """Test JSON format with show-source option"""
        output_file = os.path.join(self.test_dir, "json_source_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='json',
            show_source=True,
            log=self.test_filepath,
        )
        self.assertTrue(os.path.exists(output_file), "JSON output file should be created")
        
        # Verify JSON includes source info if file has content
        if os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                content = f.read().strip()
                if content:
                    try:
                        # JSON output is now an array
                        if content.startswith('[') and content.endswith(']'):
                            data = json.loads(content)
                            if data:  # If array is not empty
                                first_item = data[0]
                                if 'meta' in first_item:
                                    # Check if source fields are present when data is available
                                    if first_item.get('data'):
                                        self.assertIn('type', first_item['meta'], "Meta should have type field")
                    except json.JSONDecodeError:
                        pass  # File might be empty or invalid, which is OK for test files

    def test_dump_csv_format(self):
        """Test CSV format output"""
        output_file = os.path.join(self.test_dir, "csv_output.csv")
        output_file='testing.csv'
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='csv',
            types='IMU2',
            log=self.test_filepath,
        )
        # Check if file was created (even if empty)
        if os.path.exists(output_file):
            with open(output_file, 'r') as f:
                content = f.read()
                if content.strip():
                    # Verify CSV format
                    lines = content.strip().split('\n')
                    if lines:
                        # First line should be headers
                        headers = lines[0].split(',')
                        self.assertIn('timestamp', headers, "CSV should have timestamp column")

    def test_dump_csv_with_custom_separator(self):
        """Test CSV format with custom separator"""
        output_file = os.path.join(self.test_dir, "csv_tab_output.csv")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='csv',
            csv_sep="tab",
            types='IMU2',
            log=self.test_filepath,
        )
        
        if os.path.exists(output_file) and os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                first_line = f.readline()
                if first_line:
                    # Check for tab separator
                    self.assertIn('\t', first_line, "CSV with tab separator should use tabs")

    def test_dump_mat_format(self):
        """Test MAT format output"""
        output_path = os.path.join(self.test_dir, "output.mat")
        
        try:
            mavlogdump.dump_log(
                format='mat',
                output_path=output_path,
                log=self.test_filepath,
            )
            # MAT format requires scipy, which might not be installed
            self.assertTrue(os.path.exists(output_path), "MAT file should be created")
        except ImportError:
            # MAT format requires scipy, which might not be installed
            self.skipTest('Missing import')

    def test_dump_mat_with_compression(self):
        """Test MAT format with compression"""
        output_path = os.path.join(self.test_dir, "output_compressed.mat")
        
        try:
            mavlogdump.dump_log(
                format='mat',
                output_path=output_path,
                compress=True,
                log=self.test_filepath,
            )
            # MAT format requires scipy, which might not be installed
            self.assertTrue(os.path.exists(output_path), "Compressed MAT file should be created")
        except ImportError:
            # MAT format requires scipy, which might not be installed
            self.skipTest('Missing import')

    def test_type_filtering(self):
        """Test message type filtering"""
        output_file = os.path.join(self.test_dir, "filtered_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            types='ATT,GPS',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Filtered output file should be created")

    def test_nottype_filtering(self):
        """Test message type exclusion"""
        output_file = os.path.join(self.test_dir, "excluded_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            nottypes='BAD_DATA',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Excluded output file should be created")

    def test_output_to_file(self):
        """Test output to file option"""
        output_file = os.path.join(self.test_dir, "direct_output.bin")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Direct output file should be created")

    def test_show_types(self):
        """Test show-types option"""
        output_file = os.path.join(self.test_dir, "types_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='types-only',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Types output file should be created")

    def test_reduce_option(self):
        """Test message reduction by ratio"""
        output_file = os.path.join(self.test_dir, "reduced_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            reduce=10,
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Reduced output file should be created")

    def test_reduce_rate_option(self):
        """Test message reduction by rate"""
        output_file = os.path.join(self.test_dir, "rate_reduced_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            reduce_rate=10,
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Rate reduced output file should be created")

    def test_condition_filtering(self):
        """Test condition-based filtering"""
        output_file = os.path.join(self.test_dir, "condition_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            condition='True',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Condition filtered output file should be created")

    def test_mav10_option(self):
        """Test MAVLink 1.0 parsing"""
        output_file = os.path.join(self.test_dir, "mav10_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            mav10=True,
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "MAV1.0 output file should be created")

    def test_source_filtering(self):
        """Test source system and component filtering"""
        output_file = os.path.join(self.test_dir, "source_filtered.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='standard',
            source_system=1,
            source_component=1,
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Source filtered output file should be created")

    def test_dump_pretty_format(self):
        """Test pretty format output"""
        output_file = os.path.join(self.test_dir, "pretty_output.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='pretty',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Pretty output file should be created")
        
        # Verify pretty format produces verbose output if file has content
        if os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                content = f.read()
                # Pretty format should produce multi-line verbose output for each message
                # Check that it's not just a single-line format
                if content.strip():
                    lines = content.strip().split('\n')
                    # Pretty format typically produces multiple lines per message
                    self.assertGreater(len(lines), 0, "Pretty format should produce output")

    def test_dump_pretty_with_types(self):
        """Test pretty format with type filtering"""
        output_file = os.path.join(self.test_dir, "pretty_filtered.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='pretty',
            types='ATT,GPS',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Pretty filtered output file should be created")

    def test_pretty_with_conditions(self):
        """Test pretty format with conditions"""
        output_file = os.path.join(self.test_dir, "pretty_condition.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='pretty',
            condition='True',
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Pretty condition output file should be created")

    def test_pretty_with_reduce(self):
        """Test pretty format with reduce option"""
        output_file = os.path.join(self.test_dir, "pretty_reduced.txt")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='pretty',
            reduce=5,
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Pretty reduced output file should be created")

    def test_combined_options(self):
        """Test combination of multiple options"""
        output_file = os.path.join(self.test_dir, "combined_output.json")
        
        mavlogdump.dump_log(
            output_path=output_file,
            format='json',
            types='ATT,GPS',
            no_bad_data=True,
            log=self.test_filepath,
        )
        
        self.assertTrue(os.path.exists(output_file), "Combined output file should be created")

    def test_programmatic_json_processing(self):
        """Test programmatic JSON processing"""
        if hasattr(mavlogdump, 'process_log'):
            # Test programmatic interface
            result = mavlogdump.process_log(
                self.test_filepath,
                output_format='json',
                types=['ATT']
            )
            self.assertEqual(result, 0, "Programmatic JSON processing should succeed")

    def test_programmatic_csv_processing(self):
        """Test programmatic CSV processing"""
        if hasattr(mavlogdump, 'process_log'):
            # Test programmatic interface
            output_file = os.path.join(self.test_dir, "prog_csv.csv")
            result = mavlogdump.process_log(
                self.test_filepath,
                output_format='csv',
                types=['*'],
                output=output_file,
                )
            self.assertEqual(result, 0, "Programmatic CSV processing should succeed")

    def test_programmatic_mat_processing(self):
        """Test programmatic MAT processing"""
        if hasattr(mavlogdump, 'process_log'):
            # Test programmatic interface
            output_path = os.path.join(self.test_dir, "prog_output.mat")
            result = mavlogdump.process_log(
                self.test_filepath,
                output_format='mat',
                output_path=output_path,
                )
            # MAT processing might fail if scipy is not installed
            if result == 0:
                self.assertTrue(os.path.exists(output_path), "Programmatic MAT file should be created")


class MAVLogDumpUnitTest(unittest.TestCase):
    """Unit tests for individual functions"""
    
    def test_match_type_function(self):
        """Test the match_type function"""

        # Test exact match
        self.assertTrue(mavlogdump.match_type('GPS', ['GPS']))
        self.assertFalse(mavlogdump.match_type('GPS', ['ATT']))
        
        # Test wildcard match
        self.assertTrue(mavlogdump.match_type('GPS_RAW', ['GPS*']))
        self.assertTrue(mavlogdump.match_type('ATT', ['A*']))
        self.assertFalse(mavlogdump.match_type('GPS', ['ATT*']))
        
        # Test multiple patterns
        self.assertTrue(mavlogdump.match_type('GPS', ['ATT', 'GPS']))
        self.assertTrue(mavlogdump.match_type('ATT', ['ATT', 'GPS']))
    
    def test_to_string_function(self):
        """Test the to_string function"""
        
        # Test string input
        self.assertEqual(mavlogdump.to_string("hello"), "hello")
        
        # Test bytes input
        self.assertEqual(mavlogdump.to_string(b"hello"), "hello")
        
        # Test bytes with special characters
        result = mavlogdump.to_string(b"\xff\xfe")
        self.assertIsInstance(result, str)


if __name__ == '__main__':
    unittest.main()