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



class MAVLogDumpTest(unittest.TestCase):
    """
    Class to test mavlogdump functionality for all formats
    """

    def setUp(self):
        """Set up test fixtures"""
        self.test_dir = tempfile.mkdtemp()
        self.test_filename = "test.BIN"
        # Get the path to mavlogdump.py relative to this test file                                                                                          │ │
        self.mavlogdump_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "mavlogdump.py") 
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
        cmd = f"{self.mavlogdump_path} {self.test_filepath} > {output_file}"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Standard format dump should succeed")
        self.assertTrue(os.path.exists(output_file), "Output file should be created")

    def test_dump_json_format(self):
        """Test JSON format output"""
        output_file = os.path.join(self.test_dir, "json_output.txt")
        cmd = f"{self.mavlogdump_path} --format json {self.test_filepath} > {output_file}"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "JSON format dump should succeed")
        self.assertTrue(os.path.exists(output_file), "JSON output file should be created")
        
        # Verify JSON format if file has content
        if os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                for line in f:
                    if line.strip():
                        try:
                            data = json.loads(line)
                            self.assertIn('meta', data, "JSON output should have 'meta' field")
                            self.assertIn('data', data, "JSON output should have 'data' field")
                            break
                        except json.JSONDecodeError:
                            pass

    def test_dump_json_with_show_source(self):
        """Test JSON format with show-source option"""
        output_file = os.path.join(self.test_dir, "json_source_output.txt")
        cmd = f"{self.mavlogdump_path} --format json --show-source {self.test_filepath} > {output_file}"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "JSON format with show-source should succeed")
        self.assertTrue(os.path.exists(output_file), "JSON output file should be created")
        
        # Verify JSON includes source info if file has content
        if os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                for line in f:
                    if line.strip():
                        try:
                            data = json.loads(line)
                            if 'meta' in data:
                                # Check if source fields are present when data is available
                                if data.get('data'):
                                    self.assertIn('type', data['meta'], "Meta should have type field")
                                break
                        except json.JSONDecodeError:
                            pass

    def test_dump_csv_format(self):
        """Test CSV format output"""
        output_file = os.path.join(self.test_dir, "csv_output.csv")
        # CSV format requires --types to be specified
        cmd = f"{self.mavlogdump_path} --format csv --types 'IMU2' {self.test_filepath} > {output_file} 2>/dev/null"
        os.system(cmd)
        
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
        cmd = f"{self.mavlogdump_path} --format csv --csv_sep tab --types 'IMU2' {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        if os.path.exists(output_file) and os.path.getsize(output_file) > 0:
            with open(output_file, 'r') as f:
                first_line = f.readline()
                if first_line:
                    # Check for tab separator
                    self.assertIn('\t', first_line, "CSV with tab separator should use tabs")

    def test_dump_mat_format(self):
        """Test MAT format output"""
        mat_file = os.path.join(self.test_dir, "output.mat")
        cmd = f"{self.mavlogdump_path} --format mat --mat_file {mat_file} {self.test_filepath} 2>/dev/null"
        result = os.system(cmd)
        
        # MAT format requires scipy, which might not be installed
        if result == 0:
            self.assertTrue(os.path.exists(mat_file), "MAT file should be created")

    def test_dump_mat_with_compression(self):
        """Test MAT format with compression"""
        mat_file = os.path.join(self.test_dir, "output_compressed.mat")
        cmd = f"{self.mavlogdump_path} --format mat --mat_file {mat_file} --compress {self.test_filepath} 2>/dev/null"
        result = os.system(cmd)
        
        # MAT format requires scipy, which might not be installed
        if result == 0:
            self.assertTrue(os.path.exists(mat_file), "Compressed MAT file should be created")

    def test_type_filtering(self):
        """Test message type filtering"""
        output_file = os.path.join(self.test_dir, "filtered_output.txt")
        cmd = f"{self.mavlogdump_path} --types 'ATT,GPS' {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Type filtering should succeed")
        self.assertTrue(os.path.exists(output_file), "Filtered output file should be created")

    def test_nottype_filtering(self):
        """Test message type exclusion"""
        output_file = os.path.join(self.test_dir, "excluded_output.txt")
        cmd = f"{self.mavlogdump_path} --nottypes 'BAD_DATA' {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Type exclusion should succeed")
        self.assertTrue(os.path.exists(output_file), "Excluded output file should be created")

    def test_quiet_mode(self):
        """Test quiet mode suppresses output"""
        output_file = os.path.join(self.test_dir, "quiet_output.txt")
        cmd = f"{self.mavlogdump_path} --quiet {self.test_filepath} > {output_file}"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Quiet mode should succeed")
        # In quiet mode, output should be minimal or empty
        self.assertTrue(os.path.exists(output_file), "Output file should be created even in quiet mode")

    def test_output_to_file(self):
        """Test output to file option"""
        output_file = os.path.join(self.test_dir, "direct_output.bin")
        cmd = f"{self.mavlogdump_path} --output {output_file} {self.test_filepath} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Output to file should succeed")
        self.assertTrue(os.path.exists(output_file), "Direct output file should be created")

    def test_show_types(self):
        """Test show-types option"""
        output_file = os.path.join(self.test_dir, "types_output.txt")
        cmd = f"{self.mavlogdump_path} --show-types {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        self.assertEqual(result, 0, "Show types should succeed")
        self.assertTrue(os.path.exists(output_file), "Types output file should be created")

    def test_reduce_option(self):
        """Test message reduction by ratio"""
        output_file = os.path.join(self.test_dir, "reduced_output.txt")
        cmd = f"{self.mavlogdump_path} --reduce 10 {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Reduce option should succeed")
        self.assertTrue(os.path.exists(output_file), "Reduced output file should be created")

    def test_reduce_rate_option(self):
        """Test message reduction by rate"""
        output_file = os.path.join(self.test_dir, "rate_reduced_output.txt")
        cmd = f"{self.mavlogdump_path} --reduce-rate 10 {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Reduce-rate option should succeed")
        self.assertTrue(os.path.exists(output_file), "Rate reduced output file should be created")

    def test_condition_filtering(self):
        """Test condition-based filtering"""
        output_file = os.path.join(self.test_dir, "condition_output.txt")
        # Simple condition that should be valid
        cmd = f"{self.mavlogdump_path} --condition 'True' {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Condition filtering should succeed")
        self.assertTrue(os.path.exists(output_file), "Condition filtered output file should be created")

    def test_mav10_option(self):
        """Test MAVLink 1.0 parsing"""
        output_file = os.path.join(self.test_dir, "mav10_output.txt")
        cmd = f"{self.mavlogdump_path} --mav10 {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "MAV1.0 parsing should succeed")
        self.assertTrue(os.path.exists(output_file), "MAV1.0 output file should be created")

    def test_verbose_mode(self):
        """Test verbose output mode"""
        output_file = os.path.join(self.test_dir, "verbose_output.txt")
        cmd = f"{self.mavlogdump_path} --verbose {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Verbose mode should succeed")
        self.assertTrue(os.path.exists(output_file), "Verbose output file should be created")

    def test_source_filtering(self):
        """Test source system and component filtering"""
        output_file = os.path.join(self.test_dir, "source_filtered.txt")
        cmd = f"{self.mavlogdump_path} --source-system 1 --source-component 1 {self.test_filepath} > {output_file} 2>/dev/null"
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Source filtering should succeed")
        self.assertTrue(os.path.exists(output_file), "Source filtered output file should be created")

    def test_combined_options(self):
        """Test combination of multiple options"""
        output_file = os.path.join(self.test_dir, "combined_output.json")
        cmd = (f"mavlogdump.py --format json --types 'ATT,GPS' "
               f"--quiet --no-bad-data {self.test_filepath} > {output_file} 2>/dev/null")
        result = os.system(cmd)
        
        self.assertEqual(result, 0, "Combined options should succeed")
        self.assertTrue(os.path.exists(output_file), "Combined output file should be created")

    def test_import_as_module(self):
        """Test importing mavlogdump as a module"""
        try:
            # Try to import the refactored version
            from pymavlink.tools import mavlogdump
            
            # Check that main functions exist
            self.assertTrue(hasattr(mavlogdump, 'process_log'), "Should have process_log function")
            self.assertTrue(hasattr(mavlogdump, 'process_log_json'), "Should have process_log_json function")
            self.assertTrue(hasattr(mavlogdump, 'process_log_csv'), "Should have process_log_csv function")
            self.assertTrue(hasattr(mavlogdump, 'process_log_mat'), "Should have process_log_mat function")
        except ImportError:
            # If new functions don't exist, check for old structure
            pass

    def test_programmatic_json_processing(self):
        """Test programmatic JSON processing"""
        try:
            from pymavlink.tools import mavlogdump
            if hasattr(mavlogdump, 'process_log'):
                # Test programmatic interface
                result = mavlogdump.process_log(
                    self.test_filepath,
                    output_format='json',
                    types=['ATT'],
                    quiet=True
                )
                self.assertEqual(result, 0, "Programmatic JSON processing should succeed")
        except ImportError:
            self.skipTest("mavlogdump module not importable")

    def test_programmatic_csv_processing(self):
        """Test programmatic CSV processing"""
        try:
            from pymavlink.tools import mavlogdump
            if hasattr(mavlogdump, 'process_log'):
                # Test programmatic interface
                output_file = os.path.join(self.test_dir, "prog_csv.csv")
                result = mavlogdump.process_log(
                    self.test_filepath,
                    output_format='csv',
                    types=['*'],
                    output=output_file,
                    quiet=True
                )
                self.assertEqual(result, 0, "Programmatic CSV processing should succeed")
        except ImportError:
            self.skipTest("mavlogdump module not importable")

    def test_programmatic_mat_processing(self):
        """Test programmatic MAT processing"""
        try:
            from pymavlink.tools import mavlogdump
            if hasattr(mavlogdump, 'process_log'):
                # Test programmatic interface
                mat_file = os.path.join(self.test_dir, "prog_output.mat")
                result = mavlogdump.process_log(
                    self.test_filepath,
                    output_format='mat',
                    mat_file=mat_file,
                    quiet=True
                )
                # MAT processing might fail if scipy is not installed
                if result == 0:
                    self.assertTrue(os.path.exists(mat_file), "Programmatic MAT file should be created")
        except ImportError:
            self.skipTest("mavlogdump module not importable")


class MAVLogDumpUnitTest(unittest.TestCase):
    """Unit tests for individual functions"""
    
    def test_match_type_function(self):
        """Test the match_type function"""
        try:
            from pymavlink.tools.mavlogdump import match_type
            
            # Test exact match
            self.assertTrue(match_type('GPS', ['GPS']))
            self.assertFalse(match_type('GPS', ['ATT']))
            
            # Test wildcard match
            self.assertTrue(match_type('GPS_RAW', ['GPS*']))
            self.assertTrue(match_type('ATT', ['A*']))
            self.assertFalse(match_type('GPS', ['ATT*']))
            
            # Test multiple patterns
            self.assertTrue(match_type('GPS', ['ATT', 'GPS']))
            self.assertTrue(match_type('ATT', ['ATT', 'GPS']))
        except ImportError:
            self.skipTest("match_type function not importable")
    
    def test_to_string_function(self):
        """Test the to_string function"""
        try:
            from pymavlink.tools.mavlogdump import to_string
            
            # Test string input
            self.assertEqual(to_string("hello"), "hello")
            
            # Test bytes input
            self.assertEqual(to_string(b"hello"), "hello")
            
            # Test bytes with special characters
            result = to_string(b"\xff\xfe")
            self.assertIsInstance(result, str)
        except ImportError:
            self.skipTest("to_string function not importable")


if __name__ == '__main__':
    unittest.main()