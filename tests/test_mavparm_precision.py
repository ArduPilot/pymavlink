import tempfile
import unittest
from pathlib import Path
from pymavlink.mavparm import MAVParmDict


class ParameterPrecisionTest(unittest.TestCase):
    def test_save_load_preserves_float_values(self):
        parameters = MAVParmDict()
        values = [1.23456789, 4e-7, -3e-9, 12345678.125, 0.0, -0.0]
        for i, value in enumerate(values):
            parameters["VALUE%d" % i] = value
        with tempfile.TemporaryDirectory() as directory:
            filename = str(Path(directory) / "parameters.parm")
            parameters.save(filename)
            restored = MAVParmDict()
            self.assertTrue(restored.load(filename, use_excludes=False))
        for key, value in parameters.items():
            self.assertEqual(restored[key], value, key)
