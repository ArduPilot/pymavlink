"""Regression tests for magnetometer selection in magfit_WMM."""

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO = Path(__file__).resolve().parents[1]


class Message:
    def __init__(self, name, **fields):
        self.name = name
        self.__dict__.update(fields)

    def get_type(self):
        return self.name


class Log:
    def __init__(self, messages):
        self.messages = messages
        self.position = 0

    def recv_match(self, type=None, condition=None):
        while self.position < len(self.messages):
            message = self.messages[self.position]
            self.position += 1
            if message.get_type() in type:
                return message
        return None

    def rewind(self):
        self.position = 0


class MagfitSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            "magfit_WMM", REPO / "tools" / "magfit_WMM.py"
        )
        with patch.object(sys, "argv", ["magfit_WMM.py", "synthetic.BIN"]):
            cls.script = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.script)

    def selected(self, messages, mag, reduce=1):
        script = self.script
        script.args.mag = mag
        script.args.reduce = reduce
        script.args.att_source = "ATT"
        script.earth_field = None
        script.new_param_format = None
        parameters = [
            Message("PARM", Name=name, Value=value)
            for name, value in (
                ("AHRS_TRIM_X", 0),
                ("AHRS_TRIM_Y", 0),
                ("AHRS_TRIM_Z", 0),
                ("COMPASS_OFS_X", 0),
            )
        ]
        attitude = Message("ATT", Roll=0, Pitch=0, Yaw=0)
        log = Log(parameters + [attitude] + messages)
        stop = RuntimeError("samples collected")
        with patch.object(script.mavutil, "mavlink_connection", return_value=log), \
                patch.object(script, "remove_offsets", return_value=True), \
                patch.object(script, "fit_WWW", side_effect=stop):
            try:
                script.magfit("synthetic.BIN")
            except RuntimeError as error:
                if error is not stop:
                    raise
        return [sample[0].MagX for sample in script.data]

    def test_legacy_message_names_keep_one_based_mag_selection(self):
        messages = [
            Message("MAG", MagX=10),
            Message("MAG2", MagX=20),
            Message("MAG3", MagX=30),
        ]
        self.assertEqual(self.selected(messages, 1), [10])
        self.assertEqual(self.selected(messages, 2), [20])
        self.assertEqual(self.selected(messages, 3), [30])

    def test_modern_mag_instance_matches_one_based_cli_index(self):
        messages = [
            Message("MAG", I=0, MagX=10),
            Message("MAG", I=1, MagX=20),
            Message("MAG", I=2, MagX=30),
            Message("MAG", I=1, MagX=21),
        ]
        self.assertEqual(self.selected(messages, 1), [10])
        self.assertEqual(self.selected(messages, 2), [20, 21])
        self.assertEqual(self.selected(messages, 3), [30])

    def test_mixed_and_missing_instance_fields_do_not_cross_sensors(self):
        messages = [
            Message("MAG", I=0, MagX=10),
            Message("MAG", I=1, MagX=20),
            Message("MAG2", MagX=21),
            Message("MAG2", I=0, MagX=99),
            Message("MAG", I=2, MagX=30),
            Message("MAG", MagX=11),
        ]
        self.assertEqual(self.selected(messages, 1), [10, 11])
        self.assertEqual(self.selected(messages, 2), [20, 21])
        self.assertEqual(self.selected(messages, 3), [30])

    def test_reduce_counts_only_selected_sensor(self):
        messages = [
            Message("MAG", I=0, MagX=10),
            Message("MAG", I=1, MagX=20),
            Message("MAG", I=0, MagX=11),
            Message("MAG", I=1, MagX=21),
            Message("MAG", I=0, MagX=12),
            Message("MAG", I=1, MagX=22),
        ]
        self.assertEqual(self.selected(messages, 2, reduce=2), [20, 22])


if __name__ == "__main__":
    unittest.main()
