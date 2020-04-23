import os
from os.path import isfile
import subprocess
import unittest


class TestDrakeVisualizerHelp(unittest.TestCase):
    def test(self):
        bin_path = "tools/drake_visualizer"
        self.assertTrue(isfile(bin_path), bin_path)
        text = subprocess.check_output([bin_path, "--help"], encoding="utf8")

        # N.B. This should be kept in sync with
        # `drake_visualizer_installed_help_test`.
        print(text)
        # Test for modifications in help text.
        self.assertIn("Drake Modifications", text)
        self.assertIn("--use_builtin_scripts", text)
        self.assertIn("Options: all,", text)
        # Test for nominal help string.
        self.assertIn("usage: drake-visualizer ", text)
        self.assertNotIn(
            "drake-visualizer: error: unrecognized arguments", text)
