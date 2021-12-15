from os.path import isfile
import subprocess
import sys
import unittest


class TestDrakeVisualizerBazel(unittest.TestCase):

    def _check_help_output(self):
        bin_path = "tools/drake_visualizer"
        self.assertTrue(isfile(bin_path), bin_path)
        return subprocess.check_output([bin_path, "--help"], encoding="utf8")

    def test_help_basic(self):
        """Ensure we can call drake_visualizer --help in Bazel."""
        self._check_help_output()

    def test_help_details(self):
        """Check that the help string is from the correct program."""
        text = self._check_help_output()
        # N.B. This should be kept in sync with
        # `drake_visualizer_installed_help_test`.
        print(text)
        # Test for nominal help string.
        self.assertIn("usage: drake-visualizer ", text)
        self.assertNotIn(
            "drake-visualizer: error: unrecognized arguments", text)
        # Test for modifications in help text.
        self.assertIn("--use_builtin_scripts", text)
        self.assertIn("Options: all,", text)
