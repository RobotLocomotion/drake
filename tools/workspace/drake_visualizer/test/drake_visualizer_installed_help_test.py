"""
Ensures we can call `./bin/drake-visualizer --help` from install.
"""

import os
import sys
import unittest

import install_test_helper


class TestDrakeVisualizerInstalledHelp(unittest.TestCase):
    def test_install(self):
        # Get install directory.
        install_dir = install_test_helper.get_install_dir()
        # N.B. Do not update PYTHONPATH, as the script should handle that
        # itself.
        bin_path = os.path.join(install_dir, "bin", "drake-visualizer")
        text = install_test_helper.check_output([bin_path, "--help"])

        # N.B. This should be kept in sync with
        # `drake_visualizer_bazel_help_test`.
        print(text)
        # Test for modifications in help text.
        self.assertIn("Drake Modifications", text)
        self.assertIn("--use_builtin_scripts", text)
        self.assertIn("Options: all,", text)
        # Test for nominal help string.
        self.assertIn("usage: drake-visualizer ", text)
        self.assertNotIn(
            "drake-visualizer: error: unrecognized arguments", text)


if __name__ == '__main__':
    unittest.main()
