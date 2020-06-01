from os.path import isfile
import unittest
from subprocess import run
import sys


class TestCreatePlot(unittest.TestCase):
    def assert_isfile(self, file):
        self.assertTrue(isfile(file), file)

    def test_sripts(self):
        # Must be run directly via Bazel.
        python_bin = sys.executable
        run([python_bin, "multibody/plant/images/ideal_stiction.py"],
            check=True)
        self.assert_isfile("./ideal_stiction.png")
        run([python_bin, "multibody/plant/images/stiction.py"],
            check=True)
        self.assert_isfile("./stribeck.png")
