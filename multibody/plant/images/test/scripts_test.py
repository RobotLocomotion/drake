from os.path import isfile
import unittest
from subprocess import run


class TestCreatePlot(unittest.TestCase):
    def assert_isfile(self, file):
        self.assertTrue(isfile(file), file)

    def test_sripts(self):
        # Must be run directly via Bazel.
        run(["python3", "multibody/plant/images/ideal_stiction.py"],
            check=True)
        self.assert_isfile("./ideal_stiction.png")
        run(["python3", "multibody/plant/images/stiction.py"],
            check=True)
        self.assert_isfile("./stribeck.png")
