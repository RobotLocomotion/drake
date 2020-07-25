"""
Regression test for mkdoc edge cases.

See README for updating.
"""

from os.path import join, realpath
import unittest
import subprocess


def _read(filename):
    with open(filename) as f:
        return f.read()


class TestDocumentation(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_regression(self):
        my_dir = "tools/workspace/pybind11/test"
        expected_file = join(
            my_dir, "output_expected/sample_header_documentation.h"
        )
        actual_file = join(my_dir, "output/sample_header_documentation.h")
        # Ensure there is no difference between files.
        print("a: {}".format(expected_file))
        print("b: {}".format(actual_file))
        expected = _read(expected_file)
        actual = _read(actual_file).replace(
            "GENERATED FILE DO NOT EDIT",
            "<GENERIC MARKER SCRUBBED FOR REVIEWABLE>")
        self.assertMultiLineEqual(expected, actual)
