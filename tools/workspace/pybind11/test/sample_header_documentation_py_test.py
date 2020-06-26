from os.path import join, realpath
import unittest
import subprocess


def _read(filename):
    with open(filename) as f:
        return f.read()

# To replace the expected header file with the most recent build's generated
# header file, use this command ...
#
# cp \
#  bazel-bin/tools/workspace/pybind11/test/sample_header_documentation.h \
#  tools/workspace/pybind11/test/sample_header_documentation.expected.h
#
# ... and then manually put back the "SCRUBBED" change.


class TestDocumentation(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_regression(self):
        d = "tools/workspace/pybind11/test"
        expected_file = join(d, "sample_header_documentation.expected.h")
        actual_file = join(d, "sample_header_documentation.h")
        # Ensure there is no difference between files.
        print("a: {}".format(expected_file))
        print("b: {}".format(actual_file))
        expected = _read(expected_file)
        actual = _read(actual_file).replace(
            "GENERATED FILE DO NOT EDIT",
            "<GENERIC MARKER SCRUBBED FOR REVIEWABLE>")
        self.assertMultiLineEqual(expected, actual)
