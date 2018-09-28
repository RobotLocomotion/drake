from os.path import join, realpath
import unittest
import subprocess


class TestDocumentation(unittest.TestCase):
    def test_regression(self):
        d = "tools/workspace/pybind11/test"
        expected_file = join(d, "test_header_documentation.expected.h")
        actual_file = join(d, "test_header_documentation.h")
        # Ensure there is no difference between files.
        # N.B. While `git diff --no-index` looks prettier, it's hard to get it
        # to ignore symlinks + filemodes.
        print("a: {}".format(expected_file))
        print("b: {}".format(actual_file))
        subprocess.check_call(["diff", expected_file, actual_file])
