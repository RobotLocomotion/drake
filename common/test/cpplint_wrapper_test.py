#!/usr/bin/python

"""cpplint_wrapper_test.py -- Unit test for cpplint_wrapper.py.
"""

import os
import subprocess
import sys
import unittest

# Set on command-line to the location of cpplint_wrapper.
_cpplint_wrapper_exe = None

# Set on command-line to the location of a valid header file.
_valid_header = None


class TestStringMethods(unittest.TestCase):

    def get_valid_header_filename(self):
        """This header should always exist, and pass cpplint."""
        self.assertTrue(os.path.exists(_valid_header))
        return _valid_header

    def run_and_expect(self, args, expected_exitcode, expected_regexps=None):
        try:
            self.assertTrue(_cpplint_wrapper_exe is not None)
            self.assertTrue(
                os.path.exists(_cpplint_wrapper_exe), _cpplint_wrapper_exe)
            output = subprocess.check_output(
                [sys.executable, _cpplint_wrapper_exe] + args,
                stderr=subprocess.STDOUT)
            returncode = 0
        except subprocess.CalledProcessError as e:
            output = e.output
            returncode = e.returncode
        self.assertEqual(returncode, expected_exitcode,
                         "output was:\n" + output)
        if expected_regexps is not None:
            for item in expected_regexps:
                self.assertRegexpMatches(output, item)

    def test_help(self):
        self.run_and_expect(
            ["--help"],
            0,
            ["usage: cpplint_wrapper"])

    def test_summarize(self):
        filename = self.get_valid_header_filename()
        self.run_and_expect(
            [filename],
            0,
            [r"Checking \.+\r?\n",
             r"TOTAL 1 files passed"])
        self.run_and_expect(
            ["--no-summarize", filename],
            0,
            [r"Done processing .*/eigen_types.h",
             r"TOTAL 1 files passed"])

    def test_true_positive(self):
        # Test that the header is clean by default.
        filename = (
            "external/styleguide/cpplint/cpplint_test_header.h")
        self.run_and_expect(
            [filename],
            0,
            [r"Checking \.+\r?\n",
             r"TOTAL 1 files passed"])
        # Test that pass-through arguments works.
        self.run_and_expect(
            [filename, "--", "--linelength=40"],
            1,
            [r"Checking \.+\r?\n",
             r"TOTAL 1 files checked",
             r"found [123] warnings",
             r"whitespace/line_length"])

    def test_ignored_extension(self):
        some_py_filename = sys.argv[0]
        self.run_and_expect(
            [some_py_filename],
            0,
            [r"Ignoring .*[/\\]cpplint_wrapper_test.py",
             r"TOTAL 0 files"])

    def test_num_processes(self):
        filename = self.get_valid_header_filename()
        self.run_and_expect(
            [filename] * 2,
            0,
            [r"TOTAL 2 files passed"])
        self.run_and_expect(
            ["--num-processes=1"] + [filename] * 2,
            0,
            [r"TOTAL 2 files passed"])
        self.run_and_expect(
            ["--num-processes=0"] + [filename] * 2,
            1)

    def test_huge(self):
        filename = self.get_valid_header_filename()
        self.run_and_expect(
            ["--num-processes=2"] + [filename] * 25,
            0,
            [r"Checking \.+\r?\n",
             r"TOTAL 25 files passed"])


if __name__ == '__main__':
    _cpplint_wrapper_exe = sys.argv.pop(1)
    _valid_header = sys.argv.pop(1)
    unittest.main()
