"""cpplint_wrapper_test.py -- Unit test for cpplint_wrapper.py.
"""

import os
import subprocess
import sys
import unittest


class TestStringMethods(unittest.TestCase):
    def setUp(self):
        self.cpplint_wrapper_exe, self.valid_header_filename = sys.argv[1:]
        self.assertTrue(
            os.path.exists(self.cpplint_wrapper_exe),
            "Could not find " + self.cpplint_wrapper_exe)
        self.assertTrue(
            os.path.exists(self.valid_header_filename),
            "Could not find " + self.valid_header_filename)

    def run_and_expect(self, args, expected_exitcode, expected_regexps=None):
        try:
            output = subprocess.check_output(
                [sys.executable, self.cpplint_wrapper_exe] + args,
                stderr=subprocess.STDOUT).decode('utf8')
            returncode = 0
        except subprocess.CalledProcessError as e:
            output = e.output.decode('utf8')
            returncode = e.returncode
        self.assertEqual(returncode, expected_exitcode,
                         "output was:\n" + output)
        if expected_regexps is not None:
            for item in expected_regexps:
                self.assertRegex(output, item)

    def test_help(self):
        self.run_and_expect(
            ["--help"],
            0,
            ["usage: cpplint_wrapper"])

    def test_summarize(self):
        filename = self.valid_header_filename
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

    def test_pass_through_arguments(self):
        filename = self.valid_header_filename
        self.run_and_expect(
            [filename],
            0,
            [r"TOTAL 1 files passed"])
        self.run_and_expect(
            [filename, "--", "--linelength=40"],
            1,
            [r"Checking \.+\r?\n",
             r"TOTAL 1 files checked",
             r"found [1-9][0-9]* warnings",
             r"whitespace/line_length"])

    def test_ignored_extension(self):
        some_py_filename = sys.argv[0]
        self.run_and_expect(
            [some_py_filename],
            0,
            [r"Ignoring \S*\.py; not a valid file name",
             r"TOTAL 0 files"])

    def test_num_processes(self):
        filename = self.valid_header_filename
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
        filename = self.valid_header_filename
        self.run_and_expect(
            ["--num-processes=2"] + [filename] * 25,
            0,
            [r"Checking \.+\r?\n",
             r"TOTAL 25 files passed"])
