"""Performs simple within-sandbox tests for resource_tool, exercising its
command-line API through all corner cases.
"""

import re
import subprocess
import os
import sys
import unittest


class TestResourceTool(unittest.TestCase):
    def setUp(self):
        self._resource_tool_exe, = sys.argv[1:]
        self.assertTrue(
            os.path.exists(self._resource_tool_exe),
            "Could not find " + self._resource_tool_exe)

    def _check_call(self, args, expected_returncode=0):
        """Run resource_tool with the given args; return output.
        """
        try:
            output = subprocess.check_output(
                [self._resource_tool_exe] + args,
                stderr=subprocess.STDOUT)
            returncode = 0
        except subprocess.CalledProcessError as e:
            output = e.output
            returncode = e.returncode
        self.assertEqual(
            returncode, expected_returncode,
            "Expected returncode %r from %r but got %r with output %r" % (
                expected_returncode, args, returncode, output))
        return output.decode('utf8')

    def test_help(self):
        output = self._check_call([
            "--help",
            ], expected_returncode=1)
        self.assertTrue("Find Drake-related resources" in output)
        self.assertGreater(len(output), 1000)

    def test_no_arguments(self):
        output = self._check_call([], expected_returncode=1)
        self.assertGreater(len(output), 1000)

    def test_too_many_arguments(self):
        self._check_call([
            "--print_resource_root_environment_variable_name",
            "--print_resource_path",
            "drake/common/test/resource_tool_test_data.txt",
            ], expected_returncode=1)

    def test_resource_root_environment_variable_name(self):
        output = self._check_call([
            "--print_resource_root_environment_variable_name",
            ], expected_returncode=0).strip()
        self.assertGreater(len(output), 0)
        # N.B. This test only confirms that the variable name is printed.  The
        # resource_tool_installed_test.py covers actually setting the variable.

    def test_print_resource_path_found(self):
        output = self._check_call([
            "--print_resource_path",
            "drake/common/test/resource_tool_test_data.txt",
            ], expected_returncode=0)
        absolute_path = output.strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(
                data.read(),
                "Test data for drake/common/test/resource_tool_test.py.\n")

    def test_print_resource_path_error(self):
        output = self._check_call([
            "--print_resource_path",
            "drake/no_such_file",
            ], expected_returncode=1)
        self.assertIn("Sought 'drake/no_such_file' in runfiles", output)
        self.assertIn("file does not exist", output)

    def test_help_example_is_correct(self):
        # Look at the usage message, and snarf its Pendulum example paths.
        output = self._check_call([
            "--help",
            ], expected_returncode=1)
        output = output.replace("\n", " ")
        m = re.search(
            (r"-print_resource_path.*`(drake.*)`.*"
             + r"absolute path.*?`/home/user/tmp/(drake/.*?)`"),
            output)
        self.assertTrue(m is not None, "Could not match in " + repr(output))

        example_relpath, example_abspath = m.groups()
        self.assertEqual(example_relpath, example_abspath)
        # Ask for the help message's example and make sure it still works.
        output = self._check_call([
            "--print_resource_path",
            example_relpath,
            ], expected_returncode=0)
        absolute_path = output.strip()
        self.assertTrue(absolute_path.endswith(example_abspath))
        self.assertTrue(os.path.exists(absolute_path))
