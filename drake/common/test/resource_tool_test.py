"""Performs simple within-sandbox tests for resource_tool, exercising its
command-line API through all corner cases.
"""

import subprocess
import os
import unittest


class TestResourceTool(unittest.TestCase):
    def _run_with_returncode_and_output(self, args):
        """Run resource_tool with the given args; return returncode, output.
        """
        try:
            output = subprocess.check_output(
                ["drake/common/resource_tool"] + args,
                stderr=subprocess.STDOUT)
            returncode = 0
        except subprocess.CalledProcessError as e:
            output = e.output
            returncode = e.returncode
        return returncode, output

    def test_help(self):
        returncode, output = self._run_with_returncode_and_output([
            "--help",
            ])
        self.assertEqual(returncode, 1)
        self.assertTrue("Find Drake-related resources" in output)
        self.assertGreater(len(output), 1000)

    def test_no_arguments(self):
        returncode, output = self._run_with_returncode_and_output([])
        self.assertEqual(returncode, 1)
        self.assertGreater(len(output), 1000)

    def test_print_resource_path_found(self):
        returncode, output = self._run_with_returncode_and_output([
             "--print_resource_path",
             "drake/common/test/resource_tool_test_data.txt",
            ])
        absolute_path = output.strip()
        self.assertEqual(returncode, 0)
        with open(absolute_path, 'r') as data:
            self.assertEqual(
                data.read(),
                "Test data for drake/common/test/resource_tool_test.py.\n")

    def test_print_resource_path_error(self):
        returncode, output = self._run_with_returncode_and_output([
            "--print_resource_path",
            "drake/no_such_file",
            ])
        self.assertEqual(returncode, 1)
        self.assertTrue("could not find resource" in output)


if __name__ == '__main__':
    unittest.main()
