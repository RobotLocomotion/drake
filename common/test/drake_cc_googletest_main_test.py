"""This program unit tests the command-line processing capabilities of the
drake_cc_googletest bazel macro, by running
`bazel-bin/drake/common/drake_cc_googletest_main_test`
with a variety of command-line flags.
"""

import re
import subprocess
import os
import sys
import unittest


class TestGtestMain(unittest.TestCase):
    def setUp(self):
        self._main_exe, = sys.argv[1:]
        self.assertTrue(
            os.path.exists(self._main_exe),
            "Could not find " + self._main_exe)

    def _check_call(self, args, expected_returncode=0):
        """Run _main_exe with the given args; return output.
        """
        try:
            output = subprocess.check_output(
                [self._main_exe] + args,
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

    def test_pass(self):
        # The device under test should pass when -magic_number=1.0 is present.
        self._check_call(["-magic_number=1.0"], expected_returncode=0)

    def test_no_arguments(self):
        # The device under test should fail when -magic_number=1.0 is missing.
        output = self._check_call([], expected_returncode=1)
        self.assertTrue("Expected equality of these values:\n"
                        "  FLAGS_magic_number" in output)

    def test_help(self):
        # The help string should mention all options.  Just spot-check for one
        # option from each expected contributor.
        output = self._check_call([
            "--help",
            ], expected_returncode=1)
        self.assertGreater(len(output), 1000)
        self.assertTrue("Using drake_cc_googletest_main" in output)
        self.assertTrue("-gtest_list_tests" in output)
        self.assertTrue("-spdlog_level" in output)
        self.assertTrue("-magic_number" in output)

    def test_logging(self):
        # The spdlog flags should be able to enable debug logging.

        # By default, there is no debug log.
        log_message = "[debug] Cross your fingers for the magic_number 1"
        args = ["-magic_number=1.0"]
        output = self._check_call(args, expected_returncode=0)
        self.assertFalse(log_message in output, output)

        # Once enabled, we see a debug log.
        args.append("-spdlog_level=debug")
        output = self._check_call(args, expected_returncode=0)
        self.assertTrue(log_message in output, output)
