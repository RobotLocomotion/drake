import subprocess
import unittest

from bazel_tools.tools.python.runfiles import runfiles


class TestRelnotes(unittest.TestCase):

    def setUp(self):
        self.dut = runfiles.Create().Rlocation(
            "drake/tools/release_engineering/relnotes")

    def test_help(self):
        """Smoke test that --help works."""
        subprocess.check_call([self.dut, "--help"])
