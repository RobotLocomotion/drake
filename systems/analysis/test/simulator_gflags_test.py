import os
import sys
import subprocess
import unittest

from pydrake.common import FindResourceOrThrow


class TestSimulatorGflags(unittest.TestCase):
    def setUp(self):
        self.dut = FindResourceOrThrow(
            f"drake/systems/analysis/simulator_gflags_main")

    def test(self):
        result = subprocess.run([
            self.dut, "--simulator_integration_scheme=implicit_euler",
            "--simulator_max_time_step=0.01", "--simulator_accuracy=1E-3",
            "--simulator_use_error_control=true",
            "--simulator_target_realtime_rate=1.",
            "--simulator_publish_every_time_step=true"
        ],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                encoding="utf-8")
        # TODO(hongkai.dai): why the drake::log()->info(...) appears in
        # result.stderr?
        # self.assertEqual(result.stderr, "")
        self.assertIn(result.stdout, "integration_scheme: implicit_euler")


if __name__ == "__main__":
    if "--cc" in sys.argv:
        sys.argv.remove("--cc")
        _backend = "cc"
    unittest.main()
