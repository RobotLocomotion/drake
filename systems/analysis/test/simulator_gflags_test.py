import subprocess
import unittest

from python import runfiles


class TestSimulatorGflags(unittest.TestCase):
    def setUp(self):
        manifest = runfiles.Create()
        self.dut = manifest.Rlocation(
            "drake/systems/analysis/simulator_gflags_main"
        )

    def test(self):
        result = subprocess.run(
            [
                self.dut,
                "--simulator_integration_scheme=implicit_euler",
                "--simulator_max_time_step=0.01",
                "--simulator_accuracy=1E-3",
                "--simulator_use_error_control=true",
                "--simulator_target_realtime_rate=1.",
                # Delete next line with publish_every_time_step 2026-06-01
                "--simulator_publish_every_time_step=true",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            encoding="utf-8",
        )
        self.assertIn("integration_scheme: implicit_euler", result.stdout)
