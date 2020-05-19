import subprocess
import unittest


class TestRunWithMotor(unittest.TestCase):
    def test_run_with_motor(self):
        """Test that run_with_motor doesn't crash."""
        subprocess.check_call(
            ["examples/multibody/strandbeest/run_with_motor",
             "--simulation_time=0.1"])


class TestEmpty(unittest.TestCase):
    def test_empty(self):
        """Empty test case for debug build"""
