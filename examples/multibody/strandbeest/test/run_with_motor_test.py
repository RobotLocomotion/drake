import subprocess
import unittest


class TestRunWithMotor(unittest.TestCase):
    def test_run_with_motor(self):
        """Test that run_with_motor doesn't crash."""
        # Test both an SDF and an URDF.
        subprocess.check_call(
            ["examples/multibody/strandbeest/run_with_motor"])


class TestEmpty(unittest.TestCase):
    def test_empty(self):
        """Empty test case for debug build"""
