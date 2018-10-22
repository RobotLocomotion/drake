from subprocess import check_call
import unittest

# This will check the Bazel Python version (2 or 3).
from lcm import LCM


class TestLcmPython2and3(unittest.TestCase):
    def test_python2(self):
        # Explicitly check Python2.
        check_call(["python2", "-c", "from lcm import LCM"])
