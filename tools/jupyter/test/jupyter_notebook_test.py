import subprocess
import unittest

from python.runfiles import Rlocation

class TestJupyterNotebook(unittest.TestCase):
    def test_help(self):
        print(Rlocation)
        """Ensures that `jupyter notebook` is installed (#12042)."""
        status = subprocess.call(args=["jupyter", "notebook", "--help"])
        self.assertEqual(status, 0)
