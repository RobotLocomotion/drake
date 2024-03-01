import subprocess
import sys
import unittest


class TestJupyterNotebook(unittest.TestCase):
    def test_help(self):
        """Ensures that `jupyter notebook` is installed (#12042)."""
        status = subprocess.call(
            args=[sys.executable, "-m", "notebook", "--help"])
        self.assertEqual(status, 0)
