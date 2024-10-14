import subprocess
import sys
import unittest

from python import runfiles


class TestJupyterNotebook(unittest.TestCase):
    def test_help(self):
        """Ensures that `jupyter notebook` is installed (#12042)."""
        manifest = runfiles.Create()
        if sys.platform == "darwin":
            jupyter = manifest.Rlocation("python/bin/jupyter")
        else:
            jupyter = "jupyter"

        status = subprocess.call(args=[jupyter, "notebook", "--help"])
        self.assertEqual(status, 0)
