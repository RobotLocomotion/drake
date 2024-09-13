import os
import subprocess
import unittest

from python import runfiles


class TestJupyterNotebook(unittest.TestCase):
    def test_help(self):
        """Ensures that `jupyter notebook` is installed (#12042)."""
        manifest = runfiles.Create()
        jupyter = manifest.Rlocation("venv/bin/jupyter")
        if not os.path.exists(jupyter):
            jupyter = "jupyter"

        status = subprocess.call(args=[jupyter, "notebook", "--help"])
        self.assertEqual(status, 0)
