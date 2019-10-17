import subprocess
import unittest

# TODO(eric.cousineau): Auto-generate a generic module and import that, rather
# than (ab)use a notebook for checking support.
# Import notebook to check if Jupyter is supported.
import drake.tools.jupyter.failing_notebook_jupyter_py_main as mut

IS_UNSUPPORTED = mut._is_unsupported


class TestJupyterNotebook(unittest.TestCase):
    @unittest.skipIf(IS_UNSUPPORTED, "Unsupported Jupyter platform")
    def test_help(self):
        """Ensures that `jupyter notebook` is installed (#12042)."""
        status = subprocess.call(args=["jupyter", "notebook", "--help"])
        self.assertEqual(status, 0)
