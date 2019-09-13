import subprocess
import unittest

# Import notebook to check if Jupyter is supported.
import drake.tools.jupyter.failing_notebook_jupyter_py_main as mut

IS_UNSUPPORTED = mut._is_unsupported


class TestFailingNotebook(unittest.TestCase):
    @unittest.skipIf(IS_UNSUPPORTED, "Unsupported Jupyter platform")
    def test_failing_notebook(self):
        p = subprocess.Popen([
            "tools/jupyter/failing_notebook",
            "--test",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, _ = p.communicate()
        self.assertEqual(p.poll(), 1)
        self.assertIn("CellExecutionError", stdout.decode("utf8"))

    @unittest.skipIf(IS_UNSUPPORTED, "Unsupported Jupyter platform")
    def test_failing_notebook_deprecation(self):
        p = subprocess.Popen([
            "tools/jupyter/failing_notebook_deprecation",
            "--test",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, _ = p.communicate()
        self.assertEqual(p.poll(), 1)
        message = stdout.decode("utf8")
        self.assertIn("CellExecutionError", message)
        self.assertIn("DrakeDeprecationWarning: Deprecation example", message)
