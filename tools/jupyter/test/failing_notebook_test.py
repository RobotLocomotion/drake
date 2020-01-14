import subprocess
import unittest


class TestFailingNotebook(unittest.TestCase):
    def test_failing_notebook(self):
        p = subprocess.Popen([
            "tools/jupyter/failing_notebook",
            "--test",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, _ = p.communicate()
        self.assertEqual(p.poll(), 1)
        self.assertIn("CellExecutionError", stdout.decode("utf8"))

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
