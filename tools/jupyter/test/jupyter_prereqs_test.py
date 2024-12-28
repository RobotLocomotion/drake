import unittest

from jupyter_core.command import list_subcommands


class TestJupyterPrereqs(unittest.TestCase):
    """Sanity checks that our install_prereqs was sufficient."""

    def test_notebook(self):
        """Ensures that `jupyter notebook` is installed (#12042). That
        subcommand is required by _jupyter_bazel_notebook_main when running
        interactively (i.e., when in non-test mode).
        """
        self.assertIn("notebook", list_subcommands())
