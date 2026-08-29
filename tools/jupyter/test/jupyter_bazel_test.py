import sys
import unittest
from unittest import mock

from jupyter_bazel import _jupyter_bazel_notebook_main


class TestJupyterBazel(unittest.TestCase):
    def test_interactive_disables_redirect_file(self):
        """Interactive launch must not use a /tmp HTML redirect (#24071)."""
        notebook_path = "/tmp/fake_notebook.ipynb"
        with (
            mock.patch("jupyter_bazel.runfiles.Create") as mock_create,
            mock.patch("jupyter_bazel._jupyter_main") as mock_jupyter_main,
            mock.patch("jupyter_bazel.sys.exit") as mock_exit,
            mock.patch(
                "jupyter_bazel.os.path.realpath", return_value=notebook_path
            ),
        ):
            mock_create.return_value.Rlocation.return_value = notebook_path
            mock_jupyter_main.return_value = 0

            _jupyter_bazel_notebook_main(
                "drake/tools/jupyter/example.ipynb", []
            )

            mock_jupyter_main.assert_called_once_with()
            mock_exit.assert_called_once_with(0)
            self.assertEqual(
                sys.argv,
                [
                    "jupyter",
                    "notebook",
                    "--ServerApp.use_redirect_file=False",
                    notebook_path,
                ],
            )
