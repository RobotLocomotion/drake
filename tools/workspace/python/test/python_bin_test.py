import os
import sys
import subprocess
import unittest

from bazel_python_actionenv import PYTHON_BIN_PATH


def get_interpreter_info(python):
    # Do not compare paths, as we may get something like `/usr/bin/python` and
    # `/usr/bin/python2.7`. Instead, ensure we have the same version
    # information and prefix (which should be invariant of symlink aliases and
    # shell forwarding scripts).
    return subprocess.check_output([
        python, "-c",
        "import sys; print('prefix: {}\\nversion: {}'" +
        ".format(sys.prefix, sys.version_info))"]).decode("utf8")


class TestPythonBin(unittest.TestCase):
    def test_bazel_and_env(self):
        """Ensures that we are supplying consistent options to Bazel.
        """
        python_bazel = sys.executable
        info_bazel = get_interpreter_info(python_bazel)
        python_actionenv = PYTHON_BIN_PATH
        info_actionenv = get_interpreter_info(python_actionenv)
        if info_bazel != info_actionenv:
            message = (
                "\n\nMismatch in Python executables:\n"
                "  bazel --python_path={}\n"
                "  bazel --action_env=PYTHON_BIN_PATH={}\n"
                "If you specify one of these, please ensure that you "
                "specify both.").format(python_bazel, python_actionenv)
            # In Python2, providing a longMessage will override the useful text
            # comparison; as a workaround, we'll print out our debugging
            # message beforehand.
            print(message)
            self.assertMultiLineEqual(info_bazel, info_actionenv)
