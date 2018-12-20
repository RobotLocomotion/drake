"""
Ensures that `build --python_path=` and
`build --action_env=DRAKE_PYTHON_BIN_PATH=` are consistent.

See `bazel_python_is_valid` in `repository.bzl` for usage.
"""

from __future__ import print_function
import os
import sys
import subprocess

from _bazel_python_actionenv import PYTHON_BIN_PATH


def indent(indent, text):
    return "\n".join([indent + line for line in text.split("\n")])


def get_interpreter_info(python):
    # Do not compare paths, as we may get something like `/usr/bin/python` and
    # `/usr/bin/python2.7`. Instead, ensure we have the same version
    # information and prefix (which should be invariant of symlink aliases and
    # shell forwarding scripts).
    return subprocess.check_output([
        python, "-c",
        "import sys; print('prefix: {}\\nversion: {}'" +
        ".format(sys.prefix, sys.version_info))"]).decode("utf8")


def main():
    python_bazel = sys.executable
    python_actionenv = PYTHON_BIN_PATH
    info_bazel = get_interpreter_info(python_bazel)
    info_actionenv = get_interpreter_info(python_actionenv)
    if info_bazel != info_actionenv:
        info = dict(
            python_bazel=python_bazel,
            info_bazel=indent(4*" ", info_bazel.strip()),
            python_actionenv=python_actionenv,
            info_actionenv=indent(4*" ", info_actionenv.strip()),
        )
        print("""
Mismatch in Python executables specified to Bazel:
  build --python_path={python_bazel}
{info_bazel}
  build --action_env=DRAKE_PYTHON_BIN_PATH={python_actionenv}
{info_actionenv}
Please rerun `install_prereqs.sh`.
""".format(**info), file=sys.stderr)
        sys.exit(1)


assert __name__ == "__main__"
main()
