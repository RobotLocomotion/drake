"""Tests the behavior of targets that use `installer.py`."""

from contextlib import redirect_stderr, redirect_stdout
import io
import os
from os.path import isdir, join, relpath
from subprocess import STDOUT, check_output
import sys
import unittest

import tools.install.installer as installer

# TODO(eric.cousineau): Expand on these tests, especially for nuanced things
# like Python C extensions.


class TestInstallMeta(unittest.TestCase):
    def get_install_dir(self, case):
        # Do not use `install_test_helper`, as its behavior is more constrained
        # than what is useful for these tests.
        assert "TEST_TMPDIR" in os.environ, (
            "Must only be run within `bazel test`."
        )
        install_dir = join(os.environ["TEST_TMPDIR"], "installation", case)
        # Ensure this is only called once per case.
        self.assertFalse(isdir(install_dir), case)
        return install_dir

    def get_python_site_packages_dir(self):
        major, minor = sys.version_info.major, sys.version_info.minor
        return join("lib", "python{}.{}".format(major, minor), "site-packages")

    def listdir_recursive(self, d):
        out = set()
        for dirpath, _, filenames in os.walk(d):
            dirrel = relpath(dirpath, d)
            for filename in filenames:
                out.add(join(dirrel, filename))
        return out

    def dummy_install_import(self, *args):
        """Invoke installation of dummy files by direct import and call to
        installer.
        """
        stream = io.StringIO()
        with redirect_stdout(stream), redirect_stderr(stream):
            installer.main(
                ["--actions", "tools/install/dummy/install_actions"]
                + list(args)
            )
        return stream.getvalue()

    def dummy_install_binary(self, *args):
        """Invoke installation of dummy files by the built ``install``
        binary.
        """
        BINARY = "tools/install/dummy/install"
        return check_output(
            [BINARY] + list(args), stderr=STDOUT, encoding="utf8"
        )

    def do_test_nominal(self, test_name, invoker):
        """Test nominal behavior of install."""
        install_dir = self.get_install_dir(test_name)
        invoker(install_dir)
        py_dir = self.get_python_site_packages_dir()
        expected_manifest = {
            "share/README.md",
            "include/dummy.h",
            "lib/libdummy.so",
            join(py_dir, "dummy.py"),
        }
        actual_manifest = self.listdir_recursive(install_dir)
        self.assertSetEqual(actual_manifest, expected_manifest)

    def do_test_strip_args(self, test_name, invoker):
        """Test behavior of `--no_strip`."""
        install_dir = self.get_install_dir(test_name)
        # - Without.
        substr_without = "Installing the project stripped..."
        text_without = invoker(install_dir)
        # N.B. `assertIn` error messages are not great for multiline, so just
        # print and use nominal asserts.
        print(text_without)
        self.assertTrue(substr_without in text_without)
        # - With.
        substr_with = "Install the project..."
        text_with = invoker(install_dir, "--no_strip")
        print(text_with)
        self.assertTrue(substr_with in text_with)

    def test_nominal_by_import(self):
        self.do_test_nominal("nominal_by_import", self.dummy_install_import)

    def test_nominal_by_binary(self):
        self.do_test_nominal("nominal_by_binary", self.dummy_install_binary)

    def test_strip_args_by_import(self):
        self.do_test_strip_args(
            "strip_args_by_import", self.dummy_install_import
        )

    def test_strip_args_by_binary(self):
        self.do_test_strip_args(
            "strip_args_by_binary", self.dummy_install_binary
        )
