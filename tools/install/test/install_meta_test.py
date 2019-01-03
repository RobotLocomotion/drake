"""Tests the behavior of targets generated from `install.py.in`."""

import os
from os.path import isdir, join, relpath
import unittest
from subprocess import STDOUT, check_call, check_output
import sys

# TODO(eric.cousineau): Expand on these tests, especially for nuanced things
# like Python C extensions.


class TestInstallMeta(unittest.TestCase):
    BINARY = "tools/install/dummy/install"

    def get_install_dir(self, case):
        # Do not use `install_test_helper`, as its behavior is more constrained
        # than what is useful for these tests.
        assert "TEST_TMPDIR" in os.environ, (
            "Must only be run within `bazel test`.")
        install_dir = join(os.environ["TEST_TMPDIR"], "installation", case)
        # Ensure this is only called once per case.
        self.assertFalse(isdir(install_dir), case)
        return install_dir

    def get_python_site_packages_dir(self):
        major, minor = sys.version_info.major, sys.version_info.minor
        return join("lib", "python{}.{}".format(major, minor), "site-packages")

    def listdir_recursive(self, d):
        out = set()
        for (dirpath, _, filenames) in os.walk(d):
            dirrel = relpath(dirpath, d)
            for filename in filenames:
                out.add(join(dirrel, filename))
        return out

    def test_nominal(self):
        """Test nominal behavior of install."""
        install_dir = self.get_install_dir("test_nominal")
        check_call([self.BINARY, install_dir])
        py_dir = self.get_python_site_packages_dir()
        expected_manifest = {
            "share/README.md",
            "include/dummy.h",
            "lib/libdummy.so",
            join(py_dir, "dummy.py"),
        }
        actual_manifest = self.listdir_recursive(install_dir)
        self.assertSetEqual(actual_manifest, expected_manifest)

    def test_strip_args(self):
        """Test behavior of `--no_strip`."""
        install_dir = self.get_install_dir("test_strip_args")
        # - Without.
        substr_without = "Installing the project stripped..."
        text_without = check_output(
            [self.BINARY, install_dir], stderr=STDOUT).decode("utf8")
        # N.B. `assertIn` error messages are not great for multiline, so just
        # print and use nominal asserts.
        print(text_without)
        self.assertTrue(substr_without in text_without)
        # - With.
        substr_with = "Install the project..."
        text_with = check_output(
            [self.BINARY, install_dir, "--no_strip"],
            stderr=STDOUT).decode("utf8")
        print(text_with)
        self.assertTrue(substr_with in text_with)
