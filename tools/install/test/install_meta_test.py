"""Tests the behavior of targets generated from `install.py.in`."""

import os
from os.path import isdir, isfile, join
from shutil import rmtree
import unittest
from subprocess import STDOUT, call, check_call, check_output
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

    def test_nominal(self):
        """Test nominal behavior of install."""
        install_dir = self.get_install_dir("test_nominal")
        check_call([self.BINARY, install_dir])
        py_dir = self.get_python_site_packages_dir()
        expected_manifest = [
            "share/README.md",
            "lib/libdummy.so",
            join(py_dir, "dummy.py"),
        ]
        for expected_file in expected_manifest:
            file_path = join(install_dir, expected_file)
            self.assertTrue(isfile(file_path), expected_file)

    def test_strip_args(self):
        """Test behavior of `--no_strip` and related arguments."""
        install_dir = self.get_install_dir("test_strip_args")
        # Negative test: Ensure `--disable_nostrip_warning` is not used in a
        # blanket fashion.
        returncode = call(
            [self.BINARY, install_dir, "--disable_no_strip_warning"])
        self.assertEqual(returncode, 1)
        # Test for warnings with no stripping:
        strip_substr = "NOTE: Symbols are not stripped."
        warning_substr = "WARNING: Proprietary dependencies are enabled."
        # - Nominal.
        text_with_warning = check_output([
            self.BINARY, install_dir, "--no_strip"], stderr=STDOUT)
        # N.B. `assertIn` error messages are not great for multiline, so just
        # print and use nominal asserts.
        print(text_with_warning)
        self.assertTrue(strip_substr in text_with_warning)
        self.assertTrue(warning_substr in text_with_warning)
        # - Warning disabled.
        text_without_warning = check_output([
            self.BINARY, install_dir, "--no_strip",
            "--disable_no_strip_warning"], stderr=STDOUT)
        print(text_without_warning)
        self.assertTrue(strip_substr in text_without_warning)
        self.assertTrue(warning_substr not in text_without_warning)
