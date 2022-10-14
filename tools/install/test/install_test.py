"""Tests the behavior of targets that use `installer.py`."""

import os
from os.path import isdir, join
import unittest

from drake.tools.install.installer import Installer


class TestInstall(unittest.TestCase):
    def get_tmpdir(self):
        assert "TEST_TMPDIR" in os.environ, (
            "Must only be run within `bazel test`.")
        return os.environ["TEST_TMPDIR"]

    def get_install_dir(self, case):
        install_dir = join(self.get_tmpdir(), "installation", case)
        # Ensure this is only called once per case.
        os.makedirs(install_dir, exist_ok=False)
        return install_dir

    def test_install_symlink(self):
        installer = Installer()

        src = join(self.get_tmpdir(), 'link')
        os.symlink("missing", src)

        install_dir = self.get_install_dir("install_symlink")
        dst = join(install_dir, 'link')

        installer.copy_or_link(src, dst)
        assert os.path.islink(dst), "Link was not created."

        installer.copy_or_link(src, dst)
        assert os.path.islink(dst), "Link was not created."
