"""Tests the behavior of targets that use `installer.py`."""

import os
from os.path import join
import unittest

from tools.install.installer import Installer


class TestInstall(unittest.TestCase):
    def get_tmpdir(self):
        assert "TEST_TMPDIR" in os.environ, (
            "Must only be run within `bazel test`."
        )
        return os.environ["TEST_TMPDIR"]

    def get_install_dir(self, case):
        install_dir = join(self.get_tmpdir(), "installation", case)
        # Ensure this is only called once per case.
        os.makedirs(install_dir, exist_ok=False)
        return install_dir

    def test_install_symlink(self):
        installer = Installer()

        # Create a relative symlink to be installed (which happens to be a
        # broken link, but that doesn't matter for the test). Note that 'src'
        # here refers to the location of the artifact to be installed.
        src = join(self.get_tmpdir(), "link")
        os.symlink(src="missing", dst=src)

        # Get the install location.
        install_dir = self.get_install_dir("install_symlink")
        dst = join(install_dir, "link")

        # Install the link and verify that it was installed.
        installer.copy_or_link(src, dst)
        assert os.path.islink(dst), "Link was not created."

        # Repeat, to verify that we can overwrite an existing symlink.
        # (See issue #18007.)
        installer.copy_or_link(src, dst)
        assert os.path.islink(dst), "Link was not created."
