"""
Ensures we can import `pydrake.all` from install.
"""

import os
import sys
import unittest

import install_test_helper


class TestAllInstall(unittest.TestCase):
    def test_install(self):
        # Get install directory.
        install_dir = install_test_helper.get_install_dir()
        # Override PYTHONPATH to only use the installed `pydrake` module.
        env_python_path = "PYTHONPATH"
        tool_env = dict(os.environ)
        tool_env[env_python_path] = \
            install_test_helper.get_python_site_packages_dir(install_dir)
        # Ensure we can import all user-visible modules.
        script = "import pydrake.all"
        install_test_helper.check_call(
            [install_test_helper.get_python_executable(), "-c", script],
            env=tool_env
        )

    def test_type_stubs(self):
        # Get pydrake directory.
        install_dir = install_test_helper.get_install_dir()
        site_packages = \
            install_test_helper.get_python_site_packages_dir(install_dir)
        # Ensure some files are present.
        self.assertTrue(
            os.path.exists(os.path.join(site_packages, 'pydrake', 'py.typed')))
        self.assertTrue(
            os.path.exists(os.path.join(site_packages, 'pydrake', 'lcm.pyi')))


if __name__ == '__main__':
    unittest.main()
