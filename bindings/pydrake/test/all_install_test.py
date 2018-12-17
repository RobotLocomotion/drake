"""
Ensures we can import `pydrake.all` from install.
"""

from __future__ import print_function
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
        tool_env = dict()
        tool_env[env_python_path] = \
            install_test_helper.get_python_site_packages_dir(install_dir)
        print(tool_env, file=sys.stderr)
        # Ensure we can import all user-visible modules.
        script = "import pydrake.all"
        install_test_helper.check_call(
            [install_test_helper.get_python_executable(), "-c", script],
            env=tool_env
        )


if __name__ == '__main__':
    unittest.main()
