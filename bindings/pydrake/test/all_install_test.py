"""
Ensures we can import `pydrake.all` from install.
"""

import os
import subprocess
import sys
import unittest

import install_test_helper


class TestAllInstall(unittest.TestCase):
    def test_install(self):
        # Install into a temporary directory.
        tmp_folder = "tmp"
        result = install_test_helper.install(tmp_folder, ['lib'])
        self.assertEqual(None, result)
        # Override PYTHONPATH to only use the installed `pydrake` module.
        env_python_path = "PYTHONPATH"
        tool_env = dict(os.environ)
        tool_env[env_python_path] = os.path.abspath(
            os.path.join(tmp_folder, "lib", "python2.7", "site-packages")
        )
        # Ensure we can import all user-visible modules.
        script = "import pydrake.all"
        subprocess.check_call([sys.executable, "-c", script], env=tool_env)


if __name__ == '__main__':
    unittest.main()
