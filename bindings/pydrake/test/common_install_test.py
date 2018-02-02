from __future__ import absolute_import, division, print_function

import os
import shutil
import subprocess
import unittest
import sys
import install_test_helper


class TestCommonInstall(unittest.TestCase):
    def testDrakeFindResourceOrThrowInInstall(self):
        # Install into a temporary directory. The temporary directory does not
        # need to be removed as bazel tests are run in a scratch space.
        tmp_folder = "tmp"
        result = install_test_helper.install(tmp_folder, ['lib', 'share'])
        self.assertEqual(None, result)
        # Set the correct PYTHONPATH to use the correct pydrake module.
        env_python_path = "PYTHONPATH"
        tool_env = dict(os.environ)
        tool_env[env_python_path] = os.path.abspath(
            os.path.join(tmp_folder, "lib", "python2.7", "site-packages")
        )
        data_folder = os.path.join(tmp_folder, "share", "drake")
        # Call python2 to force using python brew install. Calling python
        # system would result in a crash since pydrake was built against
        # brew python. On Linux this will still work and force using
        # python2 which should be a link to the actual executable.
        # Calling `pydrake.getDrakePath()` twice verifies that there
        # is no memory allocation issue in the C code.
        output_path = subprocess.check_output(
            ["python2",
             "-c", "import pydrake; print(pydrake.getDrakePath());\
             import pydrake; print(pydrake.getDrakePath())"
             ],
            cwd='/',  # Defeat the "search in parent folders" heuristic.
            env=tool_env,
            ).strip()
        found_install_path = (data_folder in output_path)
        self.assertEqual(found_install_path, True)


if __name__ == '__main__':
    unittest.main()
