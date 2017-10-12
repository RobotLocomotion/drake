from __future__ import absolute_import, division, print_function

import os
import shutil
import subprocess
import unittest
import sys


class TestCommonInstall(unittest.TestCase):
    def testDrakeFindResourceOrThrowInInstall(self):
        # Install into a temporary directory. The temporary directory does not
        # need to be removed as bazel tests are run in a scratch space.
        tmp_folder = "tmp"
        os.mkdir(tmp_folder)
        # Install the bindings and its dependencies in scratch space.
        subprocess.check_call(
            ["drake/bindings/install",
             os.path.abspath(tmp_folder),
             ])
        self.assertEqual(os.listdir(tmp_folder),
                         ['libexec', 'lib', 'include',
                         'share', 'bin', 'plugins'])
        # Remove the un-installed copy, so we _know_ it won't be used.
        shutil.rmtree("drake")
        os.remove(".drake-resource-sentinel")
        shutil.rmtree("_solib_k8")
        self.assertEqual(os.listdir("."), [tmp_folder, 'external'])

        # Set the correct PYTHONPATH and (DY)LD_LIBRARY_PATH to use the
        # correct pydrake module.
        env_python_path = "PYTHONPATH"
        if sys.platform == 'darwin':
            env_ld_library = "DYLD_LIBRARY_PATH"
        else:
            env_ld_library = "LD_LIBRARY_PATH"
        tool_env = dict(os.environ)
        tool_env[env_python_path] = os.path.abspath(
            os.path.join(tmp_folder, "lib", "python2.7", "site-packages")
        )
        tool_env[env_ld_library] = os.path.abspath(
            os.path.join(tmp_folder, "lib")
        )
        # Create empty directory since no data is installed. Otherwise
        # `GetDrakePath()` will throw an exception complaining that no
        # directory was found. This is safe as:
        # 1) if the first part of the test (installation fails), an error will
        # be thrown when trying to create this folder.
        # 2) This empty folder does not change the behavior of
        # `GetDrakePath()`; We still expect `GetDrakePath()` to find the
        # sentinel (which is installed) but simply work around the fact that
        # no data is installed (which data should be installed anyway?). The
        # test will fail if the sentinel is not found in the expected location.
        data_folder = os.path.join(tmp_folder, "share", "drake", "drake")
        output_path = subprocess.check_output(
            ["python",
             "-c", "import pydrake; print(pydrake.getDrakePath())"
             ],
            env=tool_env,
            ).strip()
        found_install_path = (data_folder in output_path)
        self.assertEqual(found_install_path, True)


if __name__ == '__main__':
    unittest.main()
