from __future__ import absolute_import, division, print_function

import os
import shutil
import subprocess
import unittest
import sys
import pydrake.common


class TestCommon(unittest.TestCase):
    def testDrakeDemandThrows(self):
        # Drake's assertion errors should turn into SystemExit by default,
        # without the user needing to do anything special.  Here, we trigger a
        # C++ assertion failure from Python and confirm that an exception with
        # an appropriate type and message comes out.
        try:
            pydrake.common.trigger_an_assertion_failure()
            self.fail("Did not get a SystemExit")
        except SystemExit as e:
            self.assertTrue(e.code is not None)
            self.assertRegexpMatches(
                e.message,
                ".*".join([
                    "Failure at ",
                    " trigger_an_assertion_failure",
                    " condition 'false' failed",
                ]))

    def testDrakeFindResourceOrThrow(self):
        pydrake.common.FindResourceOrThrow(
            'drake/examples/atlas/urdf/atlas_convex_hull.urdf'
            )

    def test_install(self):
        # Install into a temporary directory. The temporary directory does not
        # need to be removed as bazel tests are run in a scratch space.
        tmp_folder = "tmp"
        os.mkdir(tmp_folder)
        # Install the bindings and its dependencies in scratch space.
        subprocess.check_call(
            ["drake/bindings/install",
             os.path.abspath(tmp_folder),
             ])
        # Set Python env
        # Remove the un-installed copy, so we _know_ it won't be used.
        shutil.rmtree("drake")
        os.remove(".drake-resource-sentinel")
        shutil.rmtree("_solib_k8")
        shutil.rmtree("external")
        self.assertEqual(os.listdir("."), [tmp_folder])

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
        os.mkdir(data_folder)
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
