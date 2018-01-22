#!/usr/bin/env python

import os
import subprocess
import install_test_helper


def testDrakeFindResourceOrThrowInInstall():
    # Set the correct PYTHONPATH to use the correct pydrake module.
    env_python_path = "PYTHONPATH"
    tool_env = dict(os.environ)
    print install_test_helper.get_install_dir()
    tool_env[env_python_path] = os.path.abspath(
        os.path.join(install_test_helper.get_install_dir(),
        "lib", "python2.7", "site-packages")
    )
    data_folder = os.path.join(install_test_helper.get_install_dir(), "share", "drake")
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
        env=tool_env,
        ).strip()
    found_install_path = (data_folder in output_path)
    assert found_install_path == True
    
if __name__ == '__main__':
    testDrakeFindResourceOrThrowInInstall()
