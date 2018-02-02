from __future__ import absolute_import

import os
import shutil
import subprocess


def install(installation_folder="tmp", installed_subfolders=[],
            rmdir_cwd=True):
    """Install into a temporary directory.

    Runs install script to install target in the specified temporary
    directory. The directory does not need to be removed as bazel tests are run
    in a scratch space. All build artifacts are removed from the scratch space,
    leaving only the install directory. If `installed_subfolders` are not found
    in installation directory, a string containing an error message is
    returned.
    """
    os.mkdir(installation_folder)
    # Install target and its dependencies in scratch space.
    subprocess.check_call(
        ["install",
         os.path.abspath(installation_folder)]
        )
    content_install_folder = os.listdir(installation_folder)
    # Check that all expected folders are installed
    for f in installed_subfolders:
        if f not in content_install_folder:
            return str(f) + " not found in " + str(content_install_folder)
    # Skip the "remove Bazel build artifacts" when asked.
    if not rmdir_cwd:
        return None
    # Remove Bazel build artifacts, and ensure that we only have install
    # artifacts.
    content_test_folder = os.listdir(os.getcwd())
    content_test_folder.remove('tmp')
    for element in content_test_folder:
        if os.path.isdir(element):
            shutil.rmtree(element)
        else:
            os.remove(element)
    content_install_folder = os.listdir(".")
    if content_install_folder != [installation_folder]:
        return ("Bazel build artifact not removed in test folder: "
                + str(content_install_folder))
    return None
