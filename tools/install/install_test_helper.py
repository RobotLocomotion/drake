import os
import signal
import subprocess
import sys
import time


def install():
    """Install into a temporary directory.

    Runs install script to install target in the specified temporary
    directory. The directory does not need to be removed as bazel tests are run
    in a scratch space. All build artifacts are removed from the scratch space,
    leaving only the install directory.
    """
    assert "TEST_TMPDIR" in os.environ, (
        "This may only be run from within `bazel test`")
    # Install into the tmpdir that Bazel has created for us.
    installation_folder = get_install_dir()
    assert os.path.exists(installation_folder) is True
    # Install target and its dependencies in scratch space.
    subprocess.check_call(["install", installation_folder])


def get_install_dir():
    """Returns install directory specified in environment variable.

    When running tests of in the install tree, the current working directory
    needs to be changed (e.g. to '/' to avoid finding artifacts from the
    build tree when testing the installation). The caveat is that Python
    scripts that run install tests may need to know the install directory.
    Since within the tests, installation is always in `TEST_TMPDIR`, which
    is the absolute path to a private writable directory, this function returns
    the path to that folder.
    """
    return os.environ['TEST_TMPDIR']


def get_python_executable():
    """Use appropriate Python executable

    Call python2 on MacOS to force using python brew install. Calling python
    system would result in a crash since pydrake was built against brew python.
    On other systems, it will just fall-back to the current Python executable.
    """
    if sys.platform == "darwin":
        return "python2"
    else:
        return sys.executable


def run_and_kill(cmd, timeout=2.0):
    """Convenient function to start a command and kill it automatically.

    This function starts a given command and kills it after the given
    `timeout`. This is useful if one needs to test a command that doesn't
    terminate on its own.

    `cmd` is a list of a command and its arguments such as what is expected
    by `subprocess.Popen` (See `subprocess` documentation).
    """
    cmd[0] = os.path.join(get_install_dir(), cmd[0])
    proc = subprocess.Popen(cmd)
    start = time.time()
    while time.time() - start < timeout:
        time.sleep(0.5)
        ret = proc.poll()
        assert ret is None
    # time's up: kill the proc (and it's group):
    proc.terminate()
    assert proc.wait() == -signal.SIGTERM
