from __future__ import absolute_import

import os
import signal
import subprocess
import time


def install(installation_folder="tmp"):
    """Install into a temporary directory.

    Runs install script to install target in the specified temporary
    directory. The directory does not need to be removed as bazel tests are run
    in a scratch space. All build artifacts are removed from the scratch space,
    leaving only the install directory.
    """
    os.mkdir(installation_folder)
    # Install target and its dependencies in scratch space.
    subprocess.check_call(
        ["install",
         os.path.abspath(installation_folder)]
        )


def get_install_dir():
    """ Returns install directory specified in environment variable.

    When running tests of in the install tree, the current working directory
    needs to be changed (e.g. to '/' to avoid finding artifacts from the
    build tree when testing the installation. The caveat is that Python
    scripts that run install tests may need to know the install directory.
    To keep that information, the install directory is saved in the environment
    variable `TESTINSTALLPATH` and can be easily recovered with a call of this
    function.
    """
    return os.path.abspath(os.environ['TESTINSTALLPATH'])


def runAndKill(cmd, timeout=2.0):
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
    try:
        os.kill(proc.pid, signal.SIGTERM)
    except OSError as e:
        assert e.strerror == ""
    assert proc.wait() == -signal.SIGTERM
