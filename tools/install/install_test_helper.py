import errno
import os
import signal
import stat
import subprocess
import sys
import time


def _make_read_only(path):
    current = stat.S_IMODE(os.lstat(path).st_mode)
    os.chmod(path, current & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))


def install():
    """Install into a read-only temporary directory.

    Runs install script to install target in the specified temporary
    directory. The directory does not need to be removed as bazel tests are run
    in a scratch space. All build artifacts are removed from the scratch space,
    leaving only the install directory. The install directory is made read-only
    after installation process is completed.
    """
    assert "TEST_TMPDIR" in os.environ, (
        "This may only be run from within `bazel test`")
    # Install into a tmpdir.
    installation_folder = get_install_dir()
    # The following will fail if `install()` is called multiple times, which it
    # should not be anyway.
    os.makedirs(installation_folder)
    assert os.path.exists(installation_folder)
    # Install target and its dependencies in scratch space.
    subprocess.check_call(["install", installation_folder])
    # Change permissions to remove write access.
    for root, dirs, files in os.walk(installation_folder):
        for d in dirs:
            _make_read_only(os.path.join(root, d))
        for f in files:
            _make_read_only(os.path.join(root, f))


def get_install_dir():
    """Returns install directory.

    When running tests in the install tree, the current working directory
    needs to be changed (e.g. to '/' to avoid finding artifacts from the
    build tree when testing the installation). The caveat is that Python
    scripts that run install tests may need to know the install directory.
    `os.environ['TEST_TMPDIR']` is the absolute path to a private writable
    directory. This function returns the path to this writable folder appended
    with 'installation'. This allows to use this writable folder to create
    additional files without modifying the install tree.
    """
    return os.path.join(os.environ['TEST_TMPDIR'], 'installation')


def create_temporary_dir(name='tmp'):
    """Creates temporary directory and returns its path.

    When running tests in the install tree, a temporary folder is created
    by bazel and its path is accessible by getting the value of
    `os.environ['TEST_TMPDIR']`. However, in the install_test_helper framework,
    this folder is also used to install `drake`, so this function creates a
    subdirectory nested inside `os.environ['TEST_TMPDIR']`.
    """
    tmp_dir = os.path.join(os.environ['TEST_TMPDIR'], name)
    os.mkdir(tmp_dir)
    return tmp_dir


def get_python_executable():
    """Use appropriate Python executable

    Call python3.9 on macOS to force using the Python executable from the
    python@3.9 formula. Calling a different Python executable would result in a
    crash since pydrake was not built against the Python library corresponding
    to that executable. On other systems, it will just fall back to the current
    Python executable.
    """
    if sys.platform == "darwin":
        return "python3.9"
    else:
        return sys.executable


def run_and_kill(cmd, timeout=2.0, from_install_dir=True):
    """Convenient function to start a command and kill it automatically.

    This function starts a given command and kills it after the given
    `timeout`. This is useful if one needs to test a command that doesn't
    terminate on its own. The current working directory is set to '/' when
    running the given command.

    `cmd` is a list of a command and its arguments such as what is expected
    by `subprocess.Popen` (See `subprocess` documentation).

    If `from_install_dir` is True (default), the first argument of the command
    line is prepended with the install directory (found with
    `get_install_dir()`).
    """
    if from_install_dir:
        cmd[0] = os.path.join(get_install_dir(), cmd[0])
    env = os.environ
    proc = subprocess.Popen(cmd, cwd='/', env=env)
    start = time.time()
    while time.time() - start < timeout:
        time.sleep(0.5)
        ret = proc.poll()
        assert ret is None
    # Time's up: kill the proc (and it's group):
    proc.terminate()
    assert proc.wait() == -signal.SIGTERM


def check_call(args, *extra_args, **kwargs):
    """Helper function for `subprocess.check_call()`.

    Install tests should use this function instead of
    `subprocess.check_call()`. This function is a simple helper function that
    calls `subprocess.check_call()` and updates the current working directory
    to `/`.
    """
    if args[0].endswith('.py'):
        # Ensure that we test with the same Python version that Bazel is using.
        args = [get_python_executable()] + args
    if 'env' in kwargs:
        env = kwargs.pop('env')
    else:
        env = os.environ
    return subprocess.check_call(args, cwd='/', env=env, *extra_args, **kwargs)


def check_output(*args, **kwargs):
    """Helper function for `subprocess.check_output()`.

    Install tests should use this function instead of
    `subprocess.check_output()`. This function is a simple helper function that
    calls `subprocess.check_output()` and updates the current working directory
    to `/`.
    """
    if 'env' in kwargs:
        env = kwargs.pop('env')
    else:
        env = os.environ
    return subprocess.check_output(
        cwd='/', env=env, *args, **kwargs).decode('utf8')


def get_python_site_packages_dir(install_dir):
    return os.path.abspath(
        os.path.join(
            install_dir, "lib",
            "python{}.{}".format(
                sys.version_info.major, sys.version_info.minor),
            "site-packages")
    )
