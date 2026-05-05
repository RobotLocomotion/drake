from collections.abc import Callable
import logging
import os
from pathlib import Path
import pickle
import re
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest

from python import runfiles

EXPECTED_BAZELISK = "1.28.1"
EXPECTED_KCOV = "43+dfsg-1"


class InstallPrereqsActor:
    def __init__(self, *, test_case):
        self._test_case = test_case

        # Create a scratch directory for ourselves.
        test_tmpdir = Path(os.environ["TEST_TMPDIR"])
        self._temp_dir_object = tempfile.TemporaryDirectory(dir=test_tmpdir)
        base = Path(self._temp_dir_object.name)

        # Create `source`, which will contain install_prereqs and its helper
        # files. (We can't run it directly from runfiles, because it writes
        # back into the source tree in some cases and we don't want pollution
        # across putatively independent test cases.)
        self._source = base / "source"
        self._source.mkdir()
        self._script = self._set_up_source()

        # Create `cwd`, which will be the (empty) current working directory
        # while running install_prereqs.
        self._cwd = base / "cwd"
        self._cwd.mkdir()

        # Create `path` which will be the only directory on the $PATH.
        self._path = base / "path"
        self._path.mkdir()

        # Create `io` which will be used by stubby to communicate with us.
        self._io = base / "io"
        self._io.mkdir()

        # Create a stub program that inter-operates with expect_call() to mock
        # up subprocess calls made by install_prereqs. The protocol is that
        # stubby writes its argv.pkl and then waits for a result.pkl response,
        # and expect_call() waits for argv.pkl and then writes the result.pkl.
        # Each one deletes the file after reading it (in case there's more than
        # one subprocess call in a row).
        #
        # N.B. We are using the host OS interpreter (/usr/bin/python3), even on
        # macOS where that's a very old version Python. Still, it's sufficient
        # for this small stub. Using `sys.executable` encountered inexplicable
        # problems in CI.
        self._stubby = self._path / ".stubby"
        self._stubby.write_text(
            encoding="utf-8",
            data=textwrap.dedent(f"""\
            #!/usr/bin/python3
            import pathlib, pickle, sys, time
            io = pathlib.Path("{self._io}")
            (io / "argv.pkl").write_bytes(pickle.dumps(sys.argv))
            for _ in range(100):
              try:
                result = pickle.loads((io / "result.pkl").read_bytes())
                break
              except Exception:
                time.sleep(0.1)
            else:
              raise TimeoutError()
            (io / "result.pkl").unlink()
            sys.stdout.write(result["stdout"])
            sys.stdout.flush()
            sys.exit(result["returncode"])
            """),
        )
        self._stubby.chmod(0o777)

        # Establish the allowed list of commands the DUT can run.
        # Tests can use add_to_path() and remove_from_path() to fine-tune this.
        allowed = [
            "bazel",
            "dpkg",
            "dpkg-query",
            "sudo",
        ]
        for program in allowed:
            self.add_to_path(program)

        # Don't launch anything until the caller does start().
        self._process = None
        self.returncode = None
        self.stdout = None

        # Track whether the setup program has performed these actions yet.
        self._did_sudo_check = False

        # The list of currently-installed packages to report to install_prereqs;
        # a mapping of name => version number.
        self.installed_packages = {}

    def _set_up_source(self) -> Path:
        """Prepares a source-tree-like writable temporary directory that
        contains the install_prereqs script and its data dependencies.

        This used by our __init__ function to populate the empty self._source
        directory with the necessary symlinks.

        Returns the path to install_prereqs inside self._source.
        """
        assert self._source.exists()
        assert len(list(self._source.iterdir())) == 0
        manifest = runfiles.Create()
        install_prereqs = Path(
            manifest.Rlocation("drake/setup/install_prereqs.py")
        )

        # When running install_prereqs from the source tree, all of the
        # platform-specific data files are available, and the directory
        # the script resides in a directory named "setup".
        #
        # TODO(jwnimmer-tri) Add support for testing install_prereqs for an
        # installed version of Drake (the "from_binary" workflow).
        setup = self._source / "setup"
        setup.mkdir()
        result = self._source / "setup/install_prereqs.py"
        result.symlink_to(install_prereqs)
        for platform in ["mac", "ubuntu"]:
            (setup / platform).symlink_to(install_prereqs.parent / platform)
        return result

    def source(self) -> Path:
        """Returns the root of the mocked-up source tree."""
        return self._source

    def add_to_path(self, program: str):
        (self._path / program).symlink_to(self._stubby)

    def remove_from_path(self, program: str):
        (self._path / program).unlink()

    def start(self, *, args):
        """Launches install_prereqs as a subprocess, with the given `args`.
        Returns `self` to allow easy chaining. Test cases should call `finish()`
        to wait for the finished process to complete.
        """
        assert self._process is None
        logging.info(f"Running install_prereqs with {args} ...")
        full_args = [sys.executable, self._script] + args
        env = {
            "PATH": str(self._path),
        }
        self._process = subprocess.Popen(
            args=full_args,
            cwd=self._cwd,
            encoding="utf-8",
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        return self

    def finish(self):
        """Waits for the install_prereqs subprocess started by `start()` to
        finish. Does NOT fail on a non-zero returncode; test cases should check
        the saved returncode explicitly.
        """
        assert self._process is not None
        try:
            self._process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            logging.warn("install_prereqs timeout expired")
            self._process.terminate()
        self._process.wait()
        self.stdout = self._process.stdout.read()
        for line in self.stdout.splitlines():
            logging.info(f" [stdout] {line}")
        self.returncode = self._process.returncode

    def expect_call(
        self,
        expected_argv: list[str],
        *,
        stdout: str | Callable[[list[str]], str] = "",
        returncode: int = 0,
    ) -> list[str]:
        """Between `start()` and `finish()`, waits for install_prereqs to call
        out to a subprocess and mocks up the effects of that call.

        The expected command line is given by `expected_argv`; the first element
        is the command name. The actual arguments passed by install_prereqs must
        match `expected_argv` with one exception: if the last item in is "...",
        then only the arguments prior to that must match.

        The mocked call will print the given `stdout` content, which can either
        be a `str` or a callable that is given the argv and returns a `str`.

        The mocked call will exit with the given `returncode`.

        This method returns the mocked call's actual argv.
        """
        # Print now in case we get stuck.
        command = expected_argv[0]
        if command == "sudo" and expected_argv[1][0] != "-":
            command = " ".join(expected_argv[:2])
        logging.info(f"Waiting for subprocess call to {command} ...")

        # Wait for the "stubby" subprocess to dump its argv.
        for _ in range(100):
            self._process.poll()
            if self._process.returncode is not None:
                self.finish()
                self._test_case.fail("install_prereqs terminated unexpectedly")
            try:
                actual_argv = pickle.loads((self._io / "argv.pkl").read_bytes())
                break
            except Exception:
                time.sleep(0.1)
        else:
            raise TimeoutError()
        (self._io / "argv.pkl").unlink()

        # Compute stdout if necessary.
        if callable(stdout):
            stdout = stdout(actual_argv)

        # Tell stubby what to do.
        result = dict(
            stdout=stdout,
            returncode=returncode,
        )
        (self._io / "result.pkl").write_bytes(pickle.dumps(result))

        # Strip the useless directory name off of the actual command.
        actual_argv[0] = actual_argv[0].split("/")[-1]

        # Validate the called program and its arguments.
        if expected_argv[-1] == "...":
            expected_prefix = expected_argv[:-1]
            actual_prefix = actual_argv[: len(expected_prefix)]
            self._test_case.assertEqual(actual_prefix, expected_prefix)
        else:
            self._test_case.assertEqual(actual_argv, expected_argv)

        return actual_argv

    def expect_sudo_check_if_not_yet_checked(self):
        if self._did_sudo_check:
            return
        self.expect_call(["sudo", "-n", "/bin/true"])
        self._did_sudo_check = True

    def expect_dpkg_query(self):
        def _reply(argv):
            stdout = ""
            for arg in argv[1:]:
                if arg.startswith("-"):
                    # Skip over flags.
                    continue
                if arg in self.installed_packages:
                    version = self.installed_packages[arg]
                    stdout += f"{arg} ii {version}\n"
            return stdout

        self.expect_call(["dpkg-query", "..."], stdout=_reply)

    def expect_apt_install(self):
        self.expect_sudo_check_if_not_yet_checked()
        argv = self.expect_call(["sudo", "apt-get", "install", "..."])
        package_names = [arg for arg in argv[3:] if not arg.startswith("-")]
        return package_names


class InstallPrereqsTest(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None
        logging.info(f"\n\n=== Running {self.id()} === ")

    def test_help(self):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--help"]).finish()
        self.assertRegex(dut.stdout, "usage: install_prereqs")
        self.assertEqual(dut.returncode, 0)

    def test_user_environment_only(self):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--user-environment-only"]).finish()
        self.assertRegex(dut.stdout, "Writing.*gen/python_version.txt")
        self.assertRegex(dut.stdout, "Writing.*gen/environment.bazelrc")
        self.assertTrue((dut.source() / "gen/python_version.txt").exists())
        self.assertTrue((dut.source() / "gen/environment.bazelrc").exists())
        self.assertEqual(dut.returncode, 0)

    def test_developer_bootstrap(self):
        """Check --developer with nothing installed yet."""
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--developer", "-y"])

        if sys.platform != "darwin":
            # The DUT should install bazelisk and maybe kcov (after confirming
            # that they are missing).
            dut.expect_dpkg_query()
            paths = dut.expect_apt_install()
            filenames = sorted([x.split("/")[-1] for x in paths])
            names = set([re.split("[-_]", x)[0] for x in filenames])
            self.assertIn(names, ({"bazelisk"}, {"bazelisk", "kcov"}))

        # The DUT prefetches bazel.
        dut.expect_call(["bazel", "version"])

        dut.finish()
        self.assertRegex(dut.stdout, "Writing.*gen/python_version.txt")
        self.assertRegex(dut.stdout, "Writing.*gen/environment.bazelrc")
        self.assertRegex(dut.stdout, "Pre-fetching bazel")
        self.assertTrue((dut.source() / "gen/python_version.txt").exists())
        self.assertTrue((dut.source() / "gen/environment.bazelrc").exists())
        self.assertEqual(dut.returncode, 0)

    def test_developer_bump(self):
        """Check --developer with some things already installed, but at too-old
        versions."""
        dut = InstallPrereqsActor(test_case=self)
        dut.installed_packages = {
            "bazelisk": "0.0.0",
            "kcov": EXPECTED_KCOV,
        }
        dut.start(args=["--developer", "-y"])

        if sys.platform != "darwin":
            # The DUT should install bazelisk (after confirming the current
            # version is too old).
            dut.expect_dpkg_query()
            dut.expect_call(
                ["dpkg", "--compare-versions", "..."],
                returncode=1,
            )
            paths = dut.expect_apt_install()
            self.assertEqual(len(paths), 1)
            name = paths[0].split("/")[-1].split("-")[0]
            self.assertEqual(name, "bazelisk")

        # The DUT prefetches bazel.
        dut.expect_call(["bazel", "version"])

        dut.finish()
        self.assertRegex(dut.stdout, "Writing.*gen/python_version.txt")
        self.assertRegex(dut.stdout, "Writing.*gen/environment.bazelrc")
        self.assertRegex(dut.stdout, "Pre-fetching bazel")
        self.assertTrue((dut.source() / "gen/python_version.txt").exists())
        self.assertTrue((dut.source() / "gen/environment.bazelrc").exists())
        self.assertEqual(dut.returncode, 0)

    def test_developer_completed(self):
        """Check --developer when everything is already installed (as if a
        prior run had already succeeded)."""
        dut = InstallPrereqsActor(test_case=self)
        dut.installed_packages = {
            "bazelisk": EXPECTED_BAZELISK,
            "kcov": EXPECTED_KCOV,
        }
        dut.start(args=["--developer", "-y"])

        if sys.platform != "darwin":
            # The DUT confirms that bazelisk (etc) is already installed.
            dut.expect_dpkg_query()

        # The DUT prefetches bazel.
        dut.expect_call(["bazel", "version"])

        dut.finish()
        self.assertRegex(dut.stdout, "Writing.*gen/python_version.txt")
        self.assertRegex(dut.stdout, "Writing.*gen/environment.bazelrc")
        self.assertRegex(dut.stdout, "Pre-fetching bazel")
        self.assertTrue((dut.source() / "gen/python_version.txt").exists())
        self.assertTrue((dut.source() / "gen/environment.bazelrc").exists())
        self.assertEqual(dut.returncode, 0)


logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
