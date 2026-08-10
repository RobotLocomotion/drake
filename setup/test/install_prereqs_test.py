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

EXPECTED_BAZELISK = "1.29.0"
EXPECTED_KCOV = "43+dfsg-1"


class InstallPrereqsActor:
    def __init__(self, *, test_case, tree="source_tree"):
        self._test_case = test_case
        assert tree in ["source_tree", "install_tree"]
        self._tree = tree

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
            "locale",
            "lsb_release",
            "python3",
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
        self._did_apt_update = False

        # The list of currently-installed packages to report to install_prereqs;
        # a mapping of name => version number.
        self.installed_packages = {}

    def _set_up_source(self) -> Path:
        """Prepares a source-tree-like or install-tree-like writable temporary
        directory that contains the install_prereqs script and its data
        dependencies. The self._tree mode is used to select either "source_tree"
        or "install_tree"

        This used by our __init__ function to populate the empty self._source
        directory with the necessary symlinks.

        Returns the path to install_prereqs inside self._source.
        """
        assert self._source.exists()
        assert len(list(self._source.iterdir())) == 0
        assert self._tree in ["source_tree", "install_tree"]
        manifest = runfiles.Create()
        install_prereqs = Path(
            manifest.Rlocation("drake/setup/install_prereqs.py")
        )

        if self._tree == "source_tree":
            # When running install_prereqs from the source tree, all of the
            # platform-specific data files are available, and the directory
            # the script resides in a directory named "setup".
            setup = self._source / "setup"
            setup.mkdir()
            result = self._source / "setup/install_prereqs.py"
            result.symlink_to(install_prereqs)
            for platform in ["mac", "ubuntu"]:
                (setup / platform).symlink_to(install_prereqs.parent / platform)
            return result
        else:
            assert self._tree == "install_tree"
            # When running install_prereqs from the install tree, only a subset
            # of platform-specific data files are available, and the script and
            # its data files live side by side in a directory named "share".
            share = self._source / "share"
            share.mkdir()
            result = self._source / "share/install_prereqs"
            result.symlink_to(install_prereqs)
            # TODO(jwnimmer-tri) De-duplicate this list vs our BUILD.bazel.
            # We can probably literally call the ":install" program here?
            data = [
                "ubuntu/packages-jammy-binary.txt",
                "ubuntu/packages-noble-binary.txt",
            ]
            (share / "ubuntu").mkdir()
            for datum in data:
                path = Path(manifest.Rlocation(f"drake/setup/{datum}"))
                (share / datum).symlink_to(path)
            return result

    def source(self) -> Path:
        """Returns the root of the mocked-up source tree."""
        return self._source

    def add_to_path(self, program):
        if program == "python3":
            (self._path / program).symlink_to(sys.executable)
        else:
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
        match `expected_argv` with one exception: if the last item is "...",
        then only the arguments prior to that must match.

        The mocked call will print the given `stdout` content, which can either
        be a `str` or a callable that is given the argv and returns a `str`.

        The mocked call will exit with the given `returncode`.

        This method returns the mocked call's actual argv.
        """
        # Print now in case we get stuck.
        description = expected_argv[0]
        if description == "sudo" and expected_argv[1][0] != "-":
            description = " ".join(expected_argv[:2])
        logging.info(f"Waiting for subprocess call to {description} ...")

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

        # Compute stdout now, if necessary.
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

        self.expect_call(["dpkg-query", "--show", "..."], stdout=_reply)

    def expect_apt_update_if_not_yet_updated(self):
        if self._did_apt_update:
            return
        self.expect_sudo_check_if_not_yet_checked()
        self.expect_call(exact=["sudo", "apt-get", "update"])
        self._did_apt_update = True

    def expect_apt_install(self):
        self.expect_sudo_check_if_not_yet_checked()
        argv = self.expect_call(
            ["sudo", "apt-get", "install", "--no-install-recommends", "..."]
        )
        package_names = [arg for arg in argv[3:] if not arg.startswith("-")]
        return package_names


class InstallPrereqsTest(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None
        logging.info(f"\n\n=== Running {self.id()} === ")

        # In the test cases below, we'll check that prereqs flavors install the
        # correct number of packages from apt as a cross-check that the setup
        # code is probably correct, without overly coupling the test cases to
        # the specific package names of our dependencies. The tallies here are
        # based on the "noble" txt files, so any time you change those files
        # you'll also need to update these constants. The important point is
        # that the tallies are monotonically increasing.
        self._expected_num_apt_packages = {
            "binary": 31,
            "build": 51,
            "developer": 71,
        }

    def _check_stdout_match(self, *, dut: InstallPrereqsActor, expected: list):
        actual = dut.stdout.splitlines()
        expected = [line for line in expected if line is not None]
        for i in range(len(expected)):
            expected_line = expected[i]
            actual_line = actual[i]
            if isinstance(expected_line, re.Pattern):
                self.assertRegex(actual_line, expected_line)
            else:
                self.assertEqual(actual_line, expected_line)
        self.assertEqual(len(actual), len(expected))

    def test_help(self):
        for tree in ["source_tree", "install_tree"]:
            with self.subTest(tree=tree):
                dut = InstallPrereqsActor(test_case=self, tree=tree)
                dut.start(args=["--help"]).finish()
                self.assertIn("usage: install_prereqs", dut.stdout)
                self.assertEqual(dut.returncode, 0)

    def test_user_environment_only(self):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--user-environment-only"]).finish()
        self.assertRegex(dut.stdout, "Writing.*gen/python_version.txt")
        self.assertRegex(dut.stdout, "Writing.*gen/environment.bazelrc")
        self.assertTrue((dut.source() / "gen/python_version.txt").exists())
        self.assertTrue((dut.source() / "gen/environment.bazelrc").exists())
        self.assertEqual(dut.returncode, 0)

    @unittest.skipIf(sys.platform == "darwin", "Ubuntu only")
    def test_ubuntu_binary(self):
        for has_lsb_release in [False, True]:
            for already_installed in [False, True]:
                if already_installed and not has_lsb_release:
                    # Skip over a silly permutation.
                    continue
                for yes in [False, True]:
                    kwargs = dict(
                        has_lsb_release=has_lsb_release,
                        already_installed=already_installed,
                        yes=yes,
                    )
                    with self.subTest(**kwargs):
                        self._check_ubuntu_binary(**kwargs)

    def _check_ubuntu_binary(
        self, *, has_lsb_release: bool, already_installed: bool, yes: bool
    ):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--binary"] + (["-y"] if yes else []))

        # The DUT first checks which platform it's running on.
        # If lsb-release is not installed yet, then the DUT should install it.
        if not has_lsb_release:
            dut.remove_from_path("lsb_release")
            self.assertListEqual(dut.expect_apt_install(), ["lsb-release"])
            dut.add_to_path("lsb_release")
        dut.expect_lsb_release()

        # The DUT asks what's already installed.
        installed_packages = []
        if already_installed:
            installed_packages = [
                "build-essential",
                "python3",
            ]
        dut.expect_call(
            prefix=["dpkg-query"],
            stdout="".join(
                [f"{name} ii someversion\n" for name in installed_packages]
            ),
        )

        # The DUT will install any missing packages.
        package_names = dut.expect_apt_install()

        # If this check fails, see the comment in our setUp() method.
        self.assertEqual(
            len(package_names) + len(installed_packages),
            self._expected_num_apt_packages["binary"],
        )

        # Nothing else should happen.
        dut.finish()
        self._check_stdout_match(
            dut=dut,
            expected=[
                "INFO: Running: sudo apt-get update ...",
                (
                    re.compile(".*apt-get install.*lsb-release ...")
                    if not has_lsb_release
                    else None
                ),
                re.compile("INFO: Running: sudo apt-get install.*"),
                "INFO: Successfully installed --flavor=binary prereqs.",
            ],
        )
        self.assertEqual(dut.returncode, 0)

        # Cross-check our -y request vs the actual commands run.
        if yes or not has_lsb_release:
            self.assertRegex(dut.stdout, "apt-get install.*--yes")
        else:
            self.assertNotIn("--yes", dut.stdout)

    @unittest.skipIf(sys.platform == "darwin", "Ubuntu only")
    def test_ubuntu_build(self):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--build", "-y"])

        # The DUT first checks which platform it's running on.
        dut.expect_lsb_release()

        # The DUT asks what's already installed. Tell it nothing in reply.
        # Therefore, the DUT will install all packages.
        dut.expect_call(prefix=["dpkg-query"])
        package_names = dut.expect_apt_install()

        # If this check fails, see the comment in our setUp() method.
        self.assertEqual(
            len(package_names), self._expected_num_apt_packages["build"]
        )

        # The DUT asks which locales exist. Tell it nothing in reply.
        # Therefore, the DUT will geneate the locale.
        dut.expect_call(exact=["locale", "-a"])
        dut.expect_call(exact=["sudo", "locale-gen", "en_US.utf8"])

        # Nothing else should happen.
        dut.finish()
        self.assertIn("Successfully installed --flavor=build", dut.stdout)
        self.assertEqual(dut.returncode, 0)

    @staticmethod
    def _extract_debian_package_names_from_paths(paths: list[str]):
        """Given a list of filenames, e.g., ["/path/to/foo_arch.deb"], returns
        the set of package names, e.g., {"foo"}."""
        filenames = sorted([x.split("/")[-1] for x in paths])
        return set([re.split("[-_]", x)[0] for x in filenames])

    def test_developer_bootstrap(self):
        """Checks --developer with nothing installed yet."""
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--developer", "-y"])

        if sys.platform != "darwin":
            # The DUT should install bazelisk and maybe kcov (after confirming
            # that they are missing).
            dut.expect_dpkg_query()
            paths = dut.expect_apt_install()
            names = self._extract_debian_package_names_from_paths(paths)
            self.assertIn(names, ({"bazelisk"}, {"bazelisk", "kcov"}))

        # The DUT prefetches bazel.
        dut.expect_call(["bazel", "version"])

        dut.finish()
        self._check_developer_stdout_and_source_results(dut=dut)
        self.assertEqual(dut.returncode, 0)

    def _check_developer_stdout_and_source_results(self, *, dut):
        """Checks that the --developer expected stdout and files exist."""
        self.assertRegex(dut.stdout, "Writing.*gen/python_version.txt")
        self.assertRegex(dut.stdout, "Writing.*gen/environment.bazelrc")
        self.assertRegex(dut.stdout, "Pre-fetching bazel")
        self.assertTrue((dut.source() / "gen/python_version.txt").exists())
        self.assertTrue((dut.source() / "gen/environment.bazelrc").exists())

    def test_developer_bump(self):
        """Checks --developer with some things already installed, but at too-old
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
            names = self._extract_debian_package_names_from_paths(paths)
            self.assertEqual(names, {"bazelisk"})

        # The DUT prefetches bazel.
        dut.expect_call(["bazel", "version"])

        dut.finish()
        self._check_developer_stdout_and_source_results(dut=dut)
        self.assertEqual(dut.returncode, 0)

    def test_developer_completed(self):
        """Checks --developer when everything is already installed (as if a
        prior run had already succeeded)."""
        dut = InstallPrereqsActor(test_case=self)
        dut.installed_packages = {
            "bazelisk": EXPECTED_BAZELISK,
            "kcov": EXPECTED_KCOV,
        }
        dut.start(args=["--developer", "-y"])

        if sys.platform != "darwin":
            # The DUT confirms that bazelisk (etc) is already installed, and
            # doesn't install anything further.
            dut.expect_dpkg_query()

        # The DUT prefetches bazel.
        dut.expect_call(["bazel", "version"])

        dut.finish()
        self._check_developer_stdout_and_source_results(dut=dut)
        self.assertEqual(dut.returncode, 0)

    @unittest.skipIf(sys.platform == "darwin", "Ubuntu only")
    def test_ubuntu_developer_oldtest_xxx_fixme(self):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--developer", "-y"])

        # The DUT first checks which platform it's running on.
        dut.expect_lsb_release()

        # The DUT asks what's already installed. Tell it nothing in reply.
        # Therefore, the DUT will install all packages.
        dut.expect_call(prefix=["dpkg-query"])
        package_names = dut.expect_apt_install()

        # If this check fails, see the comment in our setUp() method.
        self.assertEqual(
            len(package_names), self._expected_num_apt_packages["developer"]
        )

        # The DUT asks is bazelisk is installed. Tell it nothing in reply.
        # Therefore, the DUT will install it.
        dut.expect_call(prefix=["dpkg-query"])
        dut.expect_call(exact=["dpkg", "--print-architecture"], stdout="amd64")
        dut.expect_call(prefix=["sudo", "dpkg", "--install"])
        dut.expect_call(prefix=["sudo", "apt-get", "install", "--fix-broken"])

        # The DUT asks which locales exist. Tell it nothing in reply.
        # Therefore, the DUT will geneate the locale.
        dut.expect_call(exact=["locale", "-a"])
        dut.expect_call(exact=["sudo", "locale-gen", "en_US.utf8"])

        dut.expect_call(prefix=["bazel", "version"])

        # Nothing else should happen.
        dut.finish()
        self.assertIn("Successfully installed --flavor=developer", dut.stdout)
        self.assertEqual(dut.returncode, 0)


logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
