import os
import logging
import multiprocessing.connection
import re
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
from pathlib import Path

from python import runfiles


class InstallPrereqsActor():

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
        # across putatively indepndent test cases.)
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

        # Create a listening socket used for subprocess callbacks.
        self._listener = multiprocessing.connection.Listener()
        address = self._listener.address

        # Create a stub program that calls back into this unit test anytime
        # it's run.
        self._stubby = self._path / ".stubby"
        self._stubby.write_text(
            encoding="utf-8",
            data=textwrap.dedent(f"""\
            #!{sys.executable}
            import sys
            from multiprocessing.connection import Client
            with Client({address!r}) as conn:
                conn.send(sys.argv)
                result = conn.recv()
            sys.stdout.write(result["stdout"])
            sys.stdout.flush()
            sys.exit(result["returncode"])
            """),
        )
        self._stubby.chmod(0o777)

        # Estagblish the allowed list of commands the DUT can run.
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

    def _set_up_source(self):
        """This is a subroutine used only by our __init__ function. (It just
        got a bit too long to keep this all inline there.) When we're called,
        the self._source is an empty directory. It's our job to symlink the
        install_prereqs script into there, based on the self._tree mode which
        is either "source_tree" or "install_tree". We'll return the new path
        to install_prereqs inside self._source.
        """
        assert self._source.exists()
        assert self._tree in ["source_tree", "install_tree"]
        manifest = runfiles.Create()
        install_prereqs = Path(manifest.Rlocation(
            "drake/setup/install_prereqs"))

        if self._tree == "source_tree":
            # When running install_prereqs from the source tree, all of the
            # platform-specific data files are available, and the directory
            # the script resides in a directory named "setup".
            setup = self._source / "setup"
            setup.mkdir()
            result = self._source / "setup/install_prereqs"
            result.symlink_to(install_prereqs)
            for platform in ["mac", "ubuntu"]:
                (setup / platform).symlink_to(
                    install_prereqs.parent / platform)
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

    def add_to_path(self, program):
        if program == "python3":
            (self._path / program).symlink_to(sys.executable)
        else:
            (self._path / program).symlink_to(self._stubby)

    def remove_from_path(self, program):
        (self._path / program).unlink()

    def start(self, *, args):
        assert self._process is None
        logging.info(f"Running install_prereqs with {args} ...")
        full_args = [self._script] + args
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
        assert self._process is not None
        self._listener.close()
        try:
            self._process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._process.terminate()
        self._process.wait()
        self.stdout = self._process.stdout.read()
        for line in self.stdout.splitlines():
            logging.info(f" [stdout] {line}")
        self.returncode = self._process.returncode
        # XXX somehow we should automate checking for source-tree writes

    def expect_call(self,
                    *,
                    exact=None,
                    prefix=None,
                    stdout="",
                    returncode=0):
        assert sum([exact is not None, prefix is not None]) == 1

        # Fail-fast in case the process has crashed; otherwise, our attempt to
        # accept() will block indefinitely.
        time.sleep(0.01)
        self._process.poll()
        self._test_case.assertIsNone(self._process.returncode)

        # Print now in case we get stuck.
        exact_or_prefix = (exact or prefix)
        command_info = exact_or_prefix[0]
        if command_info == "sudo" and exact_or_prefix[1][0] != "-":
            command_info = " ".join(exact_or_prefix[:2])
        logging.info(f"Waiting for subprocess call to {command_info} ...")

        # Wait for the subprocess callback and then tell it what to do.
        with self._listener.accept() as conn:
            argv = conn.recv()
            conn.send(dict(
                stdout=stdout,
                returncode=returncode,
            ))

        # Strip the useless directory name off of the actual command.
        argv[0] = argv[0].split("/")[-1]

        # Validate the called program and its arguments.
        if exact is not None:
            self._test_case.assertEqual(argv, exact)
        if prefix is not None:
            some_argv = argv[:len(prefix)]
            self._test_case.assertEqual(some_argv, prefix)
            self._test_case.assertGreaterEqual(len(argv), len(prefix))

        return argv

    def expect_lsb_release(self):
        self.expect_call(
            exact=["lsb_release", "-idrc"],
            stdout=textwrap.dedent("""\
            Distributor ID:	Ubuntu
            Description:	Ubuntu 24.04 LTS
            Release:	24.04
            Codename:	noble
            """),
        )

    def expect_sudo_check_if_not_yet_checked(self):
        if self._did_sudo_check:
            return
        self.expect_call(exact=["sudo", "-v"])
        self._did_sudo_check = True

    def expect_apt_update_if_not_yet_updated(self):
        if self._did_apt_update:
            return
        self.expect_sudo_check_if_not_yet_checked()
        self.expect_call(exact=["sudo", "apt-get", "update"])
        self._did_apt_update = True

    def expect_apt_install(self):
        self.expect_apt_update_if_not_yet_updated()
        argv = self.expect_call(prefix=["sudo", "apt-get", "install"])
        package_names = [
            arg
            for arg in argv[3:]
            if not arg.startswith("-")
        ]
        return package_names


class InstallPrereqsTest(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None

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

    # TODO(jwnimmer-tri) We should really have --help on macOS, too.
    @unittest.skipIf(sys.platform == "darwin", "Ubuntu only")
    def test_help(self):
        for tree in ["source_tree", "install_tree"]:
            with self.subTest(tree=tree):
                dut = InstallPrereqsActor(test_case=self, tree=tree)
                dut.start(args=["--help"]).finish()
                self.assertIn("usage: install_prereqs", dut.stdout)
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

    def _check_ubuntu_binary(self, *,
                             has_lsb_release: bool,
                             already_installed: bool,
                             yes: bool):
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
        dut.expect_call(prefix=["dpkg-query"], stdout="".join([
            f"{name} ii someversion\n"
            for name in installed_packages
        ]))

        # The DUT will install any missing packages.
        package_names = dut.expect_apt_install()

        # If this check fails, see the comment in our setUp() method.
        self.assertEqual(len(package_names) + len(installed_packages),
                         self._expected_num_apt_packages["binary"])

        # Nothing else should happen.
        dut.finish()
        self._check_stdout_match(dut=dut, expected=[
            "INFO: Running: sudo apt-get update ...",
            (re.compile(".*apt-get install.*lsb-release ...")
             if not has_lsb_release else None),
            re.compile("INFO: Running: sudo apt-get install.*"),
            "INFO: Successfully installed --flavor=binary prereqs.",
        ])
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
        self.assertEqual(len(package_names),
                         self._expected_num_apt_packages["build"])

        # The DUT asks which locales exist. Tell it nothing in reply.
        # Therefore, the DUT will geneate the locale.
        dut.expect_call(exact=["locale", "-a"])
        dut.expect_call(exact=["sudo", "locale-gen", "en_US.utf8"])

        # Nothing else should happen.
        dut.finish()
        self.assertIn("Successfully installed --flavor=build", dut.stdout)
        self.assertEqual(dut.returncode, 0)

    @unittest.skipIf(sys.platform == "darwin", "Ubuntu only")
    def test_ubuntu_developer(self):
        dut = InstallPrereqsActor(test_case=self)
        dut.start(args=["--developer", "-y"])

        # The DUT first checks which platform it's running on.
        dut.expect_lsb_release()

        # The DUT asks what's already installed. Tell it nothing in reply.
        # Therefore, the DUT will install all packages.
        dut.expect_call(prefix=["dpkg-query"])
        package_names = dut.expect_apt_install()

        # If this check fails, see the comment in our setUp() method.
        self.assertEqual(len(package_names),
                         self._expected_num_apt_packages["developer"])

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
