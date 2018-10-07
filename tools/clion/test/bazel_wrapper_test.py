import imp
import os
import shutil
import subprocess
import tempfile
import unittest

from six import StringIO

imp.load_source("bazel_wrapper", "tools/clion/bazel_wrapper")
from bazel_wrapper import main as bazel_wrapper_main  # noqa


class ExecDetail(Exception):
    def __init__(self, name, args):
        super(ExecDetail, self).__init__()
        self.name = name
        self.args = args


class Mock(object):
    pass


class TestBazelWrapper(unittest.TestCase):

    def setUp(self):
        # Collect what the DUT prints to stderr.
        self._stderr = ""
        # Provide a mock Popen of the bazel subprocess.
        self._bazel_subprocess_actions = []
        self._bazel_subprocess_mock = Mock()
        self._bazel_subprocess_mock.terminate = self._bazel_terminate
        self._bazel_subprocess_mock.wait = self._bazel_wait
        self._bazel_subprocess_mock.returncode = 22
        # Create a scratch dir, write a WORKSPACE, and chdir to it.
        self._oldcwd = os.getcwd()
        self._tempdir = tempfile.mkdtemp(
            prefix="drake_tools_clion_bazel_wrapper_test_",
            dir=os.environ["TEST_TMPDIR"])
        with open(os.path.join(self._tempdir, "WORKSPACE"), "w") as f:
            f.write('workspace(name = "drake")\n')
        os.chdir(self._tempdir)

    def tearDown(self):
        # Chdir back to where we were, and rm -r the tempdir.
        os.chdir(self._oldcwd)
        if self._tempdir:
            shutil.rmtree(self._tempdir)

    def _execvp(self, name, args):
        # Callback mock for the DUT's main.
        raise ExecDetail(name, args)

    def _write_stderr(self, data):
        # Callback stub for the DUT's main.
        self._stderr += data

    def _popen(self, *args, **kwargs):
        # Callback stub for the DUT's main.
        return self._bazel_subprocess_mock

    def _bazel_terminate(self):
        # Callback stub for the DUT's main.
        self._bazel_subprocess_actions.append("terminate")

    def _bazel_wait(self):
        # Callback stub for the DUT's main.
        self._bazel_subprocess_actions.append("wait")

    def _do_main(self, argv):
        bazel_wrapper_main(
            argv=argv,
            execvp=self._execvp,
            write_stderr=self._write_stderr,
            popen=self._popen)

    def test_no_workspace(self):
        # When cwd has no WORKSPACE, the wrapper immediately bails out.
        os.chdir(self._oldcwd)
        argv = ["bazel", "dummy_arg"]
        with self.assertRaises(ExecDetail) as detail:
            self._do_main(argv)
        self.assertEqual(detail.exception.name, "bazel")
        self.assertSequenceEqual(detail.exception.args, argv)
        self.assertEqual(
            "Skipping drake/tools/clion/bazel_wrapper (empty)\n",
            self._stderr)

    def test_different_workspace(self):
        # When cwd has wrong WORKSPACE, the wrapper immediately bails out.
        with open(os.path.join(self._tempdir, "WORKSPACE"), "w") as f:
            f.write('workspace(name = "foo")\n')
        argv = ["bazel", "dummy_arg"]
        with self.assertRaises(ExecDetail) as detail:
            self._do_main(argv)
        self.assertEqual(detail.exception.name, "bazel")
        self.assertSequenceEqual(detail.exception.args, argv)
        self.assertEqual(
            "Skipping drake/tools/clion/bazel_wrapper (non-drake)\n",
            self._stderr)

    def test_no_rewriting(self):
        # When the user disables stderr rewriting, the wrapper bails out but
        # still replaces the include paths.
        old_magic = "--aspects=@intellij_aspect//:intellij_info_bundled.bzl%intellij_info_aspect"  # noqa
        new_magic = "--aspects=@drake//tools/clion:aspect.bzl%intellij_info_aspect"  # noqa
        argv = ["bazel", old_magic, "--nodrake_error_rewriting", "dummy_arg"]
        with self.assertRaises(ExecDetail) as detail:
            self._do_main(argv)
        self.assertEqual(detail.exception.name, "bazel")
        self.assertSequenceEqual(
            detail.exception.args,
            ["bazel", new_magic, "dummy_arg"])
        self.assertEqual("", self._stderr)

    def test_subprocess(self):
        # The wrapper replaces the include paths and runs until EOF.
        self._bazel_subprocess_mock.stderr = StringIO("")
        argv = ["bazel", "dummy_arg"]
        with self.assertRaises(SystemExit) as detail:
            self._do_main(argv)
        self.assertEqual(
            detail.exception.code,
            self._bazel_subprocess_mock.returncode)
        self.assertEqual("", self._stderr)
        self.assertSequenceEqual(
            self._bazel_subprocess_actions,
            ["terminate", "wait"])

    def test_rewriting(self):
        # The wrapper replaces the include paths and runs until EOF.
        os.mkdir("bazel-out")
        os.symlink("WORKSPACE", "symlink-in-workspace")
        os.symlink("/dev/null", "symlink-outside-workspace")
        os.symlink("../WORKSPACE", "bazel-out/symlink")
        print(subprocess.check_output(["ls", "-lR"]))
        bold = "\x1b[0m\x1b[1m"  # ANSI escape codes (pretty!).
        original_lines = [
            "stuff",
            "In file included from symlink-in-workspace",
            "In file included from symlink-outside-workspace",
            "bazel-out/symlink:1",
            "stuff/bazel-out/symlink:2",
            bold + "bazel-out/symlink:3",
        ]
        edited_lines = [
            "stuff",
            "In file included from WORKSPACE",
            "In file included from symlink-outside-workspace",
            "WORKSPACE:1",
            "stuff/WORKSPACE:2",
            bold + "WORKSPACE:3",
        ]
        self._bazel_subprocess_mock.stderr = StringIO(
            "\n".join(original_lines))
        argv = ["bazel", "dummy_arg"]
        with self.assertRaises(SystemExit) as detail:
            self._do_main(argv)
        self.assertEqual(
            detail.exception.code,
            self._bazel_subprocess_mock.returncode)
        self.assertSequenceEqual(
            self._bazel_subprocess_actions,
            ["terminate", "wait"])
        for orig, edit in zip(edited_lines, self._stderr.splitlines()):
            self.assertEqual(orig, edit)
