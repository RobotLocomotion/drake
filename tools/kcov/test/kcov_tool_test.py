import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest
import zipfile

from python import runfiles


class TestKcovTool(unittest.TestCase):
    def setUp(self):
        self._temp_dir = tempfile.TemporaryDirectory(prefix="kcov_tool_test_")
        self.addCleanup(self._temp_dir.cleanup)
        temp = Path(self._temp_dir.name)

        # Lay out a fake workspace, with one test's output directory in the
        # place where bazel-testlogs would hold it.
        self.workspace = temp / "workspace"
        self.outputs = (
            self.workspace / "bazel-testlogs" / "pkg" / "foo_test"
            / "test.outputs"
        )  # fmt: skip
        self.outputs.mkdir(parents=True)

        # Provide a stand-in for the kcov program, which records its arguments.
        # (We only care that kcov_tool calls it with the right data in place.)
        bin_dir = temp / "bin"
        bin_dir.mkdir()
        self.kcov_log = temp / "kcov.log"
        fake_kcov = bin_dir / "kcov"
        fake_kcov.write_text('#!/bin/sh\necho "$@" > "$KCOV_LOG"\n')
        fake_kcov.chmod(fake_kcov.stat().st_mode | stat.S_IXUSR)
        self._env = dict(
            os.environ,
            PATH=f"{bin_dir}{os.pathsep}{os.environ['PATH']}",
            KCOV_LOG=str(self.kcov_log),
        )

    def _run(self, *args):
        """Runs kcov_tool as a command-line program on the fake workspace."""
        tool = runfiles.Create().Rlocation("drake/tools/kcov/kcov_tool")
        return subprocess.run(
            [sys.executable, tool, "--workspace", str(self.workspace), *args],
            env=self._env,
            capture_output=True,
            text=True,
            check=True,
        )

    def _make_zip(self):
        """Makes an outputs.zip like the one bazel makes, holding kcov data."""
        with zipfile.ZipFile(self.outputs / "outputs.zip", "w") as zf:
            zf.writestr("kcov/", "")
            zf.writestr("kcov/index.html", "hello")

    def _make_subtree(self, filename="index.html"):
        (self.outputs / "kcov").mkdir()
        (self.outputs / "kcov" / filename).write_text("hello")

    def test_clean_zip_only(self):
        # Bazel leaves the data only in the zip file until someone expands it.
        self._make_zip()
        self._run("clean")
        self.assertFalse((self.outputs / "outputs.zip").exists())

    def test_clean_subtree_and_zip(self):
        self._make_zip()
        self._make_subtree()
        self._run("clean")
        self.assertFalse((self.outputs / "kcov").exists())
        self.assertFalse((self.outputs / "outputs.zip").exists())

    def test_clean_subtree_only(self):
        self._make_subtree()
        self._run("clean")
        self.assertFalse((self.outputs / "kcov").exists())

    def test_merge_from_zip(self):
        self._make_zip()
        merged = self.workspace / "merged"
        self._run("merge", str(merged))
        index = self.outputs / "kcov" / "index.html"
        self.assertEqual(index.read_text(), "hello")
        self.assertEqual(
            self.kcov_log.read_text().split(),
            ["--merge", str(merged), str(self.outputs / "kcov")],
        )

    def test_merge_without_zip_keeps_data(self):
        # A subtree that has no index.html and no zip to restore it from is
        # left alone, not deleted, and it is not an error.
        self._make_subtree(filename="other.dat")
        merged = self.workspace / "merged"
        self._run("merge", str(merged))
        self.assertTrue((self.outputs / "kcov" / "other.dat").exists())
        self.assertEqual(
            self.kcov_log.read_text().split(),
            ["--merge", str(merged), str(self.outputs / "kcov")],
        )
