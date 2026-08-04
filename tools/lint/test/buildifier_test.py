import os
from pathlib import Path
import subprocess
import unittest


class BuildifierTest(unittest.TestCase):
    def _make_build_testdata(self, add_error=False):
        result = []
        result.append("package(default_visibility = [])")

        unwanted_whitespace = ""
        if add_error:
            unwanted_whitespace = " "
        result.append(f'filegroup(name = "mygroup"{unwanted_whitespace})')

        return "\n\n".join(result) + "\n"

    def _call_buildifier(self, testdata_contents, args):
        tmpdir = Path(os.environ["TEST_TMPDIR"])
        build_path = tmpdir / "BUILD.bazel"
        build_path.write_text(testdata_contents, encoding="utf-8")
        buildifier = Path("tools/lint/buildifier").absolute()
        command = [buildifier] + args + ["BUILD.bazel"]
        process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=tmpdir
        )
        stdout, _ = process.communicate()
        return process.returncode, stdout.decode("utf8")

    def test_mode_check(self):
        returncode, output = self._call_buildifier(
            self._make_build_testdata(add_error=False), ["-mode=check"]
        )
        self.assertEqual(returncode, 0, output)
        self.assertEqual(output, "")

        returncode, output = self._call_buildifier(
            self._make_build_testdata(add_error=True), ["-mode=check"]
        )
        self.assertEqual(returncode, 1, output)
        self.assertListEqual(
            output.splitlines(),
            [
                "ERROR: buildifier: the required formatting is incorrect",
                (
                    "ERROR: BUILD.bazel:1: "
                    "error: the required formatting is incorrect"
                ),
                (
                    "ERROR: BUILD.bazel:1: note: "
                    "fix via bazel-bin/tools/lint/buildifier BUILD.bazel"
                ),
                (
                    "ERROR: BUILD.bazel:1: note: "
                    "if that program does not exist, "
                    "you might need to compile it first: "
                    "bazel build //tools/lint/..."
                ),
                "NOTE: see https://drake.mit.edu/bazel.html#buildifier",
            ],
        )
