import os
import subprocess
import unittest


class BuildifierTest(unittest.TestCase):

    def _make_build_testdata(self, add_error=False):
        result = []
        result.append('package(default_visibility = [])')

        unwanted_whitespace = ""
        if add_error:
            unwanted_whitespace = " "
        result.append('filegroup(name = "mygroup"{})'.format(
            unwanted_whitespace))

        return "\n\n".join(result) + "\n"

    def _call_buildifier(self, testdata_contents, args):
        testdata_filename = "tmp/BUILD.bazel"
        if not os.path.exists("tmp"):
            os.mkdir("tmp")
        with open(testdata_filename, "w") as testdata_file:
            testdata_file.write(testdata_contents)
        command = ["tools/lint/buildifier"] + args + [testdata_filename]
        process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        stdout, _ = process.communicate()
        return process.returncode, stdout.decode('utf8')

    def test_mode_check(self):
        returncode, output = self._call_buildifier(
            self._make_build_testdata(add_error=False),
            ["-mode=check"])
        self.assertEqual(returncode, 0, output)
        self.assertEqual(output, "")

        returncode, output = self._call_buildifier(
            self._make_build_testdata(add_error=True),
            ["-mode=check"])
        self.assertEqual(returncode, 1, output)
        self.assertListEqual(output.splitlines(), [
            "ERROR: buildifier: the required formatting is incorrect",
            ("ERROR: tmp/BUILD.bazel:1: "
             "error: the required formatting is incorrect"),
            ("ERROR: tmp/BUILD.bazel:1: note: "
             "fix via bazel-bin/tools/lint/buildifier tmp/BUILD.bazel"),
            ("ERROR: tmp/BUILD.bazel:1: note: "
             "if that program does not exist, "
             "you might need to compile it first: "
             "bazel build //tools/lint/..."),
            "NOTE: see https://drake.mit.edu/bazel.html#buildifier"
        ])

    def test_mode_fix(self):
        returncode, output = self._call_buildifier(
            self._make_build_testdata(add_error=False),
            ["-mode=fix"])
        self.assertEqual(returncode, 1, output)
        self.assertIn(
            "do not use 'bazel run' for buildifier in fix mode",
            output)
