import unittest
from pathlib import Path

from python import runfiles


_HOW_TO_FIX = """
If you're seeing this, you probably didn't follow the upgrade instuctions in
tools/workspace/bazelisk/repository.bzl. Here they are again...

************************************************************************
To update Drake's vendored copy of bazelisk:

$ cd drake
$ bazel build @bazelisk//:*
$ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
    bazel-drake/external/bazelisk/LICENSE \\
    bazel-drake/external/bazelisk/bazelisk.py
************************************************************************
"""


class BazeliskLintTest(unittest.TestCase):

    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def test_vendored_copy(self):
        """Checks that our vendored copy of bazelisk is up to date with the
        repository pin.
        """
        for name in ["LICENSE", "bazelisk.py"]:
            upstream_content = self._read(f"bazelisk/{name}")
            vendored_content = self._read(
                f"drake/third_party/com_github_bazelbuild_bazelisk/{name}")
            self.assertMultiLineEqual(upstream_content, vendored_content,
                                      "\n" + _HOW_TO_FIX)
