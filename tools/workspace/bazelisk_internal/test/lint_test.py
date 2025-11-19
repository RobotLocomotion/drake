from pathlib import Path
import unittest

from python import runfiles

_HOW_TO_FIX = """
If you're seeing this, you probably didn't follow the upgrade instuctions in
tools/workspace/bazelisk_internal/repository.bzl. Here they are again...

************************************************************************
To update Drake's vendored copy of bazelisk:

$ cd drake
$ bazel build @bazelisk_internal//:*
$ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
    bazel-drake/external/+internal_repositories+bazelisk_internal/LICENSE \\
    bazel-drake/external/+internal_repositories+bazelisk_internal/bazelisk.py
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
            upstream_content = self._read(f"bazelisk_internal/{name}")
            vendored_content = self._read(
                f"drake/third_party/com_github_bazelbuild_bazelisk/{name}"
            )
            self.assertMultiLineEqual(
                upstream_content, vendored_content, "\n" + _HOW_TO_FIX
            )
