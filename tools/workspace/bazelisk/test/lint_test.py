import unittest

_HOW_TO_FIX = """
To update Drake's vendored copy of bazelisk:
$ cd drake
$ bazel fetch @bazelisk//:LICENSE
$ cp bazel-drake/external/bazelisk/LICENSE \
     bazel-drake/external/bazelisk/bazelisk.py \
     third_party/com_github_bazelbuild_bazelisk/
"""


class BazeliskLintTest(unittest.TestCase):

    def _read(self, filename):
        """Returns the contents of the given filename."""
        with open(filename, encoding="utf-8") as f:
            return f.read()

    def test_vendored_copy(self):
        """Checks that our vendored copy of bazelisk is up to date with the
        repository pin.
        """
        for name in ["LICENSE", "bazelisk.py"]:
            upstream_content = self._read("external/bazelisk/" + name)
            vendored_content = self._read(
                "third_party/com_github_bazelbuild_bazelisk/" + name)
            self.assertMultiLineEqual(upstream_content, vendored_content,
                                      "\n" + _HOW_TO_FIX)
