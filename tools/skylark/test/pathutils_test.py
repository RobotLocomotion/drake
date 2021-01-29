import unittest

from bazel_tools.tools.python.runfiles import runfiles


class TestPathUtils(unittest.TestCase):
    """Unit tests for the starlark code in pathutils.bzl."""

    def setUp(self):
        filename = runfiles.Create().Rlocation(
            "drake/tools/skylark/pathutils.bzl")
        with open(filename, "r", encoding="utf-8") as f:
            bzl_contents = f.read()
        self.bzl_globals = dict()
        exec(bzl_contents, self.bzl_globals)

    def _remove_prefix(self, path, prefix):
        """Calls the pathutils.bzl function of the same name."""
        expression = f"_remove_prefix({repr(path)}, {repr(prefix)})"
        return eval(expression, self.bzl_globals)

    def test_remove_prefix(self):
        # A list of path, prefix, expected result.
        suite = [
            # Basic literal matching.
            ["a/b",   "a",   "b"],
            ["a/b/c", "a",   "b/c"],
            ["a/b/c", "a/b", "c"],
            ["a/b", "a/b",   None],  # Prefix identical to path.
            ["a/b", "a/b/c", None],  # Prefix longer than the path.
            ["b",   "a/b/c", None],  # Prefix in the middle of the path.

            # Single glob matching.
            ["a/b",     "*",     "b"],
            ["a/b/c",   "*",     "b/c"],
            ["a/b/c",   "a/*",   "c"],
            ["a/b",     "a/*",   None],  # The * must match exactly one name.
            ["a/b/c",   "*/b",   "c"],
            ["b/c",     "*/b",   None],  # The * must match exactly one name.
            ["a/b/c/d", "a/*/c", "d"],
            ["a/c/d",   "a/*/c", None],  # The * must match exactly one name.

            # Two glob matching.
            ["a/b/c",   "*/*",   "c"],
            ["a/b",     "*/*",   None],  # Each * must match exactly one name.
            ["a/b/c/d", "a/*/*", "d"],
            ["a/b/c",   "a/*/*", None],  # Each * must match exactly one name.
            ["a/b/c/d", "*/*/c", "d"],
            ["a/b/d",   "*/*/c", None],  # Each * must match exactly one name.

            # Double glob matching.
            ["a/b/c/d", "**",       "d"],
            ["a/b/c/d", "a/b/c/**", "d"],
            ["a/b/c/d", "a/**",     "d"],
            ["a/b/c/d", "a/*/**",   "d"],
            ["a/b/c/d", "a/**/*",   "d"],
            ["a/b/c/d", "**/c",     "d"],
            ["a/b/c/d", "a/**/c",   "d"],
            ["a/b/c/d", "a/b/**/c", "d"],
            ["a/b/c/d", "**/b",     "c/d"],
            ["a/b/c/d", "a/**/b",   "c/d"],
            ["a/b/c/d", "**/a/b",   "c/d"],

            # Repeated final name (".../c/c") in prefix.
            ["a/b/c/c", "a/**/c", "c"],
        ]
        for path, prefix, expected in suite:
            with self.subTest(path=path, prefix=prefix):
                result = self._remove_prefix(path, prefix)
                self.assertEqual(result, expected)

                result2 = self._remove_prefix(path, prefix + "/")
                self.assertEqual(result2, expected)
