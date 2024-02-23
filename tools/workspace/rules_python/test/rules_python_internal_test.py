from pathlib import Path
import unittest


class TestRulesPythonInternal(unittest.TestCase):

    def test_vendoring(self):
        """Checks that our vendored copy of a file is up to date."""
        # Load the two putatively-identical files.
        name = "internal_config_repo.bzl"
        drake_dir = "third_party/com_github_bazelbuild_rules_python"
        external_dir = "external/rules_python/python/private"
        drake = Path(f"{drake_dir}/{name}").read_text(encoding="utf-8")
        upstream = Path(f"{external_dir}/{name}").read_text(encoding="utf-8")

        # Report any differences.
        _HELP = (
            "If this test fails, you need to copy the file from "
            f"bazel-drake/{external_dir}/{name} to {drake_dir}/{name}"
        )
        self.assertMultiLineEqual(drake, upstream, f"\n\n{_HELP}")
