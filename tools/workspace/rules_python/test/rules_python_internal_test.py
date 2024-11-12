from pathlib import Path
import unittest

from python import runfiles


class TestRulesPythonInternal(unittest.TestCase):

    def test_vendoring(self):
        """Checks that our vendored copy of a file is up to date."""
        manifest = runfiles.Create()

        # Load the two putatively-identical files.
        name = "internal_config_repo.bzl"
        drake_dir = "third_party/com_github_bazelbuild_rules_python"
        external_dir = "rules_python/python/private"
        drake_path = manifest.Rlocation(f"drake/{drake_dir}/{name}")
        upstream_path = manifest.Rlocation(f"{external_dir}/{name}")
        drake_content = Path(drake_path).read_text(encoding="utf-8")
        upstream_content = Path(upstream_path).read_text(encoding="utf-8")

        # Report any differences.
        _HELP = (
            "If this test fails, you need to copy the file from "
            f"bazel-drake/external/{external_dir}/{name} to {drake_dir}/{name}"
        )
        self.assertMultiLineEqual(drake_content, upstream_content,
                                  f"\n\n{_HELP}")
