import unittest


class TestRulesPythonInternal(unittest.TestCase):

    def test_vendoring(self):
        """Checks that our vendored copy of a file is up to date."""
        # Load the two putatively-identical files.
        name = "internal_config_repo.bzl"
        drake_dir = "third_party/com_github_bazelbuild_rules_python"
        external_dir = "external/rules_python/python/private"
        with open(f"{drake_dir}/{name}", encoding="utf-8") as f:
            drake_copy = f.read()
        with open(f"{external_dir}/{name}", encoding="utf-8") as f:
            external_copy = f.read()

        # Drake contains one customization.
        old = 'load("//python/private:bzlmod_enabled.bzl", "BZLMOD_ENABLED")'
        new = 'BZLMOD_ENABLED = False'
        patched_external_copy = external_copy.replace(old, new)

        # Report any differences. If this test fails, you need to copy the
        # file from bazel-drake/{external_dir}/{name} to {drake_dir}/{name}
        # and re-apply the customization.
        self.maxDiff = None
        self.assertMultiLineEqual(drake_copy, patched_external_copy)
