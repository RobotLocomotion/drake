import os
import unittest

from drake.tools.lint.util import find_all_sources


class UtilTest(unittest.TestCase):

    def test_find(self):
        workspace_dir, relpaths = find_all_sources("drake")

        # Sanity-check workspace_dir.  Most of the correctness assertions are
        # already embedded within the subroutine itself.
        with open(os.path.join(workspace_dir, "WORKSPACE"), "r") as contents:
            workspace_lines = contents.readlines()
        self.assertTrue('workspace(name = "drake")\n' in workspace_lines)

        # Sanity-check relpaths.
        self.assertGreater(len(relpaths), 1_000)
        self.assertTrue('.bazelproject' in relpaths)
        self.assertTrue('setup/ubuntu/install_prereqs.sh' in relpaths)
        THIRD_PARTY_SOURCES_ALLOWED_TO_BE_FOUND = [
            "third_party/BUILD.bazel",
            "third_party/README.md",
        ]
        for one_relpath in relpaths:
            self.assertTrue(".git/" not in one_relpath, one_relpath)
            if one_relpath.startswith("third_party/"):
                self.assertTrue(
                    one_relpath in THIRD_PARTY_SOURCES_ALLOWED_TO_BE_FOUND
                    or one_relpath.startswith("."),
                    one_relpath + " has been mis-identified as a source file")
