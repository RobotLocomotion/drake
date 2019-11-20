import os
import unittest

from drake.tools.lint.util import find_all_sources


class UtilTest(unittest.TestCase):

    def test_find(self):
        workspace_dir, relpaths = find_all_sources("drake")

        # Sanity-check workspace_dir.
        self.assertTrue(workspace_dir.startswith("/"), workspace_dir)
        self.assertTrue(os.path.isdir(workspace_dir), workspace_dir)
        workspace = os.path.join(workspace_dir, "WORKSPACE")
        self.assertTrue(os.path.exists(workspace), workspace)
        with open(workspace, "r") as workspace_contents:
            workspace_lines = workspace_contents.readlines()
        self.assertTrue('workspace(name = "drake")\n' in workspace_lines)

        # Sanity-check relpaths.
        self.assertGreater(len(relpaths), 1_000)
        self.assertLess(len(relpaths), 10_000)
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
                    one_relpath in THIRD_PARTY_SOURCES_ALLOWED_TO_BE_FOUND or
                    one_relpath.startswith("."),
                    one_relpath + " has been mis-identified as a source file")
