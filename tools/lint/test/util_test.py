import os
import unittest

from tools.lint.util import find_all_sources


class UtilTest(unittest.TestCase):

    def test_find(self):
        workspace_dir, relpaths = find_all_sources()

        # Sanity-check workspace_dir.
        self.assertGreater(len(workspace_dir), 10)
        workspace = os.path.join(workspace_dir, "WORKSPACE")
        self.assertTrue(os.path.exists(workspace))
        with open(workspace, "r") as workspace_contents:
            workspace_lines = workspace_contents.readlines()
        self.assertTrue('workspace(name = "drake")\n' in workspace_lines)

        # Sanity-check relpaths.
        self.assertGreater(len(relpaths), 1000)
        self.assertTrue('.bazelproject' in relpaths)
        self.assertTrue('setup/ubuntu/16.04/install_prereqs.sh' in relpaths)
        THIRD_PARTY_SOURCES_ALLOWED_TO_BE_FOUND = [
            "third_party/BUILD.bazel",
            "third_party/README.md",
        ]
        for one_relpath in relpaths:
            self.assertTrue(".git/" not in one_relpath, one_relpath)
            if "third_party/" in one_relpath:
                self.assertTrue(
                    one_relpath in THIRD_PARTY_SOURCES_ALLOWED_TO_BE_FOUND or
                    one_relpath.startswith("."),
                    one_relpath)


# TODO(jwnimmer-tri) Omitting or mistyping these lines means that no tests get
# run, and nobody notices.  We should probably have drake_py_unittest macro
# that takes care of this, to be less brittle.
if __name__ == '__main__':
    unittest.main()
