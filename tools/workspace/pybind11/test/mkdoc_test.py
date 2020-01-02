import unittest
from drake.third_party.com_github_pybind_pybind11.mkdoc import process_comment


class TestMkdoc(unittest.TestCase):

    # TODO(m-chaturvedi, eric.cousineau): Completely test `mkdoc.py`.
    # Meanwhile, please refer to: https://git.io/JexDb
    def test_process_comment_issue_12445(self):
        self.assertEqual(process_comment("// `one` and `two`"),
                         "// ``one`` and ``two``")
        self.assertEqual(process_comment("/// this is foo"), "this is foo")
        self.assertEqual(process_comment(
            "///< this is foo\n/// this is bar"), "this is foo this is bar")
