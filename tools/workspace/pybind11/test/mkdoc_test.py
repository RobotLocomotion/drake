import unittest
import third_party.com_github_pybind_pybind11.mkdoc as mkdoc


class TestMkdoc(unittest.TestCase):

    # Test for issue #12445
    def test_process_comment(self):
        proc = mkdoc.process_comment
        self.assertEqual(proc("// `one` and `two`"), "// ``one`` and ``two``")
        self.assertEqual(proc("/// this is foo"), "this is foo")
        self.assertEqual(proc("///< this is foo"), "this is foo")
