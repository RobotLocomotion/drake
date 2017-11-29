import os
import unittest

from tools.lint.find_data import find_data


class FindDataTest(unittest.TestCase):

    def test_find(self):
        filename = find_data("tools/lint/test/find_data_test.txt")
        self.assertTrue(os.path.exists(filename))
        with open(filename, "r") as contents:
            lines = contents.readlines()
        self.assertListEqual(lines, ["Sample data for unit find_data_test.\n"])


# TODO(jwnimmer-tri) Omitting or mistyping these lines means that no tests get
# run, and nobody notices.  We should probably have drake_py_unittest macro
# that takes care of this, to be less brittle.
if __name__ == '__main__':
    unittest.main()
