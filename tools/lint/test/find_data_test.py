import os
import unittest

from drake.tools.lint.find_data import find_data


class FindDataTest(unittest.TestCase):

    def test_find(self):
        filename = find_data("tools/lint/test/find_data_test.txt")
        self.assertTrue(os.path.exists(filename))
        with open(filename, "r") as contents:
            lines = contents.readlines()
        self.assertListEqual(lines, ["Sample data for unit find_data_test.\n"])
