"""..."""

import unittest


class TestRulesPythonInternalVendoring(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_foo(self):
        self.assertEqual(1, 0)
