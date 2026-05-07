import pydrake.planning.experimental as mut  # ruff: isort: skip

import unittest


class TestPlaceholder(unittest.TestCase):
    def test_placeholder(self):
        dut = mut.Placeholder()
        self.assertIsNotNone(dut)
