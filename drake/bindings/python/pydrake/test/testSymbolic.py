from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake.symbolic as sym


class TestSymbolicVariables(unittest.TestCase):
    def testOperators(self):
        x = sym.Variable("x")
        self.assertEqual(str(x), "x")

        y = sym.Variable("y")
        self.assertEqual(str(y), "y")

        self.assertEqual(str(x + y), "(x + y)")
        self.assertEqual(str(x - y), "(x - y)")
        self.assertEqual(str(x * y), "(x * y)")
        self.assertEqual(str(x / y), "(x / y)")
        self.assertEqual(str((x + y) * x), "(x * (x + y))")

    def testSimplify(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        self.assertEqual(str(0 * (x + y)), "0")
        self.assertEqual(str(x + y - x - y), "0")
        self.assertEqual(str(x / x - 1), "0")
        self.assertEqual(str(x / x), "1")

    def testFormula(self):
        x = sym.Variable("x")
        y = sym.Variable("y")

        self.assertEqual(str(x < y), "(x < y)")
        self.assertEqual(str(x <= y), "(x <= y)")
        self.assertEqual(str(x > y), "(x > y)")
        self.assertEqual(str(x >= y), "(x >= y)")
        self.assertEqual(str(x == y), "(x = y)")



if __name__ == '__main__':
    unittest.main()
