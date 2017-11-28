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

    def testExpand(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        ex = 2 * (x + y)
        self.assertEqual(str(ex), "(2 * (x + y))")
        self.assertEqual(str(ex.Expand()), "(2 * x + 2 * y)")

    def testPow(self):
        x = sym.Variable("x")
        self.assertEqual(str(x**2), "pow(x, 2)")
        y = sym.Variable("y")
        self.assertEqual(str(x**y), "pow(x, y)")
        self.assertEqual(str((x + 1)**(y - 1)), "pow((1 + x), (-1 + y))")

    def testNeg(self):
        x = sym.Variable("x")
        self.assertEqual(str(-x), "(-1 * x)")
        self.assertEqual(str(-(x + 1)), "(-1 - x)")

    def testLogical(self):
        x = sym.Variable("x")
        self.assertEqual(str(sym.logical_not(x == 0)),
                         "!((x = 0))")
        # Test single-operand logical statements
        self.assertEqual(str(sym.logical_and(x >= 1)), "(x >= 1)")
        self.assertEqual(str(sym.logical_or(x >= 1)), "(x >= 1)")
        # Test binary operand logical statements
        self.assertEqual(str(sym.logical_and(x >= 1, x <= 2)),
                         "((x >= 1) and (x <= 2))")
        self.assertEqual(str(sym.logical_or(x <= 1, x >= 2)),
                         "((x >= 2) or (x <= 1))")
        # Test multiple operand logical statements
        y = sym.Variable("y")
        self.assertEqual(str(sym.logical_and(x >= 1, x <= 2, y == 2)),
                         "((y = 2) and (x >= 1) and (x <= 2))")
        self.assertEqual(str(sym.logical_or(x >= 1, x <= 2, y == 2)),
                         "((y = 2) or (x >= 1) or (x <= 2))")


if __name__ == '__main__':
    unittest.main()
