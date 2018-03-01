# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake.symbolic as sym


# Define global variables to make the tests less verbose.
x = sym.Variable("x")
y = sym.Variable("y")
z = sym.Variable("z")
a = sym.Variable("a")
b = sym.Variable("b")
c = sym.Variable("c")


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


class TestSymbolicExpression(unittest.TestCase):
    def test_functions_with_float(self):
        v_x = 1.0
        v_y = 1.0
        self.assertEqual(sym.abs(v_x), np.abs(v_x))
        self.assertEqual(sym.exp(v_x), np.exp(v_x))
        self.assertEqual(sym.sqrt(v_x), np.sqrt(v_x))
        self.assertEqual(sym.pow(v_x, v_y), v_x ** v_y)
        self.assertEqual(sym.sin(v_x), np.sin(v_x))
        self.assertEqual(sym.cos(v_x), np.cos(v_x))
        self.assertEqual(sym.tan(v_x), np.tan(v_x))
        self.assertEqual(sym.asin(v_x), np.arcsin(v_x))
        self.assertEqual(sym.acos(v_x), np.arccos(v_x))
        self.assertEqual(sym.atan(v_x), np.arctan(v_x))
        self.assertEqual(sym.atan2(v_x, v_y), np.arctan2(v_x, v_y))
        self.assertEqual(sym.sinh(v_x), np.sinh(v_x))
        self.assertEqual(sym.cosh(v_x), np.cosh(v_x))
        self.assertEqual(sym.tanh(v_x), np.tanh(v_x))
        self.assertEqual(sym.min(v_x, v_y), min(v_x, v_y))
        self.assertEqual(sym.max(v_x, v_y), max(v_x, v_y))
        self.assertEqual(sym.ceil(v_x), np.ceil(v_x))
        self.assertEqual(sym.floor(v_x), np.floor(v_x))
        self.assertEqual(
          sym.if_then_else(
            sym.Expression(v_x) > sym.Expression(v_y),
            v_x, v_y),
          v_x if v_x > v_y else v_y)

    def test_functions_with_variable(self):
        self.assertEqual(str(sym.abs(x)), "abs(x)")
        self.assertEqual(str(sym.exp(x)), "exp(x)")
        self.assertEqual(str(sym.sqrt(x)), "sqrt(x)")
        self.assertEqual(str(sym.pow(x, y)), "pow(x, y)")
        self.assertEqual(str(sym.sin(x)), "sin(x)")
        self.assertEqual(str(sym.cos(x)), "cos(x)")
        self.assertEqual(str(sym.tan(x)), "tan(x)")
        self.assertEqual(str(sym.asin(x)), "asin(x)")
        self.assertEqual(str(sym.acos(x)), "acos(x)")
        self.assertEqual(str(sym.atan(x)), "atan(x)")
        self.assertEqual(str(sym.atan2(x, y)), "atan2(x, y)")
        self.assertEqual(str(sym.sinh(x)), "sinh(x)")
        self.assertEqual(str(sym.cosh(x)), "cosh(x)")
        self.assertEqual(str(sym.tanh(x)), "tanh(x)")
        self.assertEqual(str(sym.min(x, y)), "min(x, y)")
        self.assertEqual(str(sym.max(x, y)), "max(x, y)")
        self.assertEqual(str(sym.ceil(x)), "ceil(x)")
        self.assertEqual(str(sym.floor(x)), "floor(x)")
        self.assertEqual(str(sym.if_then_else(x > y, x, y)),
                         "(if (x > y) then x else y)")

    def test_functions_with_expression(self):
        e_x = sym.Expression(x)
        e_y = sym.Expression(y)
        self.assertEqual(str(sym.abs(e_x)), "abs(x)")
        self.assertEqual(str(sym.exp(e_x)), "exp(x)")
        self.assertEqual(str(sym.sqrt(e_x)), "sqrt(x)")
        self.assertEqual(str(sym.pow(e_x, e_y)), "pow(x, y)")
        self.assertEqual(str(sym.sin(e_x)), "sin(x)")
        self.assertEqual(str(sym.cos(e_x)), "cos(x)")
        self.assertEqual(str(sym.tan(e_x)), "tan(x)")
        self.assertEqual(str(sym.asin(e_x)), "asin(x)")
        self.assertEqual(str(sym.acos(e_x)), "acos(x)")
        self.assertEqual(str(sym.atan(e_x)), "atan(x)")
        self.assertEqual(str(sym.atan2(e_x, e_y)), "atan2(x, y)")
        self.assertEqual(str(sym.sinh(e_x)), "sinh(x)")
        self.assertEqual(str(sym.cosh(e_x)), "cosh(x)")
        self.assertEqual(str(sym.tanh(e_x)), "tanh(x)")
        self.assertEqual(str(sym.min(e_x, e_y)), "min(x, y)")
        self.assertEqual(str(sym.max(e_x, e_y)), "max(x, y)")
        self.assertEqual(str(sym.ceil(e_x)), "ceil(x)")
        self.assertEqual(str(sym.floor(e_x)), "floor(x)")
        self.assertEqual(str(sym.if_then_else(e_x > e_y, e_x, e_y)),
                         "(if (x > y) then x else y)")


class TestSymbolicPolynomial(unittest.TestCase):
    def test_default_constructor(self):
        p = sym.Polynomial()
        self.assertEqual(p.ToExpression(), sym.Expression())

    def test_constructor_maptype(self):
        m = {sym.Monomial(x): sym.Expression(3),
             sym.Monomial(y): sym.Expression(2)}  # 3x + 2y
        p = sym.Polynomial(m)
        expected = 3 * x + 2 * y
        self.assertEqual(p.ToExpression(), expected)

    def test_constructor_expression(self):
        e = 2 * x + 3 * y
        p = sym.Polynomial(e)
        self.assertEqual(p.ToExpression(), e)

    def test_constructor_expression_indeterminates(self):
        e = a * x + b * y + c * z
        p = sym.Polynomial(e, sym.Variables([x, y, z]))
        decision_vars = sym.Variables([a, b, c])
        indeterminates = sym.Variables([x, y, z])
        self.assertEqual(p.indeterminates(), indeterminates)
        self.assertEqual(p.decision_variables(), decision_vars)

    def test_degree_total_degree(self):
        e = a * (x ** 2) + b * (y ** 3) + c * z
        p = sym.Polynomial(e, [x, y, z])
        self.assertEqual(p.Degree(x), 2)
        self.assertEqual(p.TotalDegree(), 3)

    def test_monomial_to_coefficient_map(self):
        m = sym.Monomial(x, 2)
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])
        the_map = p.monomial_to_coefficient_map()
        self.assertEqual(the_map[m], a)

    def test_differentiate(self):
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])  # p = ax²
        result = p.Differentiate(x)  # = 2ax
        self.assertEqual(result.ToExpression(), 2 * a * x)

    def test_add_product(self):
        p = sym.Polynomial()
        m = sym.Monomial(x)
        p.AddProduct(sym.Expression(3), m)  # p += 3 * x
        self.assertEqual(p.ToExpression(), 3 * x)

    def test_comparison(self):
        p = sym.Polynomial()
        self.assertTrue(p == p)
        self.assertTrue(p.EqualTo(p))

    def test_repr(self):
        p = sym.Polynomial()
        self.assertEqual(str(p), "0")

    def test_addition(self):
        p = sym.Polynomial()
        self.assertEqual(p + p, p)
        m = sym.Monomial(x)
        self.assertEqual(m + p, sym.Polynomial(1 * x))
        self.assertEqual(p + m, sym.Polynomial(1 * x))
        self.assertEqual(p + 0, p)
        self.assertEqual(0 + p, p)

    def test_subtraction(self):
        p = sym.Polynomial()
        self.assertEqual(p - p, p)
        m = sym.Monomial(x)
        self.assertEqual(m - p, sym.Polynomial(1 * x))
        self.assertEqual(p - m, sym.Polynomial(-1 * x))
        self.assertEqual(p - 0, p)
        self.assertEqual(0 - p, -p)

    def test_multiplication(self):
        p = sym.Polynomial()
        self.assertEqual(p * p, p)
        m = sym.Monomial(x)
        self.assertEqual(m * p, p)
        self.assertEqual(p * m, p)
        self.assertEqual(p * 0, p)
        self.assertEqual(0 * p, p)

    def test_addition_assignment(self):
        p = sym.Polynomial()
        p += p
        self.assertEqual(p, sym.Polynomial())
        p += sym.Monomial(x)
        self.assertEqual(p, sym.Polynomial(1 * x))
        p += 3
        self.assertEqual(p, sym.Polynomial(1 * x + 3))

    def test_subtraction_assignment(self):
        p = sym.Polynomial()
        p -= p
        self.assertEqual(p, sym.Polynomial())
        p -= sym.Monomial(x)
        self.assertEqual(p, sym.Polynomial(-1 * x))
        p -= 3
        self.assertEqual(p, sym.Polynomial(-1 * x - 3))

    def test_multiplication_assignment(self):
        p = sym.Polynomial()
        p *= p
        self.assertEqual(p, sym.Polynomial())
        p *= sym.Monomial(x)
        self.assertEqual(p, sym.Polynomial())
        p *= 3
        self.assertEqual(p, sym.Polynomial())

    def test_pow(self):
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])  # p = ax²
        p = pow(p, 2)  # p = a²x⁴
        self.assertEqual(p.ToExpression(), (a ** 2) * (x ** 4))


if __name__ == '__main__':
    unittest.main()
