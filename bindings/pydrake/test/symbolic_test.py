# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake.symbolic as sym
from pydrake.test.algebra_test_util import ScalarAlgebra, VectorizedAlgebra
from pydrake.util.containers import EqualToDict
from copy import copy


# TODO(eric.cousineau): Replace usages of `sym` math functions with the
# overloads from `pydrake.math`.

# Define global variables to make the tests less verbose.
x = sym.Variable("x")
y = sym.Variable("y")
z = sym.Variable("z")
w = sym.Variable("w")
a = sym.Variable("a")
b = sym.Variable("b")
c = sym.Variable("c")
e_x = sym.Expression(x)
e_y = sym.Expression(y)

TYPES = [
    sym.Variable,
    sym.Expression,
    sym.Polynomial,
    sym.Monomial,
]

RHS_TYPES = TYPES + [float, np.float64]


class SymbolicTestCase(unittest.TestCase):
    def _check_operand_types(self, lhs, rhs):
        self.assertTrue(type(lhs) in TYPES, type(lhs))
        self.assertTrue(type(rhs) in RHS_TYPES, type(rhs))

    def assertEqualStructure(self, lhs, rhs):
        self._check_operand_types(lhs, rhs)
        self.assertTrue(lhs.EqualTo(rhs), "{} != {}".format(lhs, rhs))

    def assertNotEqualStructure(self, lhs, rhs):
        self._check_operand_types(lhs, rhs)
        self.assertFalse(lhs.EqualTo(rhs), "{} == {}".format(lhs, rhs))


class TestSymbolicVariable(SymbolicTestCase):
    def test_addition(self):
        self.assertEqual(str(x + y), "(x + y)")
        self.assertEqual(str(x + 1), "(1 + x)")
        self.assertEqual(str(1 + x), "(1 + x)")

    def test_subtraction(self):
        self.assertEqual(str(x - y), "(x - y)")
        self.assertEqual(str(x - 1), "(-1 + x)")
        self.assertEqual(str(1 - x), "(1 - x)")

    def test_multiplication(self):
        self.assertEqual(str(x * y), "(x * y)")
        self.assertEqual(str(x * 1), "x")
        self.assertEqual(str(1 * x), "x")

    def test_division(self):
        self.assertEqual(str(x / y), "(x / y)")
        self.assertEqual(str(x / 1), "x")
        self.assertEqual(str(1 / x), "(1 / x)")

    def test_unary_operators(self):
        self.assertEqual(str(+x), "x")
        self.assertEqual(str(-x), "(-1 * x)")

    def test_relational_operators(self):
        # Variable rop float
        self.assertEqual(str(x >= 1), "(x >= 1)")
        self.assertEqual(str(x > 1), "(x > 1)")
        self.assertEqual(str(x <= 1), "(x <= 1)")
        self.assertEqual(str(x < 1), "(x < 1)")
        self.assertEqual(str(x == 1), "(x = 1)")
        self.assertEqual(str(x != 1), "(x != 1)")

        # float rop Variable
        self.assertEqual(str(1 < y), "(y > 1)")
        self.assertEqual(str(1 <= y), "(y >= 1)")
        self.assertEqual(str(1 > y), "(y < 1)")
        self.assertEqual(str(1 >= y), "(y <= 1)")
        self.assertEqual(str(1 == y), "(y = 1)")
        self.assertEqual(str(1 != y), "(y != 1)")

        # Variable rop Variable
        self.assertEqual(str(x < y), "(x < y)")
        self.assertEqual(str(x <= y), "(x <= y)")
        self.assertEqual(str(x > y), "(x > y)")
        self.assertEqual(str(x >= y), "(x >= y)")
        self.assertEqual(str(x == y), "(x = y)")
        self.assertEqual(str(x != y), "(x != y)")

    def test_repr(self):
        self.assertEqual(repr(x), "Variable('x')")

    def test_simplify(self):
        self.assertEqual(str(0 * (x + y)), "0")
        self.assertEqual(str(x + y - x - y), "0")
        self.assertEqual(str(x / x - 1), "0")
        self.assertEqual(str(x / x), "1")

    def test_expand(self):
        ex = 2 * (x + y)
        self.assertEqual(str(ex), "(2 * (x + y))")
        self.assertEqual(str(ex.Expand()), "(2 * x + 2 * y)")

    def test_pow(self):
        self.assertEqual(str(x**2), "pow(x, 2)")
        self.assertEqual(str(x**y), "pow(x, y)")
        self.assertEqual(str((x + 1)**(y - 1)), "pow((1 + x), (-1 + y))")

    def test_neg(self):
        self.assertEqual(str(-(x + 1)), "(-1 - x)")

    def test_equalto(self):
        self.assertTrue(x.EqualTo(x))
        self.assertFalse(x.EqualTo(y))

    def test_logical(self):
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
        self.assertEqual(str(sym.logical_and(x >= 1, x <= 2, y == 2)),
                         "((y = 2) and (x >= 1) and (x <= 2))")
        self.assertEqual(str(sym.logical_or(x >= 1, x <= 2, y == 2)),
                         "((y = 2) or (x >= 1) or (x <= 2))")

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

    def test_array_str(self):
        # Addresses #8729.
        value = str(np.array([x, y]))
        self.assertIn("Variable('x')", value)
        self.assertIn("Variable('y')", value)


class TestSymbolicVariables(SymbolicTestCase):
    def test_default_constructor(self):
        vars = sym.Variables()
        self.assertEqual(vars.size(), 0)
        self.assertTrue(vars.empty())

    def test_constructor_list(self):
        vars = sym.Variables([x, y, z])
        self.assertEqual(vars.size(), 3)
        self.assertEqual(len(vars), 3)

    def test_to_string(self):
        vars = sym.Variables([x, y, z])
        self.assertEqual(vars.to_string(), "{x, y, z}")
        self.assertEqual("{}".format(vars), "{x, y, z}")

    def test_repr(self):
        vars = sym.Variables([x, y, z])
        self.assertEqual(repr(vars), '<Variables "{x, y, z}">')

    def test_insert1(self):
        vars = sym.Variables()
        vars.insert(x)
        self.assertEqual(vars.size(), 1)

    def test_insert2(self):
        vars = sym.Variables([x])
        vars.insert(sym.Variables([y, z]))
        self.assertEqual(vars.size(), 3)

    def test_erase1(self):
        vars = sym.Variables([x, y, z])
        count = vars.erase(x)
        self.assertEqual(count, 1)

    def test_erase2(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([w, z])
        count = vars1.erase(vars2)
        self.assertEqual(count, 1)
        self.assertEqual(vars1.size(), 2)

    def test_include(self):
        vars = sym.Variables([x, y, z])
        self.assertTrue(vars.include(y))
        self.assertTrue(z in vars)

    def test_equalto(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([y, z])
        vars3 = sym.Variables([x, y, z])
        self.assertTrue(vars1.EqualTo(vars3))
        self.assertFalse(vars1.EqualTo(vars2))

    def test_to_string(self):
        vars = sym.Variables()
        vars.insert(x)
        vars.insert(y)
        vars.insert(z)
        self.assertEqual(vars.to_string(), "{x, y, z}")

    def test_subset_properties(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([x, y])
        self.assertFalse(vars1.IsSubsetOf(vars2))
        self.assertFalse(vars1.IsStrictSubsetOf(vars2))
        self.assertTrue(vars1.IsSupersetOf(vars2))
        self.assertTrue(vars1.IsStrictSupersetOf(vars2))

    def test_eq(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([x, y])
        self.assertFalse(vars1 == vars2)

    def test_lt(self):
        vars1 = sym.Variables([x, y])
        vars2 = sym.Variables([x, y, z])
        self.assertTrue(vars1 < vars2)

    def test_add(self):
        vars1 = sym.Variables([x, y])
        vars2 = sym.Variables([y, z])
        vars3 = vars1 + vars2  # [x, y, z]
        self.assertEqual(vars3.size(), 3)
        vars4 = vars1 + z  # [x, y, z]
        self.assertEqual(vars4.size(), 3)
        vars5 = x + vars1  # [x, y]
        self.assertEqual(vars5.size(), 2)

    def test_add_assignment(self):
        vars = sym.Variables([x])
        vars += y
        self.assertEqual(vars.size(), 2)
        vars += sym.Variables([x, z])
        self.assertEqual(vars.size(), 3)

    def test_sub(self):
        vars1 = sym.Variables([x, y])
        vars2 = sym.Variables([y, z])
        vars3 = vars1 - vars2  # [x]
        self.assertEqual(vars3, sym.Variables([x]))
        vars4 = vars1 - y  # [x]
        self.assertEqual(vars4, sym.Variables([x]))

    def test_sub_assignment(self):
        vars = sym.Variables([x, y, z])
        vars -= y  # = [x, z]
        self.assertEqual(vars, sym.Variables([x, z]))
        vars -= sym.Variables([x])  # = [z]
        self.assertEqual(vars, sym.Variables([z]))

    def test_intersect(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([y, w])
        vars3 = sym.intersect(vars1, vars2)  # = [y]
        self.assertEqual(vars3, sym.Variables([y]))

    def test_iterable(self):
        vars = sym.Variables([x, y, z])
        count = 0
        for var in vars:
            self.assertTrue(var in vars)
            count = count + 1
        self.assertEqual(count, 3)


class TestSymbolicExpression(SymbolicTestCase):
    def _check_scalar(self, actual, expected):
        self.assertIsInstance(actual, sym.Expression)
        # Chain conversion to ensure equivalent treatment.
        if isinstance(expected, float) or isinstance(expected, int):
            expected = sym.Expression(expected)
        if isinstance(expected, sym.Expression):
            expected = str(expected)
        self.assertIsInstance(expected, str)
        self.assertEqual(str(actual), expected)

    def _check_array(self, actual, expected):
        self.assertEqual(actual.shape, expected.shape)
        for a, b in zip(actual.flat, expected.flat):
            self._check_scalar(a, b)

    def _check_algebra(self, algebra):
        xv = algebra.to_algebra(x)
        yv = algebra.to_algebra(y)
        zv = algebra.to_algebra(z)
        wv = algebra.to_algebra(w)
        av = algebra.to_algebra(a)
        bv = algebra.to_algebra(b)
        cv = algebra.to_algebra(c)
        e_xv = algebra.to_algebra(e_x)
        e_yv = algebra.to_algebra(e_y)

        # Addition.
        algebra.check_value(e_xv + e_yv, "(x + y)")
        algebra.check_value(e_xv + yv, "(x + y)")
        algebra.check_value(e_xv + 1, "(1 + x)")
        algebra.check_value(xv + e_yv, "(x + y)")
        algebra.check_value(1 + e_xv, "(1 + x)")

        # - In place.
        e = copy(xv)
        e += e_yv
        algebra.check_value(e, "(x + y)")
        e += zv
        algebra.check_value(e, "(x + y + z)")
        e += 1
        algebra.check_value(e, "(1 + x + y + z)")

        # Subtraction.
        algebra.check_value((e_xv - e_yv), "(x - y)")
        algebra.check_value((e_xv - yv), "(x - y)")
        algebra.check_value((e_xv - 1), "(-1 + x)")
        algebra.check_value((xv - e_yv), "(x - y)")
        algebra.check_value((1 - e_xv), "(1 - x)")

        # - In place.
        e = copy(xv)
        e -= e_yv
        algebra.check_value(e, (x - y))
        e -= zv
        algebra.check_value(e, (x - y - z))
        e -= 1
        algebra.check_value(e, (x - y - z - 1))

        # Multiplication.
        algebra.check_value((e_xv * e_yv), "(x * y)")
        algebra.check_value((e_xv * yv), "(x * y)")
        algebra.check_value((e_xv * 1), "x")
        algebra.check_value((xv * e_yv), "(x * y)")
        algebra.check_value((1 * e_xv), "x")

        # - In place.
        e = copy(xv)
        e *= e_yv
        algebra.check_value(e, "(x * y)")
        e *= zv
        algebra.check_value(e, "(x * y * z)")
        e *= 1
        algebra.check_value(e, "(x * y * z)")

        # Division
        algebra.check_value((e_xv / e_yv), (x / y))
        algebra.check_value((e_xv / yv), (x / y))
        algebra.check_value((e_xv / 1), "x")
        algebra.check_value((xv / e_yv), (x / y))
        algebra.check_value((1 / e_xv), (1 / x))

        # - In place.
        e = copy(xv)
        e /= e_yv
        algebra.check_value(e, (x / y))
        e /= zv
        algebra.check_value(e, (x / y / z))
        e /= 1
        algebra.check_value(e, ((x / y) / z))

        # Unary
        algebra.check_value((+e_xv), "x")
        algebra.check_value((-e_xv), "(-1 * x)")

        # Math functions.
        algebra.check_value((algebra.abs(e_xv)), "abs(x)")
        algebra.check_value((algebra.exp(e_xv)), "exp(x)")
        algebra.check_value((algebra.sqrt(e_xv)), "sqrt(x)")
        algebra.check_value((algebra.pow(e_xv, e_yv)), "pow(x, y)")
        algebra.check_value((algebra.sin(e_xv)), "sin(x)")
        algebra.check_value((algebra.cos(e_xv)), "cos(x)")
        algebra.check_value((algebra.tan(e_xv)), "tan(x)")
        algebra.check_value((algebra.arcsin(e_xv)), "asin(x)")
        algebra.check_value((algebra.arccos(e_xv)), "acos(x)")
        algebra.check_value((algebra.arctan2(e_xv, e_yv)), "atan2(x, y)")
        algebra.check_value((algebra.sinh(e_xv)), "sinh(x)")
        algebra.check_value((algebra.cosh(e_xv)), "cosh(x)")
        algebra.check_value((algebra.tanh(e_xv)), "tanh(x)")
        algebra.check_value((algebra.ceil(e_xv)), "ceil(x)")
        algebra.check_value((algebra.floor(e_xv)), "floor(x)")

        if isinstance(algebra, ScalarAlgebra):
            # TODO(eric.cousineau): Uncomment these lines if we can teach numpy
            # that reduction is not just selection.
            algebra.check_value((algebra.min(e_xv, e_yv)), "min(x, y)")
            algebra.check_value((algebra.max(e_xv, e_yv)), "max(x, y)")
            # TODO(eric.cousineau): Add broadcasting functions for these
            # operations.
            algebra.check_value((sym.atan(e_xv)), "atan(x)")
            algebra.check_value((sym.if_then_else(e_xv > e_yv, e_xv, e_yv)),
                                "(if (x > y) then x else y)")

        return xv, e_xv

    def test_scalar_algebra(self):
        xv, e_xv = self._check_algebra(
            ScalarAlgebra(
                self._check_scalar, scalar_to_float=lambda x: x.Evaluate()))
        self.assertIsInstance(xv, sym.Variable)
        self.assertIsInstance(e_xv, sym.Expression)

    def test_array_algebra(self):
        xv, e_xv = self._check_algebra(
            VectorizedAlgebra(
                self._check_array,
                scalar_to_float=lambda x: x.Evaluate()))
        self.assertEqual(xv.shape, (2,))
        self.assertIsInstance(xv[0], sym.Variable)
        self.assertEqual(e_xv.shape, (2,))
        self.assertIsInstance(e_xv[0], sym.Expression)

    def test_equalto(self):
        self.assertTrue((x + y).EqualTo(x + y))
        self.assertFalse((x + y).EqualTo(x - y))

    def test_relational_operators(self):
        # TODO(eric.cousineau): Use `VectorizedAlgebra` overloads once #8315 is
        # resolved.
        # Expression rop Expression
        self.assertEqual(str(e_x < e_y), "(x < y)")
        self.assertEqual(str(e_x <= e_y), "(x <= y)")
        self.assertEqual(str(e_x > e_y), "(x > y)")
        self.assertEqual(str(e_x >= e_y), "(x >= y)")
        self.assertEqual(str(e_x == e_y), "(x = y)")
        self.assertEqual(str(e_x != e_y), "(x != y)")

        # Expression rop Variable
        self.assertEqual(str(e_x < y), "(x < y)")
        self.assertEqual(str(e_x <= y), "(x <= y)")
        self.assertEqual(str(e_x > y), "(x > y)")
        self.assertEqual(str(e_x >= y), "(x >= y)")
        self.assertEqual(str(e_x == y), "(x = y)")
        self.assertEqual(str(e_x != y), "(x != y)")

        # Variable rop Expression
        self.assertEqual(str(x < e_y), "(x < y)")
        self.assertEqual(str(x <= e_y), "(x <= y)")
        self.assertEqual(str(x > e_y), "(x > y)")
        self.assertEqual(str(x >= e_y), "(x >= y)")
        self.assertEqual(str(x == e_y), "(x = y)")
        self.assertEqual(str(x != e_y), "(x != y)")

        # Expression rop float
        self.assertEqual(str(e_x < 1), "(x < 1)")
        self.assertEqual(str(e_x <= 1), "(x <= 1)")
        self.assertEqual(str(e_x > 1), "(x > 1)")
        self.assertEqual(str(e_x >= 1), "(x >= 1)")
        self.assertEqual(str(e_x == 1), "(x = 1)")
        self.assertEqual(str(e_x != 1), "(x != 1)")

        # float rop Expression
        self.assertEqual(str(1 < e_y), "(y > 1)")
        self.assertEqual(str(1 <= e_y), "(y >= 1)")
        self.assertEqual(str(1 > e_y), "(y < 1)")
        self.assertEqual(str(1 >= e_y), "(y <= 1)")
        self.assertEqual(str(1 == e_y), "(y = 1)")
        self.assertEqual(str(1 != e_y), "(y != 1)")

    def test_relational_operators_nonzero(self):
        # For issues #8135 and #8491.
        # Ensure that we throw on `__nonzero__`.
        with self.assertRaises(RuntimeError) as cm:
            value = bool(e_x == e_x)
        message = cm.exception.message
        self.assertTrue(
            all([s in message for s in ["__nonzero__", "EqualToDict"]]),
            message)
        # Ensure that compound formulas fail (#8536).
        with self.assertRaises(RuntimeError):
            value = 0 < e_y < e_y
        # Indication of #8135. Ideally, these would all be arrays of formulas.
        e_xv = np.array([e_x, e_x])
        e_yv = np.array([e_y, e_y])
        # N.B. In some versions of NumPy, `!=` for dtype=object implies ID
        # comparison (e.g. `is`).
        # N.B. If `__nonzero__` throws, then NumPy swallows the error and
        # produces a DeprecationWarning, in addition to effectively garbage
        # values. For this reason, `pydrake.symbolic` will automatically
        # promote these warnings to errors.
        # - All false.
        with self.assertRaises(DeprecationWarning):
            value = (e_xv == e_yv)
        # - True + False.
        with self.assertRaises(DeprecationWarning):
            e_xyv = np.array([e_x, e_y])
            value = (e_xv == e_xyv)
        # - All true.
        with self.assertRaises(DeprecationWarning):
            value = (e_xv == e_xv)

    def test_functions_with_float(self):
        # TODO(eric.cousineau): Use concrete values once vectorized methods are
        # supported.
        v_x = 1.0
        v_y = 1.0
        self.assertEqualStructure(sym.abs(v_x), np.abs(v_x))
        self.assertNotEqualStructure(sym.abs(v_x), 0.5*np.abs(v_x))
        self._check_scalar(sym.abs(v_x), np.abs(v_x))
        self._check_scalar(sym.abs(v_x), np.abs(v_x))
        self._check_scalar(sym.exp(v_x), np.exp(v_x))
        self._check_scalar(sym.sqrt(v_x), np.sqrt(v_x))
        self._check_scalar(sym.pow(v_x, v_y), v_x ** v_y)
        self._check_scalar(sym.sin(v_x), np.sin(v_x))
        self._check_scalar(sym.cos(v_x), np.cos(v_x))
        self._check_scalar(sym.tan(v_x), np.tan(v_x))
        self._check_scalar(sym.asin(v_x), np.arcsin(v_x))
        self._check_scalar(sym.acos(v_x), np.arccos(v_x))
        self._check_scalar(sym.atan(v_x), np.arctan(v_x))
        self._check_scalar(sym.atan2(v_x, v_y), np.arctan2(v_x, v_y))
        self._check_scalar(sym.sinh(v_x), np.sinh(v_x))
        self._check_scalar(sym.cosh(v_x), np.cosh(v_x))
        self._check_scalar(sym.tanh(v_x), np.tanh(v_x))
        self._check_scalar(sym.min(v_x, v_y), min(v_x, v_y))
        self._check_scalar(sym.max(v_x, v_y), max(v_x, v_y))
        self._check_scalar(sym.ceil(v_x), np.ceil(v_x))
        self._check_scalar(sym.floor(v_x), np.floor(v_x))
        self._check_scalar(
            sym.if_then_else(
                sym.Expression(v_x) > sym.Expression(v_y),
                v_x, v_y),
            v_x if v_x > v_y else v_y)

    def test_non_method_jacobian(self):
        # Jacobian([x * cos(y), x * sin(y), x ** 2], [x, y]) returns
        # the following 3x2 matrix:
        #
        #  = |cos(y)   -x * sin(y)|
        #    |sin(y)    x * cos(y)|
        #    | 2 * x             0|
        J = sym.Jacobian([x * sym.cos(y), x * sym.sin(y), x ** 2], [x, y])
        self._check_scalar(J[0, 0], sym.cos(y))
        self._check_scalar(J[1, 0], sym.sin(y))
        self._check_scalar(J[2, 0], 2 * x)
        self._check_scalar(J[0, 1], - x * sym.sin(y))
        self._check_scalar(J[1, 1], x * sym.cos(y))
        self._check_scalar(J[2, 1], 0)

    def test_method_jacobian(self):
        # (x * cos(y)).Jacobian([x, y]) returns [cos(y), -x * sin(y)].
        J = (x * sym.cos(y)).Jacobian([x, y])
        self._check_scalar(J[0], sym.cos(y))
        self._check_scalar(J[1], -x * sym.sin(y))

    def test_differentiate(self):
        e = x * x
        self._check_scalar(e.Differentiate(x), 2 * x)

    def test_repr(self):
        self.assertEqual(repr(e_x), '<Expression "x">')

    def test_evaluate(self):
        env = {x: 3.0,
               y: 4.0}
        self.assertEqual((x + y).Evaluate(env),
                         env[x] + env[y])

    def test_evaluate_partial(self):
        env = {x: 3.0,
               y: 4.0}
        partial_evaluated = (x + y + z).EvaluatePartial(env)
        expected = env[x] + env[y] + z
        self.assertTrue(partial_evaluated.EqualTo(expected))

    def test_evaluate_exception_np_nan(self):
        env = {x: np.nan}
        with self.assertRaises(RuntimeError):
            (x + 1).Evaluate(env)

    def test_evaluate_exception_python_nan(self):
        env = {x: float('nan')}
        with self.assertRaises(RuntimeError):
            (x + 1).Evaluate(env)

    def test_substitute_with_pair(self):
        e = x + y
        self.assertEqualStructure(e.Substitute(x, x + 5), x + y + 5)
        self.assertEqualStructure(e.Substitute(y, z), x + z)
        self.assertEqualStructure(e.Substitute(y, 3), x + 3)

    def test_substitute_with_dict(self):
        e = x + y
        env = {x: x + 2, y:  y + 3}
        self.assertEqualStructure(e.Substitute(env), x + y + 5)

    # See `math_overloads_test` for more comprehensive checks on math
    # functions.


class TestSymbolicFormula(SymbolicTestCase):
    def test_get_free_variables(self):
        f = x > y
        self.assertEqual(f.GetFreeVariables(), sym.Variables([x, y]))

    def test_substitute_with_pair(self):
        f = x > y
        self.assertEqual(f.Substitute(y, y + 5), x > y + 5)
        self.assertEqual(f.Substitute(y, z), x > z)
        self.assertEqual(f.Substitute(y, 3), x > 3)

    def test_substitute_with_dict(self):
        f = x + y > z
        self.assertEqual(f.Substitute({x: x + 2, y:  y + 3}),
                         x + y + 5 > z)

    def test_to_string(self):
        f = x > y
        self.assertEqual(f.to_string(), "(x > y)")
        self.assertEqual("{}".format(f), "(x > y)")

    def test_equality_inequality_hash(self):
        f1 = x > y
        f2 = x > y
        f3 = x >= y
        self.assertTrue(f1.EqualTo(f2))
        self.assertEqual(hash(f1), hash(f2))
        self.assertTrue(f1 == f2)
        self.assertFalse(f1.EqualTo(f3))
        self.assertNotEqual(hash(f1), hash(f3))
        self.assertTrue(f1 != f3)

    def test_static_true_false(self):
        tt = sym.Formula.True()
        ff = sym.Formula.False()
        self.assertEqual(x == x, tt)
        self.assertEqual(x != x, ff)

    def test_repr(self):
        self.assertEqual(repr(x > y), '<Formula "(x > y)">')

    def test_evaluate(self):
        env = {x: 3.0,
               y: 4.0}
        self.assertEqual((x > y).Evaluate(env),
                         env[x] > env[y])

    def test_evaluate_exception_np_nan(self):
        env = {x: np.nan}
        with self.assertRaises(RuntimeError):
            (x > 1).Evaluate(env)

    def test_evaluate_exception_python_nan(self):
        env = {x: float('nan')}
        with self.assertRaises(RuntimeError):
            (x > 1).Evaluate(env)


class TestSymbolicMonomial(SymbolicTestCase):
    def test_constructor_variable(self):
        m = sym.Monomial(x)  # m = x¹
        self.assertEqual(m.degree(x), 1)
        self.assertEqual(m.total_degree(), 1)

    def test_constructor_variable_int(self):
        m = sym.Monomial(x, 2)  # m = x²
        self.assertEqual(m.degree(x), 2)
        self.assertEqual(m.total_degree(), 2)

    def test_constructor_map(self):
        powers_in = {x: 2, y: 3, z: 4}
        m = sym.Monomial(powers_in)
        powers_out = EqualToDict(m.get_powers())
        self.assertEqual(powers_out[x], 2)
        self.assertEqual(powers_out[y], 3)
        self.assertEqual(powers_out[z], 4)

    def test_comparison(self):
        # m1 = m2 = x²
        m1 = sym.Monomial(x, 2)
        m2 = sym.Monomial(x, 2)
        m3 = sym.Monomial(x, 1)
        m4 = sym.Monomial(y, 2)
        # Test operator==
        self.assertIsInstance(m1 == m2, bool)
        self.assertTrue(m1 == m2)
        self.assertFalse(m1 == m3)
        self.assertFalse(m1 == m4)
        self.assertFalse(m2 == m3)
        self.assertFalse(m2 == m4)
        self.assertFalse(m3 == m4)
        # Test operator!=
        self.assertIsInstance(m1 != m2, bool)
        self.assertFalse(m1 != m2)
        self.assertTrue(m1 != m3)
        self.assertTrue(m1 != m4)
        self.assertTrue(m2 != m3)
        self.assertTrue(m2 != m4)
        self.assertTrue(m3 != m4)

    def test_equalto(self):
        m1 = sym.Monomial(x, 2)
        m2 = sym.Monomial(x, 1)
        m3 = sym.Monomial(x, 2)
        self.assertTrue(m1.EqualTo(m3))
        self.assertFalse(m1.EqualTo(m2))

    def test_str(self):
        m1 = sym.Monomial(x, 2)
        self.assertEqual(str(m1), "x^2")
        m2 = m1 * sym.Monomial(y)
        self.assertEqual(str(m2), "x^2 * y")

    def test_repr(self):
        m = sym.Monomial(x, 2)
        self.assertEqual(repr(m), '<Monomial "x^2">')

    def test_multiplication1(self):
        m1 = sym.Monomial(x, 2)
        m2 = sym.Monomial(y, 3)
        m3 = m1 * m2
        self.assertEqual(m3.degree(x), 2)
        self.assertEqual(m3.degree(y), 3)

    def test_multiplication2(self):
        m1 = sym.Monomial(x, 2)
        m2 = m1 * sym.Monomial(y)
        m3 = sym.Monomial(y) * m1
        self.assertEqual(m2.degree(x), 2)
        self.assertEqual(m2.degree(y), 1)
        self.assertEqual(m2, m3)

    def test_multiplication_assignment1(self):
        m = sym.Monomial(x, 2)
        m *= sym.Monomial(y, 3)
        self.assertEqual(m.degree(x), 2)
        self.assertEqual(m.degree(y), 3)

    def test_multiplication_assignment2(self):
        m = sym.Monomial(x, 2)
        m *= sym.Monomial(y)
        self.assertEqual(m.degree(x), 2)
        self.assertEqual(m.degree(y), 1)

    def test_pow(self):
        m1 = sym.Monomial(x, 2) * sym.Monomial(y)  # m1 = x²y
        m2 = m1 ** 2                 # m2 = x⁴y²
        self.assertEqual(m2.degree(x), 4)
        self.assertEqual(m2.degree(y), 2)

    def test_pow_in_place(self):
        m1 = sym.Monomial(x, 2) * sym.Monomial(y)  # m1 = x²y
        m2 = m1.pow_in_place(2)      # m1 = m2 = x⁴y²
        self.assertEqual(m1.degree(x), 4)
        self.assertEqual(m1.degree(y), 2)
        self.assertEqual(m2.degree(x), 4)
        self.assertEqual(m2.degree(y), 2)

    def test_get_powers(self):
        m = sym.Monomial(x, 2) * sym.Monomial(y)  # m = x²y
        powers = EqualToDict(m.get_powers())
        self.assertEqual(powers[x], 2)
        self.assertEqual(powers[y], 1)

    def test_to_expression(self):
        m = sym.Monomial(x, 3) * sym.Monomial(y)  # m = x³y
        e = m.ToExpression()
        self.assertEqual(str(e), "(pow(x, 3) * y)")

    def test_get_variables(self):
        m = sym.Monomial(x, 3) * sym.Monomial(y)  # m = x³y
        vars = m.GetVariables()  # = [x, y]
        self.assertEqual(vars.size(), 2)

    def test_monomial_basis(self):
        vars = sym.Variables([x, y, z])
        basis1 = sym.MonomialBasis(vars, 3)
        basis2 = sym.MonomialBasis([x, y, z], 3)
        self.assertEqual(basis1.size, 20)
        self.assertEqual(basis2.size, 20)

    def test_evaluate(self):
        m = sym.Monomial(x, 3) * sym.Monomial(y)  # m = x³y
        env = {x: 2.0,
               y: 3.0}
        self.assertEqual(m.Evaluate(env),
                         env[x] ** 3 * env[y])

    def test_evaluate_exception_np_nan(self):
        m = sym.Monomial(x, 3)
        env = {x: np.nan}
        with self.assertRaises(RuntimeError):
            m.Evaluate(env)

    def test_evaluate_exception_python_nan(self):
        m = sym.Monomial(x, 3)
        env = {x: float('nan')}
        with self.assertRaises(RuntimeError):
            m.Evaluate(env)


class TestSymbolicPolynomial(SymbolicTestCase):
    def test_default_constructor(self):
        p = sym.Polynomial()
        self.assertEqualStructure(p.ToExpression(), sym.Expression())

    def test_constructor_maptype(self):
        m = {sym.Monomial(x): sym.Expression(3),
             sym.Monomial(y): sym.Expression(2)}  # 3x + 2y
        p = sym.Polynomial(m)
        expected = 3 * x + 2 * y
        self.assertEqualStructure(p.ToExpression(), expected)

    def test_constructor_expression(self):
        e = 2 * x + 3 * y
        p = sym.Polynomial(e)
        self.assertEqualStructure(p.ToExpression(), e)

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
        self.assertEqualStructure(the_map[m], a)

    def test_differentiate(self):
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])  # p = ax²
        result = p.Differentiate(x)  # = 2ax
        self.assertEqualStructure(result.ToExpression(), 2 * a * x)

    def test_add_product(self):
        p = sym.Polynomial()
        m = sym.Monomial(x)
        p.AddProduct(sym.Expression(3), m)  # p += 3 * x
        self.assertEqualStructure(p.ToExpression(), 3 * x)

    def test_comparison(self):
        p = sym.Polynomial()
        self.assertEqualStructure(p, p)
        self.assertIsInstance(p == p, sym.Formula)
        self.assertEqual(p == p, sym.Formula.True())
        self.assertTrue(p.EqualTo(p))
        q = sym.Polynomial(sym.Expression(10))
        self.assertNotEqualStructure(p, q)
        self.assertIsInstance(p != q, sym.Formula)
        self.assertEqual(p != q, sym.Formula.True())
        self.assertFalse(p.EqualTo(q))

    def test_repr(self):
        p = sym.Polynomial()
        self.assertEqual(repr(p), '<Polynomial "0">')

    def test_addition(self):
        p = sym.Polynomial()
        self.assertEqualStructure(p + p, p)
        m = sym.Monomial(x)
        self.assertEqualStructure(m + p, sym.Polynomial(1 * x))
        self.assertEqualStructure(p + m, sym.Polynomial(1 * x))
        self.assertEqualStructure(p + 0, p)
        self.assertEqualStructure(0 + p, p)

    def test_subtraction(self):
        p = sym.Polynomial()
        self.assertEqualStructure(p - p, p)
        m = sym.Monomial(x)
        self.assertEqualStructure(m - p, sym.Polynomial(1 * x))
        self.assertEqualStructure(p - m, sym.Polynomial(-1 * x))
        self.assertEqualStructure(p - 0, p)
        self.assertEqualStructure(0 - p, -p)

    def test_multiplication(self):
        p = sym.Polynomial()
        self.assertEqualStructure(p * p, p)
        m = sym.Monomial(x)
        self.assertEqualStructure(m * p, p)
        self.assertEqualStructure(p * m, p)
        self.assertEqualStructure(p * 0, p)
        self.assertEqualStructure(0 * p, p)

    def test_addition_assignment(self):
        p = sym.Polynomial()
        p += p
        self.assertEqualStructure(p, sym.Polynomial())
        p += sym.Monomial(x)
        self.assertEqualStructure(p, sym.Polynomial(1 * x))
        p += 3
        self.assertEqualStructure(p, sym.Polynomial(3 + 1 * x))

    def test_subtraction_assignment(self):
        p = sym.Polynomial()
        p -= p
        self.assertEqualStructure(p, sym.Polynomial())
        p -= sym.Monomial(x)
        self.assertEqualStructure(p, sym.Polynomial(-1 * x))
        p -= 3
        self.assertEqualStructure(p, sym.Polynomial(-1 * x - 3))

    def test_multiplication_assignment(self):
        p = sym.Polynomial()
        p *= p
        self.assertEqualStructure(p, sym.Polynomial())
        p *= sym.Monomial(x)
        self.assertEqualStructure(p, sym.Polynomial())
        p *= 3
        self.assertEqualStructure(p, sym.Polynomial())

    def test_pow(self):
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])  # p = ax²
        p = pow(p, 2)  # p = a²x⁴
        self.assertEqualStructure(p.ToExpression(), (a ** 2) * (x ** 4))

    def test_jacobian(self):
        e = 5 * x ** 2 + 4 * y ** 2 + 8 * x * y
        p = sym.Polynomial(e, [x, y])                  # p = 5x² + 4y² + 8xy
        p_dx = sym.Polynomial(10 * x + 8 * y, [x, y])  # ∂p/∂x = 10x + 8y
        p_dy = sym.Polynomial(8 * y + 8 * x, [x, y])   # ∂p/∂y =  8y + 8x

        J = p.Jacobian([x, y])
        self.assertEqualStructure(J[0], p_dx)
        self.assertEqualStructure(J[1], p_dy)

    def test_hash(self):
        p1 = sym.Polynomial(x * x, [x])
        p2 = sym.Polynomial(x * x, [x])
        self.assertEqualStructure(p1, p2)
        self.assertEqual(hash(p1), hash(p2))
        p1 += 1
        self.assertNotEqualStructure(p1, p2)
        self.assertNotEqual(hash(p1), hash(p2))

    def test_evaluate(self):
        p = sym.Polynomial(a * x * x + b * x + c, [x])
        env = {a: 2.0,
               b: 3.0,
               c: 5.0,
               x: 2.0}
        self.assertEqual(p.Evaluate(env),
                         env[a] * env[x] * env[x] + env[b] * env[x] + env[c])

    def test_evaluate_exception_np_nan(self):
        p = sym.Polynomial(x * x, [x])
        env = {x: np.nan}
        with self.assertRaises(RuntimeError):
            p.Evaluate(env)

    def test_evaluate_exception_python_nan(self):
        p = sym.Polynomial(x * x, [x])
        env = {x: float('nan')}
        with self.assertRaises(RuntimeError):
            p.Evaluate(env)
