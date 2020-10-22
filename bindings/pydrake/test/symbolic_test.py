# -*- coding: utf-8 -*-

import copy
import unittest

import numpy as np

import pydrake.symbolic as sym
import pydrake.common
from pydrake.test.algebra_test_util import ScalarAlgebra, VectorizedAlgebra
from pydrake.common.containers import EqualToDict
from pydrake.common.deprecation import install_numpy_warning_filters
from pydrake.common.test_utilities import numpy_compare

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


class TestSymbolicVariable(unittest.TestCase):
    def test_addition(self):
        numpy_compare.assert_equal(x + y, "(x + y)")
        numpy_compare.assert_equal(x + 1, "(1 + x)")
        numpy_compare.assert_equal(1 + x, "(1 + x)")

    def test_subtraction(self):
        numpy_compare.assert_equal(x - y, "(x - y)")
        numpy_compare.assert_equal(x - 1, "(-1 + x)")
        numpy_compare.assert_equal(1 - x, "(1 - x)")

    def test_multiplication(self):
        numpy_compare.assert_equal(x * y, "(x * y)")
        numpy_compare.assert_equal(x * 1, "x")
        numpy_compare.assert_equal(1 * x, "x")

    def test_division(self):
        numpy_compare.assert_equal(x / y, "(x / y)")
        numpy_compare.assert_equal(x / 1, "x")
        numpy_compare.assert_equal(1 / x, "(1 / x)")

    def test_unary_operators(self):
        numpy_compare.assert_equal(+x, "x")
        numpy_compare.assert_equal(-x, "(-1 * x)")

    def test_relational_operators(self):
        # Variable rop float
        numpy_compare.assert_equal(x >= 1, "(x >= 1)")
        numpy_compare.assert_equal(x > 1, "(x > 1)")
        numpy_compare.assert_equal(x <= 1, "(x <= 1)")
        numpy_compare.assert_equal(x < 1, "(x < 1)")
        numpy_compare.assert_equal(x == 1, "(x == 1)")
        numpy_compare.assert_equal(x != 1, "(x != 1)")

        # float rop Variable
        numpy_compare.assert_equal(1 < y, "(y > 1)")
        numpy_compare.assert_equal(1 <= y, "(y >= 1)")
        numpy_compare.assert_equal(1 > y, "(y < 1)")
        numpy_compare.assert_equal(1 >= y, "(y <= 1)")
        numpy_compare.assert_equal(1 == y, "(y == 1)")
        numpy_compare.assert_equal(1 != y, "(y != 1)")

        # Variable rop Variable
        numpy_compare.assert_equal(x < y, "(x < y)")
        numpy_compare.assert_equal(x <= y, "(x <= y)")
        numpy_compare.assert_equal(x > y, "(x > y)")
        numpy_compare.assert_equal(x >= y, "(x >= y)")
        numpy_compare.assert_equal(x == y, "(x == y)")
        numpy_compare.assert_equal(x != y, "(x != y)")

    def test_get_type(self):
        i = sym.Variable('i', sym.Variable.Type.INTEGER)
        self.assertEqual(i.get_type(), sym.Variable.Type.INTEGER)
        g = sym.Variable('g', sym.Variable.Type.RANDOM_GAUSSIAN)
        self.assertEqual(g.get_type(), sym.Variable.Type.RANDOM_GAUSSIAN)

    def test_repr(self):
        self.assertEqual(repr(x), "Variable('x', Continuous)")

    def test_simplify(self):
        numpy_compare.assert_equal(0 * (x + y), "0")
        numpy_compare.assert_equal(x + y - x - y, "0")
        numpy_compare.assert_equal(x / x - 1, "0")
        numpy_compare.assert_equal(x / x, "1")

    def test_expand(self):
        ex = 2 * (x + y)
        numpy_compare.assert_equal(ex, "(2 * (x + y))")
        numpy_compare.assert_equal(ex.Expand(), "(2 * x + 2 * y)")

    def test_pow(self):
        numpy_compare.assert_equal(x**2, "pow(x, 2)")
        numpy_compare.assert_equal(x**y, "pow(x, y)")
        numpy_compare.assert_equal((x + 1)**(y - 1), "pow((1 + x), (-1 + y))")

    def test_neg(self):
        numpy_compare.assert_equal(-(x + 1), "(-1 - x)")

    def test_equalto(self):
        self.assertTrue(x.EqualTo(x))
        self.assertFalse(x.EqualTo(y))

    def test_is_polynomial(self):
        self.assertTrue((x*y).is_polynomial())
        self.assertFalse((x/y).is_polynomial())

    def test_logical(self):
        numpy_compare.assert_equal(
            sym.logical_not(x == 0), "!((x == 0))")

        # Test single-operand logical statements
        numpy_compare.assert_equal(sym.logical_and(x >= 1), "(x >= 1)")
        numpy_compare.assert_equal(sym.logical_or(x >= 1), "(x >= 1)")
        # Test binary operand logical statements
        numpy_compare.assert_equal(
            sym.logical_and(x >= 1, x <= 2), "((x >= 1) and (x <= 2))")
        numpy_compare.assert_equal(
            sym.logical_or(x <= 1, x >= 2), "((x >= 2) or (x <= 1))")
        # Test multiple operand logical statements
        numpy_compare.assert_equal(
            sym.logical_and(x >= 1, x <= 2, y == 2),
            "((y == 2) and (x >= 1) and (x <= 2))")
        numpy_compare.assert_equal(
            sym.logical_or(x >= 1, x <= 2, y == 2),
            "((y == 2) or (x >= 1) or (x <= 2))")

    def test_functions_with_variable(self):
        numpy_compare.assert_equal(sym.abs(x), "abs(x)")
        numpy_compare.assert_equal(sym.exp(x), "exp(x)")
        numpy_compare.assert_equal(sym.sqrt(x), "sqrt(x)")
        numpy_compare.assert_equal(sym.pow(x, y), "pow(x, y)")
        numpy_compare.assert_equal(sym.sin(x), "sin(x)")
        numpy_compare.assert_equal(sym.cos(x), "cos(x)")
        numpy_compare.assert_equal(sym.tan(x), "tan(x)")
        numpy_compare.assert_equal(sym.asin(x), "asin(x)")
        numpy_compare.assert_equal(sym.acos(x), "acos(x)")
        numpy_compare.assert_equal(sym.atan(x), "atan(x)")
        numpy_compare.assert_equal(sym.atan2(x, y), "atan2(x, y)")
        numpy_compare.assert_equal(sym.sinh(x), "sinh(x)")
        numpy_compare.assert_equal(sym.cosh(x), "cosh(x)")
        numpy_compare.assert_equal(sym.tanh(x), "tanh(x)")
        numpy_compare.assert_equal(sym.min(x, y), "min(x, y)")
        numpy_compare.assert_equal(sym.max(x, y), "max(x, y)")
        numpy_compare.assert_equal(sym.ceil(x), "ceil(x)")
        numpy_compare.assert_equal(sym.floor(x), "floor(x)")
        numpy_compare.assert_equal(
            sym.if_then_else(x > y, x, y), "(if (x > y) then x else y)")

    def test_array_str(self):
        # Addresses #8729.
        value = str(np.array([x, y]))
        self.assertIn("Variable('x', Continuous)", value)
        self.assertIn("Variable('y', Continuous)", value)


class TestSymbolicVariables(unittest.TestCase):
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


class TestSymbolicExpression(unittest.TestCase):
    def setUp(self):
        unittest.TestCase.setUp(self)
        # For some reason, something in how `unittest` tries to scope warnings
        # causes the previous filters to be lost. Re-install here.
        # TODO(eric.cousineau): This used to be necessary for PY3-only, but
        # with NumPy 1.16, it became PY2 too. Figure out why.
        install_numpy_warning_filters(force=True)

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
        numpy_compare.assert_equal(e_xv + e_yv, "(x + y)")
        numpy_compare.assert_equal(e_xv + yv, "(x + y)")
        numpy_compare.assert_equal(e_xv + 1, "(1 + x)")
        numpy_compare.assert_equal(xv + e_yv, "(x + y)")
        numpy_compare.assert_equal(1 + e_xv, "(1 + x)")

        # - In place.
        e = copy.copy(xv)
        e += e_yv
        numpy_compare.assert_equal(e, "(x + y)")
        e += zv
        numpy_compare.assert_equal(e, "(x + y + z)")
        e += 1
        numpy_compare.assert_equal(e, "(1 + x + y + z)")

        # Subtraction.
        numpy_compare.assert_equal(e_xv - e_yv, "(x - y)")
        numpy_compare.assert_equal(e_xv - yv, "(x - y)")
        numpy_compare.assert_equal(e_xv - 1, "(-1 + x)")
        numpy_compare.assert_equal(xv - e_yv, "(x - y)")
        numpy_compare.assert_equal(1 - e_xv, "(1 - x)")

        # - In place.
        e = copy.copy(xv)
        e -= e_yv
        numpy_compare.assert_equal(e, (x - y))
        e -= zv
        numpy_compare.assert_equal(e, (x - y - z))
        e -= 1
        numpy_compare.assert_equal(e, (x - y - z - 1))

        # Multiplication.
        numpy_compare.assert_equal(e_xv * e_yv, "(x * y)")
        numpy_compare.assert_equal(e_xv * yv, "(x * y)")
        numpy_compare.assert_equal(e_xv * 1, "x")
        numpy_compare.assert_equal(xv * e_yv, "(x * y)")
        numpy_compare.assert_equal(1 * e_xv, "x")

        # - In place.
        e = copy.copy(xv)
        e *= e_yv
        numpy_compare.assert_equal(e, "(x * y)")
        e *= zv
        numpy_compare.assert_equal(e, "(x * y * z)")
        e *= 1
        numpy_compare.assert_equal(e, "(x * y * z)")

        # Division
        numpy_compare.assert_equal(e_xv / e_yv, (x / y))
        numpy_compare.assert_equal(e_xv / yv, (x / y))
        numpy_compare.assert_equal(e_xv / 1, "x")
        numpy_compare.assert_equal(xv / e_yv, (x / y))
        numpy_compare.assert_equal(1 / e_xv, (1 / x))

        # - In place.
        e = copy.copy(xv)
        e /= e_yv
        numpy_compare.assert_equal(e, (x / y))
        e /= zv
        numpy_compare.assert_equal(e, (x / y / z))
        e /= 1
        numpy_compare.assert_equal(e, ((x / y) / z))

        # Unary
        numpy_compare.assert_equal(+e_xv, "x")
        numpy_compare.assert_equal(-e_xv, "(-1 * x)")

        # Comparison. For `VectorizedAlgebra`, uses `np.vectorize` workaround
        # for #8315.
        # TODO(eric.cousineau): `BaseAlgebra.check_logical` is designed for
        # AutoDiffXd (float-convertible), not for symbolic (not always
        # float-convertible).
        numpy_compare.assert_equal(algebra.lt(e_xv, e_yv), "(x < y)")
        numpy_compare.assert_equal(algebra.le(e_xv, e_yv), "(x <= y)")
        numpy_compare.assert_equal(algebra.eq(e_xv, e_yv), "(x == y)")
        numpy_compare.assert_equal(algebra.ne(e_xv, e_yv), "(x != y)")
        numpy_compare.assert_equal(algebra.ge(e_xv, e_yv), "(x >= y)")
        numpy_compare.assert_equal(algebra.gt(e_xv, e_yv), "(x > y)")

        # Math functions.
        numpy_compare.assert_equal(algebra.abs(e_xv), "abs(x)")
        numpy_compare.assert_equal(algebra.exp(e_xv), "exp(x)")
        numpy_compare.assert_equal(algebra.sqrt(e_xv), "sqrt(x)")
        numpy_compare.assert_equal(algebra.pow(e_xv, e_yv), "pow(x, y)")
        numpy_compare.assert_equal(algebra.sin(e_xv), "sin(x)")
        numpy_compare.assert_equal(algebra.cos(e_xv), "cos(x)")
        numpy_compare.assert_equal(algebra.tan(e_xv), "tan(x)")
        numpy_compare.assert_equal(algebra.arcsin(e_xv), "asin(x)")
        numpy_compare.assert_equal(algebra.arccos(e_xv), "acos(x)")
        numpy_compare.assert_equal(algebra.arctan2(e_xv, e_yv), "atan2(x, y)")
        numpy_compare.assert_equal(algebra.sinh(e_xv), "sinh(x)")
        numpy_compare.assert_equal(algebra.cosh(e_xv), "cosh(x)")
        numpy_compare.assert_equal(algebra.tanh(e_xv), "tanh(x)")
        numpy_compare.assert_equal(algebra.ceil(e_xv), "ceil(x)")
        numpy_compare.assert_equal(algebra.floor(e_xv), "floor(x)")

        if isinstance(algebra, ScalarAlgebra):
            # TODO(eric.cousineau): Uncomment these lines if we can teach numpy
            # that reduction is not just selection.
            numpy_compare.assert_equal(algebra.min(e_xv, e_yv), "min(x, y)")
            numpy_compare.assert_equal(algebra.max(e_xv, e_yv), "max(x, y)")
            # TODO(eric.cousineau): Add broadcasting functions for these
            # operations.
            numpy_compare.assert_equal(sym.atan(e_xv), "atan(x)")
            numpy_compare.assert_equal(
                sym.if_then_else(e_xv > e_yv, e_xv, e_yv),
                "(if (x > y) then x else y)")

        return xv, e_xv

    def test_scalar_algebra(self):
        xv, e_xv = self._check_algebra(ScalarAlgebra())
        self.assertIsInstance(xv, sym.Variable)
        self.assertIsInstance(e_xv, sym.Expression)

    def test_array_algebra(self):
        xv, e_xv = self._check_algebra(VectorizedAlgebra())
        self.assertEqual(xv.shape, (2,))
        self.assertIsInstance(xv[0], sym.Variable)
        self.assertEqual(e_xv.shape, (2,))
        self.assertIsInstance(e_xv[0], sym.Expression)

    def test_equalto(self):
        self.assertTrue((x + y).EqualTo(x + y))
        self.assertFalse((x + y).EqualTo(x - y))

    def test_get_variables(self):
        vars = e_x.GetVariables()
        self.assertEqual(len(vars), 1)
        self.assertTrue(list(vars)[0].EqualTo(x))

    def test_relational_operators(self):
        # TODO(eric.cousineau): Use `VectorizedAlgebra` overloads once #8315 is
        # resolved.
        # Expression rop Expression
        numpy_compare.assert_equal(e_x < e_y, "(x < y)")
        numpy_compare.assert_equal(e_x <= e_y, "(x <= y)")
        numpy_compare.assert_equal(e_x > e_y, "(x > y)")
        numpy_compare.assert_equal(e_x >= e_y, "(x >= y)")
        numpy_compare.assert_equal(e_x == e_y, "(x == y)")
        numpy_compare.assert_equal(e_x != e_y, "(x != y)")

        # Expression rop Variable
        numpy_compare.assert_equal(e_x < y, "(x < y)")
        numpy_compare.assert_equal(e_x <= y, "(x <= y)")
        numpy_compare.assert_equal(e_x > y, "(x > y)")
        numpy_compare.assert_equal(e_x >= y, "(x >= y)")
        numpy_compare.assert_equal(e_x == y, "(x == y)")
        numpy_compare.assert_equal(e_x != y, "(x != y)")

        # Variable rop Expression
        numpy_compare.assert_equal(x < e_y, "(x < y)")
        numpy_compare.assert_equal(x <= e_y, "(x <= y)")
        numpy_compare.assert_equal(x > e_y, "(x > y)")
        numpy_compare.assert_equal(x >= e_y, "(x >= y)")
        numpy_compare.assert_equal(x == e_y, "(x == y)")
        numpy_compare.assert_equal(x != e_y, "(x != y)")

        # Expression rop float
        numpy_compare.assert_equal(e_x < 1, "(x < 1)")
        numpy_compare.assert_equal(e_x <= 1, "(x <= 1)")
        numpy_compare.assert_equal(e_x > 1, "(x > 1)")
        numpy_compare.assert_equal(e_x >= 1, "(x >= 1)")
        numpy_compare.assert_equal(e_x == 1, "(x == 1)")
        numpy_compare.assert_equal(e_x != 1, "(x != 1)")

        # float rop Expression
        numpy_compare.assert_equal(1 < e_y, "(y > 1)")
        numpy_compare.assert_equal(1 <= e_y, "(y >= 1)")
        numpy_compare.assert_equal(1 > e_y, "(y < 1)")
        numpy_compare.assert_equal(1 >= e_y, "(y <= 1)")
        numpy_compare.assert_equal(1 == e_y, "(y == 1)")
        numpy_compare.assert_equal(1 != e_y, "(y != 1)")

    def test_relational_operators_nonzero(self):
        # For issues #8135 and #8491. See `pydrake.math` for operator overloads
        # that work around this, which are tested in `_check_algebra`.
        # Ensure that we throw on `__nonzero__`.
        with self.assertRaises(RuntimeError) as cm:
            value = bool(e_x == e_x)
        message = str(cm.exception)
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
        numpy_compare.assert_equal(sym.abs(v_x), np.abs(v_x))
        numpy_compare.assert_not_equal(sym.abs(v_x), 0.5*np.abs(v_x))
        numpy_compare.assert_equal(sym.abs(v_x), np.abs(v_x))
        numpy_compare.assert_equal(sym.abs(v_x), np.abs(v_x))
        numpy_compare.assert_equal(sym.exp(v_x), np.exp(v_x))
        numpy_compare.assert_equal(sym.sqrt(v_x), np.sqrt(v_x))
        numpy_compare.assert_equal(sym.pow(v_x, v_y), v_x ** v_y)
        numpy_compare.assert_equal(sym.sin(v_x), np.sin(v_x))
        numpy_compare.assert_equal(sym.cos(v_x), np.cos(v_x))
        numpy_compare.assert_equal(sym.tan(v_x), np.tan(v_x))
        numpy_compare.assert_equal(sym.asin(v_x), np.arcsin(v_x))
        numpy_compare.assert_equal(sym.acos(v_x), np.arccos(v_x))
        numpy_compare.assert_equal(sym.atan(v_x), np.arctan(v_x))
        numpy_compare.assert_equal(sym.atan2(v_x, v_y), np.arctan2(v_x, v_y))
        numpy_compare.assert_equal(sym.sinh(v_x), np.sinh(v_x))
        numpy_compare.assert_equal(sym.cosh(v_x), np.cosh(v_x))
        numpy_compare.assert_equal(sym.tanh(v_x), np.tanh(v_x))
        numpy_compare.assert_equal(sym.min(v_x, v_y), min(v_x, v_y))
        numpy_compare.assert_equal(sym.max(v_x, v_y), max(v_x, v_y))
        numpy_compare.assert_equal(sym.ceil(v_x), np.ceil(v_x))
        numpy_compare.assert_equal(sym.floor(v_x), np.floor(v_x))
        numpy_compare.assert_equal(
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
        numpy_compare.assert_equal(J[0, 0], sym.cos(y))
        numpy_compare.assert_equal(J[1, 0], sym.sin(y))
        numpy_compare.assert_equal(J[2, 0], 2 * x)
        numpy_compare.assert_equal(J[0, 1], - x * sym.sin(y))
        numpy_compare.assert_equal(J[1, 1], x * sym.cos(y))
        numpy_compare.assert_equal(J[2, 1], sym.Expression(0))

    def test_method_jacobian(self):
        # (x * cos(y)).Jacobian([x, y]) returns [cos(y), -x * sin(y)].
        J = (x * sym.cos(y)).Jacobian([x, y])
        numpy_compare.assert_equal(J[0], sym.cos(y))
        numpy_compare.assert_equal(J[1], -x * sym.sin(y))

    def test_is_affine(self):
        M = np.array([[a * a * x, 3 * x], [2 * x, 3 * a]])
        self.assertTrue(sym.IsAffine(M, sym.Variables([x])))
        self.assertFalse(sym.IsAffine(M))

    def test_differentiate(self):
        e = x * x
        numpy_compare.assert_equal(e.Differentiate(x), 2 * x)

    def test_repr(self):
        self.assertEqual(repr(e_x), '<Expression "x">')

    def test_evaluate(self):
        env = {x: 3.0,
               y: 4.0}
        self.assertEqual((x + y).Evaluate(env),
                         env[x] + env[y])

    def test_evaluate_with_random_generator(self):
        g = pydrake.common.RandomGenerator()
        uni = sym.Variable("uni", sym.Variable.Type.RANDOM_UNIFORM)
        gau = sym.Variable("gau", sym.Variable.Type.RANDOM_GAUSSIAN)
        exp = sym.Variable("exp", sym.Variable.Type.RANDOM_EXPONENTIAL)
        # Checks if we can evaluate an expression with a random number
        # generator.
        (uni + gau + exp).Evaluate(g)
        (uni + gau + exp).Evaluate(generator=g)

        env = {x: 3.0,
               y: 4.0}
        # Checks if we can evaluate an expression with an environment and a
        # random number generator.
        (x + y + uni + gau + exp).Evaluate(env, g)
        (x + y + uni + gau + exp).Evaluate(env=env, generator=g)

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
        numpy_compare.assert_equal(e.Substitute(x, x + 5), x + y + 5)
        numpy_compare.assert_equal(e.Substitute(y, z), x + z)
        numpy_compare.assert_equal(e.Substitute(y, 3), x + 3)

    def test_substitute_with_dict(self):
        e = x + y
        env = {x: x + 2, y:  y + 3}
        numpy_compare.assert_equal(e.Substitute(env), x + y + 5)

    def test_copy(self):
        numpy_compare.assert_equal(copy.copy(e_x), e_x)
        numpy_compare.assert_equal(copy.deepcopy(e_x), e_x)

    # See `math_overloads_test` for more comprehensive checks on math
    # functions.


class TestSymbolicFormula(unittest.TestCase):
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
        tt = sym.Formula.True_()
        ff = sym.Formula.False_()
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


class TestSymbolicMonomial(unittest.TestCase):
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

    def test_constructor_vars_exponents(self):
        m = sym.Monomial([x, y], [1, 2])
        powers_out = EqualToDict(m.get_powers())
        self.assertEqual(powers_out[x], 1)
        self.assertEqual(powers_out[y], 2)

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
        numpy_compare.assert_equal(m1, "x^2")
        m2 = m1 * sym.Monomial(y)
        numpy_compare.assert_equal(m2, "x^2 * y")

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
        numpy_compare.assert_equal(e, "(pow(x, 3) * y)")

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

    def test_even_degree_monomial_basis(self):
        vars = sym.Variables([x, y])
        basis = sym.EvenDegreeMonomialBasis(vars, 2)
        self.assertEqual(basis.size, 4)

    def test_off_degree_monomial_basis(self):
        vars = sym.Variables([x, y])
        basis = sym.OddDegreeMonomialBasis(vars, 3)
        self.assertEqual(basis.size, 6)

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


class TestSymbolicPolynomial(unittest.TestCase):
    def test_default_constructor(self):
        p = sym.Polynomial()
        numpy_compare.assert_equal(p.ToExpression(), sym.Expression())

    def test_constructor_maptype(self):
        m = {sym.Monomial(x): sym.Expression(3),
             sym.Monomial(y): sym.Expression(2)}  # 3x + 2y
        p = sym.Polynomial(m)
        expected = 3 * x + 2 * y
        numpy_compare.assert_equal(p.ToExpression(), expected)

    def test_constructor_expression(self):
        e = 2 * x + 3 * y
        p = sym.Polynomial(e)
        numpy_compare.assert_equal(p.ToExpression(), e)

    def test_constructor_expression_indeterminates(self):
        e = a * x + b * y + c * z
        p = sym.Polynomial(e, sym.Variables([x, y, z]))
        decision_vars = sym.Variables([a, b, c])
        indeterminates = sym.Variables([x, y, z])
        self.assertEqual(p.indeterminates(), indeterminates)
        self.assertEqual(p.decision_variables(), decision_vars)

    def test_set_indeterminates(self):
        e = a * x * x + b * y + c * z
        indeterminates1 = sym.Variables([x, y, z])
        p = sym.Polynomial(e, indeterminates1)
        self.assertEqual(p.TotalDegree(), 2)

        indeterminates2 = sym.Variables([a, b, c])
        p.SetIndeterminates(indeterminates2)
        self.assertEqual(p.TotalDegree(), 1)

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
        numpy_compare.assert_equal(the_map[m], a)

    def test_differentiate(self):
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])  # p = ax²
        result = p.Differentiate(x)  # = 2ax
        numpy_compare.assert_equal(result.ToExpression(), 2 * a * x)

    def test_add_product(self):
        p = sym.Polynomial()
        m = sym.Monomial(x)
        p.AddProduct(sym.Expression(3), m)  # p += 3 * x
        numpy_compare.assert_equal(p.ToExpression(), 3 * x)

    def test_remove_terms_with_small_coefficients(self):
        e = 3 * x + 1e-12 * y
        p = sym.Polynomial(e, [x, y])
        q = p.RemoveTermsWithSmallCoefficients(1e-6)
        numpy_compare.assert_equal(q.ToExpression(), 3 * x)

    def test_comparison(self):
        p = sym.Polynomial()
        numpy_compare.assert_equal(p, p)
        self.assertIsInstance(p == p, sym.Formula)
        self.assertEqual(p == p, sym.Formula.True_())
        self.assertTrue(p.EqualTo(p))
        q = sym.Polynomial(sym.Expression(10))
        numpy_compare.assert_not_equal(p, q)
        self.assertIsInstance(p != q, sym.Formula)
        self.assertEqual(p != q, sym.Formula.True_())
        self.assertFalse(p.EqualTo(q))
        self.assertTrue(
            p.CoefficientsAlmostEqual(p + sym.Polynomial(1e-7), 1e-6))
        self.assertTrue(
            p.CoefficientsAlmostEqual(p + sym.Polynomial(1e-7 * x), 1e-6))
        self.assertFalse(
            p.CoefficientsAlmostEqual(p + sym.Polynomial(2e-6 * x), 1e-6))

    def test_repr(self):
        p = sym.Polynomial()
        self.assertEqual(repr(p), '<Polynomial "0">')

    def test_addition(self):
        p = sym.Polynomial(0.0, [x])
        numpy_compare.assert_equal(p + p, p)
        m = sym.Monomial(x)
        numpy_compare.assert_equal(m + p, sym.Polynomial(1 * x))
        numpy_compare.assert_equal(p + m, sym.Polynomial(1 * x))
        numpy_compare.assert_equal(p + 0, p)
        numpy_compare.assert_equal(0 + p, p)
        numpy_compare.assert_equal(x + p, sym.Polynomial(x) + p)
        numpy_compare.assert_equal(p + x, p + sym.Polynomial(x))

    def test_subtraction(self):
        p = sym.Polynomial(0.0, [x])
        numpy_compare.assert_equal(p - p, p)
        m = sym.Monomial(x)
        numpy_compare.assert_equal(m - p, sym.Polynomial(1 * x))
        numpy_compare.assert_equal(p - m, sym.Polynomial(-1 * x))
        numpy_compare.assert_equal(p - 0, p)
        numpy_compare.assert_equal(0 - p, -p)
        numpy_compare.assert_equal(x - p, sym.Polynomial(x))
        numpy_compare.assert_equal(p - x, sym.Polynomial(-x))

    def test_multiplication(self):
        p = sym.Polynomial(0.0, [x])
        numpy_compare.assert_equal(p * p, p)
        m = sym.Monomial(x)
        numpy_compare.assert_equal(m * p, p)
        numpy_compare.assert_equal(p * m, p)
        numpy_compare.assert_equal(p * 0, p)
        numpy_compare.assert_equal(0 * p, p)
        numpy_compare.assert_equal(sym.Polynomial(x) * x,
                                   sym.Polynomial(x * x))
        numpy_compare.assert_equal(x * sym.Polynomial(x),
                                   sym.Polynomial(x * x))

    def test_division(self):
        p = sym.Polynomial(x * x + x)
        numpy_compare.assert_equal(p / 2,
                                   sym.Polynomial(1 / 2 * x * x + 1 / 2 * x))

    def test_addition_assignment(self):
        p = sym.Polynomial()
        p += p
        numpy_compare.assert_equal(p, sym.Polynomial())
        p += sym.Monomial(x)
        numpy_compare.assert_equal(p, sym.Polynomial(1 * x))
        p += 3
        numpy_compare.assert_equal(p, sym.Polynomial(3 + 1 * x))

    def test_subtraction_assignment(self):
        p = sym.Polynomial()
        p -= p
        numpy_compare.assert_equal(p, sym.Polynomial())
        p -= sym.Monomial(x)
        numpy_compare.assert_equal(p, sym.Polynomial(-1 * x))
        p -= 3
        numpy_compare.assert_equal(p, sym.Polynomial(-1 * x - 3))

    def test_multiplication_assignment(self):
        p = sym.Polynomial()
        p *= p
        numpy_compare.assert_equal(p, sym.Polynomial())
        p *= sym.Monomial(x)
        numpy_compare.assert_equal(p, sym.Polynomial())
        p *= 3
        numpy_compare.assert_equal(p, sym.Polynomial())

    def test_pow(self):
        e = a * (x ** 2)
        p = sym.Polynomial(e, [x])  # p = ax²
        p = pow(p, 2)  # p = a²x⁴
        numpy_compare.assert_equal(p.ToExpression(), (a ** 2) * (x ** 4))

    def test_jacobian_vector(self):
        e = 5 * x ** 2 + 4 * y ** 2 + 8 * x * y
        p = sym.Polynomial(e, [x, y])                  # p = 5x² + 4y² + 8xy
        p_dx = sym.Polynomial(10 * x + 8 * y, [x, y])  # ∂p/∂x = 10x + 8y
        p_dy = sym.Polynomial(8 * y + 8 * x, [x, y])   # ∂p/∂y =  8y + 8x

        J = p.Jacobian([x, y])
        numpy_compare.assert_equal(J[0], p_dx)
        numpy_compare.assert_equal(J[1], p_dy)

    def test_jacobian_matrix(self):
        p1 = sym.Polynomial(x * x + y, [x, y])      # p1 = x² + y
        p2 = sym.Polynomial(2 * x + y * y, [x, y])  # p2 = 2x + y²
        p1_dx = sym.Polynomial(2 * x, [x, y])       # ∂p1/∂x = 2x
        p1_dy = sym.Polynomial(1.0, [x, y])         # ∂p1/∂y =  1
        p2_dx = sym.Polynomial(2, [x, y])           # ∂p1/∂x = 2
        p2_dy = sym.Polynomial(2 * y, [x, y])       # ∂p1/∂y =  2y

        J = sym.Jacobian([p1, p2], [x, y])
        numpy_compare.assert_equal(J[0, 0], p1_dx)
        numpy_compare.assert_equal(J[0, 1], p1_dy)
        numpy_compare.assert_equal(J[1, 0], p2_dx)
        numpy_compare.assert_equal(J[1, 1], p2_dy)

    def test_evaluate_polynomial_matrix(self):
        p1 = sym.Polynomial(x * x + y, [x, y])      # p1 = x² + y
        p2 = sym.Polynomial(2 * x + y * y, [x, y])  # p2 = 2x + y²
        env = {x: 1.0, y: 2.0}

        result = sym.Evaluate([p1, p2], env)
        numpy_compare.assert_equal(result, [[3.0], [6.0]])

    def test_matrix_substitute_with_substitution(self):
        m = np.array([[x + y, x * y]])
        env = {x: x + 2, y:  y + 3}
        substituted = sym.Substitute(m, env)
        numpy_compare.assert_equal(substituted[0, 0], m[0, 0].Substitute(env))
        numpy_compare.assert_equal(substituted[0, 1], m[0, 1].Substitute(env))

    def test_matrix_substitute_with_variable_and_expression(self):
        m = np.array([[x + y, x * y]])
        substituted = sym.Substitute(m, x, 3.0)
        numpy_compare.assert_equal(
            substituted[0, 0], m[0, 0].Substitute(x, 3.0))
        numpy_compare.assert_equal(
            substituted[0, 1], m[0, 1].Substitute(x, 3.0))

    def test_matrix_evaluate_without_env(self):
        m = np.array([[3, 4]])
        evaluated1 = sym.Evaluate(m)
        evaluated2 = sym.Evaluate(m=m)
        self.assertTrue(np.array_equal(evaluated1, m))
        self.assertTrue(np.array_equal(evaluated2, m))

    def test_matrix_evaluate_with_env(self):
        m = np.array([[x + y, x * y]])
        env = {x: 3.0,
               y: 4.0}
        expected = np.array([[m[0, 0].Evaluate(env),
                              m[0, 1].Evaluate(env)]])
        evaluated1 = sym.Evaluate(m, env)
        evaluated2 = sym.Evaluate(m=m, env=env)
        self.assertTrue(np.array_equal(evaluated1, expected))
        self.assertTrue(np.array_equal(evaluated2, expected))

    def test_matrix_evaluate_with_random_generator(self):
        u = sym.Variable("uni", sym.Variable.Type.RANDOM_UNIFORM)
        m = np.array([[x + u, x - u]])
        env = {x: 3.0}
        g = pydrake.common.RandomGenerator()
        evaluated1 = sym.Evaluate(m, env, g)
        evaluated2 = sym.Evaluate(m=m, env=env, generator=g)
        self.assertEqual(evaluated1[0, 0] + evaluated1[0, 1], 2 * env[x])
        self.assertEqual(evaluated2[0, 0] + evaluated2[0, 1], 2 * env[x])

    def test_hash(self):
        p1 = sym.Polynomial(x * x, [x])
        p2 = sym.Polynomial(x * x, [x])
        numpy_compare.assert_equal(p1, p2)
        self.assertEqual(hash(p1), hash(p2))
        p1 += 1
        numpy_compare.assert_not_equal(p1, p2)
        self.assertNotEqual(hash(p1), hash(p2))

    def test_polynomial_evaluate(self):
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

    def test_polynomial_evaluate_partial(self):
        p = sym.Polynomial(a * x * x + b * x + c, [x])
        env = {a: 2.0,
               b: 3.0,
               c: 5.0}
        numpy_compare.assert_equal(
            p.EvaluatePartial(env),
            sym.Polynomial(env[a] * x * x + env[b] * x + env[c], [x]))
        numpy_compare.assert_equal(
            p.EvaluatePartial(a, 2),
            sym.Polynomial(2 * x * x + b * x + c, [x]))
