# -*- coding: utf-8 -*-

import copy
import itertools
import unittest

import numpy as np

import pydrake.symbolic as sym
import pydrake.common
import pydrake.math as drake_math
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
boolean = sym.Variable(name="boolean", type=sym.Variable.Type.BOOLEAN)


class TestSymbolicVariable(unittest.TestCase):
    def test_is_dummy(self):
        self.assertFalse(a.is_dummy())

    def test_get_name(self):
        self.assertEqual(a.get_name(), "a")
        self.assertEqual(b.get_name(), "b")
        self.assertEqual(c.get_name(), "c")

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
        numpy_compare.assert_equal(np.exp(x), "exp(x)")
        numpy_compare.assert_equal(sym.sqrt(x), "sqrt(x)")
        numpy_compare.assert_equal(np.sqrt(x), "sqrt(x)")
        numpy_compare.assert_equal(sym.pow(x, y), "pow(x, y)")
        numpy_compare.assert_equal(sym.sin(x), "sin(x)")
        numpy_compare.assert_equal(np.sin(x), "sin(x)")
        numpy_compare.assert_equal(sym.cos(x), "cos(x)")
        numpy_compare.assert_equal(np.cos(x), "cos(x)")
        numpy_compare.assert_equal(sym.tan(x), "tan(x)")
        numpy_compare.assert_equal(np.tan(x), "tan(x)")
        numpy_compare.assert_equal(sym.asin(x), "asin(x)")
        numpy_compare.assert_equal(np.arcsin(x), "asin(x)")
        numpy_compare.assert_equal(sym.acos(x), "acos(x)")
        numpy_compare.assert_equal(np.arccos(x), "acos(x)")
        numpy_compare.assert_equal(sym.atan(x), "atan(x)")
        numpy_compare.assert_equal(np.arctan(x), "atan(x)")
        numpy_compare.assert_equal(sym.atan2(x, y), "atan2(x, y)")
        numpy_compare.assert_equal(sym.sinh(x), "sinh(x)")
        numpy_compare.assert_equal(np.sinh(x), "sinh(x)")
        numpy_compare.assert_equal(sym.cosh(x), "cosh(x)")
        numpy_compare.assert_equal(np.cosh(x), "cosh(x)")
        numpy_compare.assert_equal(sym.tanh(x), "tanh(x)")
        numpy_compare.assert_equal(np.tanh(x), "tanh(x)")
        numpy_compare.assert_equal(sym.min(x, y), "min(x, y)")
        numpy_compare.assert_equal(sym.max(x, y), "max(x, y)")
        numpy_compare.assert_equal(sym.ceil(x), "ceil(x)")
        numpy_compare.assert_equal(sym.floor(x), "floor(x)")
        numpy_compare.assert_equal(
            sym.if_then_else(x > y, x, y), "(if (x > y) then x else y)")
        numpy_compare.assert_equal(
            sym.if_then_else(f_cond=x > y, e_then=x, e_else=y),
            "(if (x > y) then x else y)")
        numpy_compare.assert_equal(
            sym.uninterpreted_function(name="func_name", arguments=[e_x, e_y]),
            "func_name(x, y)")

    def test_array_str(self):
        # Addresses #8729.
        value = str(np.array([x, y]))
        self.assertIn("Variable('x', Continuous)", value)
        self.assertIn("Variable('y', Continuous)", value)


class TestMakeMatrixVariable(unittest.TestCase):
    # Test both MakeMatrixVariable and MakeVectorVariable (and the variations)
    def test_make_matrix_variable(self):
        # Call MakeMatrixVariable with default variable type.
        A = sym.MakeMatrixVariable(3, 2, "A")
        self.assertEqual(A.shape, (3, 2))
        for i in range(3):
            for j in range(2):
                self.assertEqual(
                    repr(A[i, j]), f"Variable('A({i}, {j})', Continuous)")

        # Call MakeMatrixVariable with specified variable type.
        A = sym.MakeMatrixVariable(3, 2, "A", sym.Variable.Type.BINARY)
        self.assertEqual(A.shape, (3, 2))
        for i in range(3):
            for j in range(2):
                self.assertEqual(
                    repr(A[i, j]), f"Variable('A({i}, {j})', Binary)")

    def test_make_matrix_continuous_variable(self):
        A = sym.MakeMatrixContinuousVariable(3, 2, "A")
        self.assertEqual(A.shape, (3, 2))
        for i in range(3):
            for j in range(2):
                self.assertEqual(
                    repr(A[i, j]), f"Variable('A({i}, {j})', Continuous)")

    def test_make_matrix_binary_variable(self):
        A = sym.MakeMatrixBinaryVariable(3, 2, "A")
        self.assertEqual(A.shape, (3, 2))
        for i in range(3):
            for j in range(2):
                self.assertEqual(
                    repr(A[i, j]), f"Variable('A({i}, {j})', Binary)")

    def test_make_matrix_boolean_variable(self):
        A = sym.MakeMatrixBooleanVariable(3, 2, "A")
        self.assertEqual(A.shape, (3, 2))
        for i in range(3):
            for j in range(2):
                self.assertEqual(
                    repr(A[i, j]), f"Variable('A({i}, {j})', Boolean)")

    def test_make_vector_variable(self):
        # Call MakeVectorVariable with default variable type.
        a = sym.MakeVectorVariable(3, "a")
        self.assertEqual(a.shape, (3,))
        for i in range(3):
            self.assertEqual(repr(a[i]), f"Variable('a({i})', Continuous)")

        # Call MakeVectorVariable with specified type.
        a = sym.MakeVectorVariable(3, "a", sym.Variable.Type.BINARY)
        self.assertEqual(a.shape, (3,))
        for i in range(3):
            self.assertEqual(repr(a[i]), f"Variable('a({i})', Binary)")

    def test_make_vector_continuous_variable(self):
        a = sym.MakeVectorContinuousVariable(3, "a")
        self.assertEqual(a.shape, (3,))
        for i in range(3):
            self.assertEqual(repr(a[i]), f"Variable('a({i})', Continuous)")

    def test_make_vector_binary_variable(self):
        a = sym.MakeVectorBinaryVariable(3, "a")
        self.assertEqual(a.shape, (3,))
        for i in range(3):
            self.assertEqual(repr(a[i]), f"Variable('a({i})', Binary)")

    def test_make_vector_boolean_variable(self):
        a = sym.MakeVectorBooleanVariable(3, "a")
        self.assertEqual(a.shape, (3,))
        for i in range(3):
            self.assertEqual(repr(a[i]), f"Variable('a({i})', Boolean)")


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
        vars.insert(var=y)
        self.assertEqual(vars.size(), 2)

    def test_insert2(self):
        vars = sym.Variables([x])
        vars.insert(sym.Variables([y, z]))
        self.assertEqual(vars.size(), 3)
        vars.insert(vars=sym.Variables([a, b, c]))
        self.assertEqual(vars.size(), 6)

    def test_erase1(self):
        vars = sym.Variables([x, y, z])
        count = vars.erase(x)
        self.assertEqual(count, 1)
        self.assertEqual(vars.size(), 2)
        count = vars.erase(key=y)
        self.assertEqual(count, 1)
        self.assertEqual(vars.size(), 1)

    def test_erase2(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([w, z])
        count = vars1.erase(vars2)
        self.assertEqual(count, 1)
        self.assertEqual(vars1.size(), 2)

    def test_erase2_kwarg(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([x, y])
        count = vars1.erase(vars=vars2)
        self.assertEqual(count, 2)
        self.assertEqual(vars1.size(), 1)

    def test_include(self):
        vars = sym.Variables([x, y, z])
        self.assertTrue(vars.include(y))
        self.assertTrue(vars.include(key=x))
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
        self.assertFalse(vars1.IsSubsetOf(vars=vars2))
        self.assertFalse(vars1.IsStrictSubsetOf(vars2))
        self.assertFalse(vars1.IsStrictSubsetOf(vars=vars2))
        self.assertTrue(vars1.IsSupersetOf(vars2))
        self.assertTrue(vars1.IsSupersetOf(vars=vars2))
        self.assertTrue(vars1.IsStrictSupersetOf(vars2))
        self.assertTrue(vars1.IsStrictSupersetOf(vars=vars2))

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
        vars4 = sym.intersect(vars1=vars1, vars2=vars2)
        self.assertEqual(vars4, sym.Variables([y]))

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

    def test_constructor(self):
        sym.Expression(z)
        sym.Expression(var=z)
        sym.Expression(2.2)
        sym.Expression(constant=2.2)

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

    def test_vectorized_binary_operator_type_combinatorics(self):
        """
        Tests vectorized binary operator via brute-force combinatorics per
        #15549.

        This complements test with the same name in ``autodiffutils_test.py``.
        """

        def expand_values(value):
            return (
                # Scalar.
                value,
                # Scalar array.
                np.array(value),
                # Size-1 array.
                np.array([value]),
                # Size-2 array.
                np.array([value, value]),
            )

        operators = drake_math._OPERATORS
        operators_reverse = drake_math._OPERATORS_REVERSE

        T_operands_x = (
            # Variable.
            expand_values(x)
            # Expression.
            + expand_values(e_x)
        )
        T_operands_y = (
            # Variable.
            expand_values(y)
            # Expression.
            + expand_values(e_y)
        )
        numeric_operands = (
            # Float.
            # - Native.
            expand_values(1.0)
            # - np.generic
            + expand_values(np.float64(1.0))
            # Int.
            # - Native.
            + expand_values(1)
            # - np.generic
            + expand_values(np.int64(1.0))
        )

        @np.vectorize
        def assert_nontrivial_formula(value):
            self.assertIsInstance(value, sym.Formula)
            self.assertNotEqual(value, sym.Formula.True_())
            self.assertNotEqual(value, sym.Formula.False_())

        def check_operands(op, lhs_operands, rhs_operands):
            operand_combinatorics_iter = itertools.product(
                lhs_operands, rhs_operands
            )
            op_reverse = operators_reverse[op]
            for lhs, rhs in operand_combinatorics_iter:
                hint_for_error = f"{op.__doc__}: {repr(lhs)}, {repr(rhs)}"
                with numpy_compare.soft_sub_test(hint_for_error):
                    value = op(lhs, rhs)
                    assert_nontrivial_formula(value)
                    reverse_value = op_reverse(rhs, lhs)
                    assert_nontrivial_formula(reverse_value)
                    numpy_compare.assert_equal(value, reverse_value)

        # Combinations (unordered) that we're interested in.
        operand_combinations = (
            (T_operands_x, T_operands_y),
            (T_operands_x, numeric_operands),
        )
        for op in operators:
            for (op_a, op_b) in operand_combinations:
                check_operands(op, op_a, op_b)
                check_operands(op, op_b, op_a)

    def test_equalto(self):
        self.assertTrue((x + y).EqualTo(x + y))
        self.assertFalse((x + y).EqualTo(x - y))

    def test_get_kind(self):
        self.assertEqual((x + y).get_kind(), sym.ExpressionKind.Add)
        self.assertEqual((x * y).get_kind(), sym.ExpressionKind.Mul)

    def test_get_variables(self):
        vars = e_x.GetVariables()
        self.assertEqual(len(vars), 1)
        self.assertTrue(list(vars)[0].EqualTo(x))

    def test_get_variable_vector(self):
        vars_ = sym.GetVariableVector([e_x, e_y])
        self.assertEqual(len(vars_), 2)
        if vars_[0].get_id() == x.get_id():
            self.assertEqual(vars_[1].get_id(), y.get_id())
        else:
            self.assertEqual(vars_[0].get_id(), y.get_id())
            self.assertEqual(vars_[1].get_id(), x.get_id())

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
        def check_jacobian(J):
            numpy_compare.assert_equal(J[0, 0], sym.cos(y))
            numpy_compare.assert_equal(J[1, 0], sym.sin(y))
            numpy_compare.assert_equal(J[2, 0], 2 * x)
            numpy_compare.assert_equal(J[0, 1], - x * sym.sin(y))
            numpy_compare.assert_equal(J[1, 1], x * sym.cos(y))
            numpy_compare.assert_equal(J[2, 1], sym.Expression(0))

        f = [x * sym.cos(y), x * sym.sin(y), x ** 2]
        vars = [x, y]
        check_jacobian(sym.Jacobian(f, vars))
        check_jacobian(sym.Jacobian(f=f, vars=vars))

    def test_method_jacobian(self):
        # (x * cos(y)).Jacobian([x, y]) returns [cos(y), -x * sin(y)].
        def check_jacobian(J):
            numpy_compare.assert_equal(J[0], sym.cos(y))
            numpy_compare.assert_equal(J[1], -x * sym.sin(y))

        e = x * sym.cos(y)
        vars = [x, y]
        check_jacobian(e.Jacobian(vars))
        check_jacobian(e.Jacobian(vars=vars))

    def test_is_affine(self):
        M = np.array([[a * a * x, 3 * x], [2 * x, 3 * a]])
        vars = sym.Variables([x])
        self.assertTrue(sym.IsAffine(M, vars))
        self.assertTrue(sym.IsAffine(m=M, vars=vars))
        self.assertFalse(sym.IsAffine(M))
        self.assertFalse(sym.IsAffine(m=M))

    def test_differentiate(self):
        e = x * x
        numpy_compare.assert_equal(e.Differentiate(x), 2 * x)
        numpy_compare.assert_equal(e.Differentiate(x=x), 2 * x)

    def test_repr(self):
        self.assertEqual(repr(e_x), '<Expression "x">')

    def test_to_string(self):
        e = (x + y)
        self.assertEqual(e.to_string(), "(x + y)")
        self.assertEqual(str(e), "(x + y)")

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
        partial_evaluated = (x + y + z).EvaluatePartial(env=env)
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
        numpy_compare.assert_equal(e.Substitute(var=x, e=(x + 5)), x + y + 5)
        numpy_compare.assert_equal(e.Substitute(y, z), x + z)
        numpy_compare.assert_equal(e.Substitute(var=y, e=z), x + z)
        numpy_compare.assert_equal(e.Substitute(y, 3), x + 3)
        numpy_compare.assert_equal(e.Substitute(var=y, e=3), x + 3)

    def test_substitute_with_dict(self):
        e = x + y
        env = {x: x + 2, y:  y + 3}
        numpy_compare.assert_equal(e.Substitute(env), x + y + 5)
        numpy_compare.assert_equal(e.Substitute(s=env), x + y + 5)

    def test_copy(self):
        numpy_compare.assert_equal(copy.copy(e_x), e_x)
        numpy_compare.assert_equal(copy.deepcopy(e_x), e_x)

    def test_taylor_expand(self):
        e = sym.sin(x)
        env = {x: 0}
        numpy_compare.assert_equal(
            sym.TaylorExpand(f=e, a=env, order=1), sym.Expression(x))

    # See `math_overloads_test` for more comprehensive checks on math
    # functions.


class TestSymbolicFormula(unittest.TestCase):
    def test_constructor(self):
        sym.Formula(var=boolean)

    def test_factory_functions(self):
        f = sym.forall(vars=sym.Variables([x]), f=(x > 0))
        self.assertEqual(f.get_kind(), sym.FormulaKind.Forall)

        f = sym.isnan(e_x)
        self.assertEqual(f.get_kind(), sym.FormulaKind.Isnan)

        f = sym.positive_semidefinite([e_x])
        self.assertEqual(f.get_kind(), sym.FormulaKind.PositiveSemidefinite)

    def test_get_kind(self):
        self.assertEqual((x > y).get_kind(), sym.FormulaKind.Gt)

    def test_get_free_variables(self):
        f = x > y
        self.assertEqual(f.GetFreeVariables(), sym.Variables([x, y]))

    def test_substitute_with_pair(self):
        f = x > y
        self.assertEqual(f.Substitute(y, y + 5), x > y + 5)
        self.assertEqual(f.Substitute(var=y, e=y + 5), x > y + 5)
        self.assertEqual(f.Substitute(y, z), x > z)
        self.assertEqual(f.Substitute(var=y, e=z), x > z)
        self.assertEqual(f.Substitute(y, 3), x > 3)
        self.assertEqual(f.Substitute(var=y, e=3), x > 3)

    def test_substitute_with_dict(self):
        f = x + y > z
        s = {x: x + 2, y:  y + 3}
        self.assertEqual(f.Substitute(s), x + y + 5 > z)
        self.assertEqual(f.Substitute(s=s), x + y + 5 > z)

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
        self.assertEqual((x < y).Evaluate(env=env),
                         env[x] < env[y])
        self.assertTrue(sym.Formula.True_().Evaluate())

    def test_evaluate_exception_np_nan(self):
        env = {x: np.nan}
        with self.assertRaises(RuntimeError):
            (x > 1).Evaluate(env)

    def test_evaluate_exception_python_nan(self):
        env = {x: float('nan')}
        with self.assertRaises(RuntimeError):
            (x > 1).Evaluate(env)


class TestSymbolicMonomial(unittest.TestCase):
    def test_constructor_empty(self):
        m = sym.Monomial()  # m = 1
        self.assertEqual(m.GetVariables().size(), 0)
        self.assertEqual(m.total_degree(), 0)

    def test_constructor_variable(self):
        def check_monomial(m):  # m = x¹
            self.assertEqual(m.degree(x), 1)
            self.assertEqual(m.total_degree(), 1)

        check_monomial(sym.Monomial(x))
        check_monomial(sym.Monomial(var=x))

    def test_constructor_variable_int(self):
        def check_monomial(m):  # m = x²
            self.assertEqual(m.degree(x), 2)
            self.assertEqual(m.total_degree(), 2)

        check_monomial(sym.Monomial(x, 2))
        check_monomial(sym.Monomial(var=x, exponent=2))

    def test_constructor_map(self):
        def check_monomial(m):
            powers_out = EqualToDict(m.get_powers())
            self.assertEqual(powers_out[x], 2)
            self.assertEqual(powers_out[y], 3)
            self.assertEqual(powers_out[z], 4)

        powers_in = {x: 2, y: 3, z: 4}
        check_monomial(sym.Monomial(powers_in))
        check_monomial(sym.Monomial(powers=powers_in))

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
        self.assertEqual(m3.degree(v=y), 3)  # NOTE: tests kwarg v binding.

    def test_multiplication2(self):
        m1 = sym.Monomial(x, 2)
        m2 = m1 * sym.Monomial(y)
        m3 = sym.Monomial(y) * m1
        self.assertEqual(m2.degree(x), 2)
        self.assertEqual(m2.degree(y), 1)
        self.assertEqual(m2, m3)

    def test_multiplication_float(self):
        # Test monomial multiplies with a float. Should return a polynomial.
        m1 = sym.Monomial(x, 2)
        p1 = m1 * 3
        self.assertIsInstance(p1, sym.Polynomial)
        self.assertEqual(len(p1.monomial_to_coefficient_map()), 1)
        numpy_compare.assert_equal(
            p1.monomial_to_coefficient_map()[m1], sym.Expression(3))

        p2 = 3 * m1
        self.assertIsInstance(p2, sym.Polynomial)
        self.assertEqual(len(p2.monomial_to_coefficient_map()), 1)
        numpy_compare.assert_equal(
            p2.monomial_to_coefficient_map()[m1], sym.Expression(3))

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
        # Test repeated for testing kwarg p=2.
        m3 = m1.pow_in_place(p=2)
        self.assertEqual(m1.degree(x), 8)
        self.assertEqual(m1.degree(y), 4)
        self.assertEqual(m3.degree(x), 8)
        self.assertEqual(m3.degree(y), 4)

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
        basis2 = sym.MonomialBasis(vars=vars, degree=3)
        basis3 = sym.MonomialBasis([x, y, z], 3)
        basis4 = sym.MonomialBasis(vars=[x, y, z], degree=3)
        self.assertEqual(basis1.size, 20)
        self.assertEqual(basis2.size, 20)
        self.assertEqual(basis3.size, 20)
        self.assertEqual(basis4.size, 20)

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
        self.assertEqual(m.Evaluate(env=env),
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

    def test_evaluate_partial(self):
        m = sym.Monomial(x, 3) * sym.Monomial(y, 2)  # m = x³y²
        env = {x: 2.0,
               y: 3.0}
        d_72, m_1 = m.EvaluatePartial(env)
        self.assertEqual(d_72, 72.0)
        self.assertEqual(m_1, sym.Monomial())  # Monomial{} = 1

        d_8, y_2 = m.EvaluatePartial(env={x: 2.0})
        self.assertEqual(d_8, 8.0)
        self.assertEqual(y_2, sym.Monomial(y, 2))


class TestSymbolicPolynomial(unittest.TestCase):
    def test_default_constructor(self):
        p = sym.Polynomial()
        numpy_compare.assert_equal(p.ToExpression(), sym.Expression())

    def test_constructor_maptype(self):
        m = {sym.Monomial(x): sym.Expression(3),
             sym.Monomial(y): sym.Expression(2)}  # 3x + 2y
        p1 = sym.Polynomial(m)
        p2 = sym.Polynomial(map=m)
        expected = 3 * x + 2 * y
        numpy_compare.assert_equal(p1.ToExpression(), expected)
        numpy_compare.assert_equal(p2.ToExpression(), expected)

    def test_constructor_monomial(self):
        m = sym.Monomial(x, 2)
        p1 = sym.Polynomial(m)
        p2 = sym.Polynomial(m=m)
        expected = "pow(x, 2)"
        numpy_compare.assert_equal(p1.ToExpression(), expected)
        numpy_compare.assert_equal(p2.ToExpression(), expected)

    def test_constructor_expression(self):
        e = 2 * x + 3 * y
        p1 = sym.Polynomial(e)
        p2 = sym.Polynomial(e=e)
        numpy_compare.assert_equal(p1.ToExpression(), e)
        numpy_compare.assert_equal(p2.ToExpression(), e)

    def test_constructor_expression_indeterminates(self):
        e = a * x + b * y + c * z
        p1 = sym.Polynomial(e, sym.Variables([x, y, z]))
        p2 = sym.Polynomial(e=e, indeterminates=sym.Variables([x, y, z]))
        p3 = sym.Polynomial(e=e, indeterminates=[x, y, z])
        decision_vars = sym.Variables([a, b, c])
        indeterminates = sym.Variables([x, y, z])
        self.assertEqual(p1.indeterminates(), indeterminates)
        self.assertEqual(p1.decision_variables(), decision_vars)
        self.assertEqual(p2.indeterminates(), indeterminates)
        self.assertEqual(p2.decision_variables(), decision_vars)
        self.assertEqual(p3.indeterminates(), indeterminates)
        self.assertEqual(p3.decision_variables(), decision_vars)

    def test_set_indeterminates(self):
        e = a * x * x + b * y + c * z
        indeterminates1 = sym.Variables([x, y, z])
        p = sym.Polynomial(e, indeterminates1)
        self.assertEqual(p.TotalDegree(), 2)

        indeterminates2 = sym.Variables([a, b, c])
        p.SetIndeterminates(indeterminates2)
        self.assertEqual(p.TotalDegree(), 1)

        p.SetIndeterminates(new_indeterminates=indeterminates1)
        self.assertEqual(p.TotalDegree(), 2)

    def test_degree_total_degree(self):
        e = a * (x ** 2) + b * (y ** 3) + c * z
        p = sym.Polynomial(e, [x, y, z])
        self.assertEqual(p.Degree(x), 2)
        self.assertEqual(p.Degree(v=y), 3)
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
        result = p.Differentiate(x=x)
        numpy_compare.assert_equal(result.ToExpression(), 2 * a * x)

    def test_integrate(self):
        e = 3 * a * (x ** 2)
        p = sym.Polynomial(e, [x])
        result = p.Integrate(x)  # = ax³
        numpy_compare.assert_equal(result.ToExpression(), a * x**3)
        result = p.Integrate(x, -1, 1)  # = 2a
        numpy_compare.assert_equal(result.ToExpression(), 2 * a)
        result = p.Integrate(x=x, a=-1, b=1)
        numpy_compare.assert_equal(result.ToExpression(), 2 * a)

    def test_add_product(self):
        p = sym.Polynomial()
        m = sym.Monomial(x)
        p.AddProduct(sym.Expression(3), m)  # p += 3 * x
        numpy_compare.assert_equal(p.ToExpression(), 3 * x)
        p.AddProduct(coeff=sym.Expression(3), m=m)  # p += 3 * x
        numpy_compare.assert_equal(p.ToExpression(), 6 * x)

    def test_remove_terms_with_small_coefficients(self):
        e = 3 * x + 1e-12 * y
        p = sym.Polynomial(e, [x, y])
        q = p.RemoveTermsWithSmallCoefficients(1e-6)
        numpy_compare.assert_equal(q.ToExpression(), 3 * x)
        e = 3 * x + 1e-12 * y
        p = sym.Polynomial(e, [x, y])
        q = p.RemoveTermsWithSmallCoefficients(coefficient_tol=1e-6)
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
            p.CoefficientsAlmostEqual(
                p=(p + sym.Polynomial(2e-6 * x)), tolerance=1e-6))

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

        def check_jacobian(J):
            numpy_compare.assert_equal(J[0], p_dx)
            numpy_compare.assert_equal(J[1], p_dy)

        vars = [x, y]
        check_jacobian(p.Jacobian(vars))
        check_jacobian(p.Jacobian(vars=vars))

    def test_jacobian_matrix(self):
        p1 = sym.Polynomial(x * x + y, [x, y])      # p1 = x² + y
        p2 = sym.Polynomial(2 * x + y * y, [x, y])  # p2 = 2x + y²
        p1_dx = sym.Polynomial(2 * x, [x, y])       # ∂p1/∂x = 2x
        p1_dy = sym.Polynomial(1.0, [x, y])         # ∂p1/∂y =  1
        p2_dx = sym.Polynomial(2, [x, y])           # ∂p1/∂x = 2
        p2_dy = sym.Polynomial(2 * y, [x, y])       # ∂p1/∂y =  2y

        def check_jacobian(J):
            numpy_compare.assert_equal(J[0, 0], p1_dx)
            numpy_compare.assert_equal(J[0, 1], p1_dy)
            numpy_compare.assert_equal(J[1, 0], p2_dx)
            numpy_compare.assert_equal(J[1, 1], p2_dy)

        f = [p1, p2]
        vars = [x, y]
        check_jacobian(sym.Jacobian(f, vars))
        check_jacobian(sym.Jacobian(f=f, vars=vars))

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
        self.assertEqual(p.Evaluate(env=env),
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
            p.EvaluatePartial(env=env),
            sym.Polynomial(env[a] * x * x + env[b] * x + env[c], [x]))
        numpy_compare.assert_equal(
            p.EvaluatePartial(a, 2),
            sym.Polynomial(2 * x * x + b * x + c, [x]))
        numpy_compare.assert_equal(
            p.EvaluatePartial(var=a, c=2),
            sym.Polynomial(2 * x * x + b * x + c, [x]))


class TestExtractVariablesFromExpression(unittest.TestCase):
    def test(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = x + sym.sin(y) + x*y
        variables, map_var_to_index = sym.ExtractVariablesFromExpression(e)
        self.assertEqual(variables.shape, (2,))
        self.assertNotEqual(variables[0].get_id(), variables[1].get_id())
        self.assertEqual(len(map_var_to_index), 2)
        for i in range(2):
            self.assertEqual(map_var_to_index[variables[i].get_id()], i)


class TestDecomposeAffineExpression(unittest.TestCase):
    def test(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = 2 * x + 3 * y + 4
        variables, map_var_to_index = sym.ExtractVariablesFromExpression(e)
        coeffs, constant_term = sym.DecomposeAffineExpression(
            e, map_var_to_index)
        self.assertEqual(constant_term, 4)
        self.assertEqual(coeffs.shape, (2,))
        self.assertEqual(coeffs[map_var_to_index[x.get_id()]], 2)
        self.assertEqual(coeffs[map_var_to_index[y.get_id()]], 3)


class TestDecomposeAffineExpressions(unittest.TestCase):
    def test(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = [2 * x + 3 * y + 4, 4 * x + 2 * y, 3 * x + 5]
        M, v = sym.DecomposeAffineExpressions(e, [x, y])
        np.testing.assert_allclose(M, np.array([[2, 3], [4., 2.], [3., 0.]]))
        np.testing.assert_allclose(v, np.array([4., 0., 5.]))

        A, b, variables = sym.DecomposeAffineExpressions(e)
        self.assertEqual(variables.shape, (2,))
        np.testing.assert_allclose(b, np.array([4., 0., 5.]))
        if variables[0].get_id() == x.get_id():
            np.testing.assert_allclose(
                A, np.array([[2., 3.], [4., 2.], [3., 0.]]))
            self.assertEqual(variables[1].get_id(), y.get_id())
        else:
            np.testing.assert_allclose(
                A, np.array([[3., 2.], [2., 4.], [0., 3.]]))
            self.assertEqual(variables[0].get_id(), y.get_id())
            self.assertEqual(variables[1].get_id(), x.get_id())


class TestDecomposeLinearExpressions(unittest.TestCase):
    def test(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = [2 * x + 3 * y, 4 * x + 2 * y, 3 * x]
        M = sym.DecomposeLinearExpressions(e, [x, y])
        np.testing.assert_array_equal(
            M, np.array([[2, 3], [4., 2.], [3., 0.]]))


class TestDecomposeQuadraticPolynomial(unittest.TestCase):
    def test(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = x * x + 2 * y * y + 4 * x * y + 3 * x + 2 * y + 4
        poly = sym.Polynomial(e, [x, y])
        variables, map_var_to_index = sym.ExtractVariablesFromExpression(e)
        Q, b, c = sym.DecomposeQuadraticPolynomial(poly, map_var_to_index)
        self.assertEqual(c, 4)
        x_idx = map_var_to_index[x.get_id()]
        y_idx = map_var_to_index[y.get_id()]
        self.assertEqual(Q[x_idx, x_idx], 2)
        self.assertEqual(Q[x_idx, y_idx], 4)
        self.assertEqual(Q[y_idx, x_idx], 4)
        self.assertEqual(Q[y_idx, y_idx], 4)
        self.assertEqual(Q.shape, (2, 2))
        self.assertEqual(b[x_idx], 3)
        self.assertEqual(b[y_idx], 2)
        self.assertEqual(b.shape, (2,))


class TestDecomposeLumpedParameters(unittest.TestCase):
    def test(self):
        x = sym.Variable("x")
        a = sym.Variable("a")
        b = sym.Variable("b")

        f = [a + x, a*a*x*x]
        [W, alpha, w0] = sym.DecomposeLumpedParameters(f, [a, b])
        numpy_compare.assert_equal(W,
                                   [[sym.Expression(1), sym.Expression(0)],
                                    [sym.Expression(0), x*x]])
        numpy_compare.assert_equal(alpha, [sym.Expression(a), a*a])
        numpy_compare.assert_equal(w0, [sym.Expression(x), sym.Expression(0)])


class TestUnapplyExpression(unittest.TestCase):
    def setUp(self):
        # For these kinds of expressions, the unapplied args should only ever
        # be of type Expression or float.
        self._composite_expressions = [
            # Abs
            sym.abs(e_x),
            # Acos
            sym.acos(e_x),
            # Add
            1.0 + (2.0 * x) + (3.0 * y * z),
            # Asin
            sym.asin(e_x),
            # Atan
            sym.atan(e_x),
            # Atan2
            sym.atan2(e_x, e_y),
            # Ceil
            sym.ceil(e_x),
            # Constant
            sym.Expression(1.0),
            # Cos
            sym.cos(e_x),
            # Cosh
            sym.cosh(e_x),
            # Div
            e_x / e_y,
            # Exp
            sym.exp(e_x),
            # Floor
            sym.floor(e_x),
            # Log
            sym.log(e_x),
            # Max
            sym.max(e_x, e_y),
            # Min
            sym.min(e_x, e_y),
            # Mul
            2.0 * x * sym.pow(y, z),
            # NaN
            sym.Expression(np.nan),
            # Pow
            sym.pow(e_x, e_y),
            # Sin
            sym.sin(e_x),
            # Sinh
            sym.sinh(e_x),
            # Sqrt
            sym.sqrt(e_x),
            # Tan
            sym.tan(e_x),
            # Tanh
            sym.tanh(e_x),
        ]
        # For these kinds of expressions, at least one of the unapplied args
        # will not be an Expression nor float.
        self._other_expressions = [
            # IfThenElse
            sym.if_then_else(e_x < e_y, e_x, e_y),
            # UninterpretedFunction
            sym.uninterpreted_function("name", [e_x, e_y]),
            # Var
            e_x,
        ]

    def test_round_trip(self):
        """Focused unit test to check each kind of Expression at least once."""
        for e in self._composite_expressions + self._other_expressions:
            with self.subTest(e=e):
                self._check_one_round_trip(e)

    def _check_one_round_trip(self, e):
        ctor, args = e.Unapply()
        result = ctor(*args)
        self.assertTrue(e.EqualTo(result), msg=repr(result))

    def test_is_composite(self):
        """Confirms that some kinds of expressions have only expressions or
        floats as children.
        """
        for e in self._composite_expressions:
            with self.subTest(e=e):
                self._check_one_composite(e)

    def _check_one_composite(self, e):
        ctor, args = e.Unapply()
        self.assertGreater(len(args), 0)
        for arg in args:
            self.assertIsInstance(arg, (sym.Expression, float))

    def test_replace(self):
        """Acceptance test that shows how to perform substitutions."""
        e1 = x * sym.sin(y) + 2.0 * sym.exp(sym.sin(y))
        sy = sym.Variable("sy")
        e2 = self._replace(e1, sym.sin(y), sy).Expand()
        self.assertEqual(str(e2), "((x * sy) + 2 * exp(sy))")

    def _replace(self, expr, old_subexpr, new_subexpr):
        if not isinstance(expr, sym.Expression):
            return expr
        if expr.EqualTo(old_subexpr):
            return new_subexpr
        ctor, old_args = expr.Unapply()
        new_args = [
            self._replace(arg, old_subexpr, new_subexpr)
            for arg in old_args
        ]
        return ctor(*new_args)


class TestUnapplyFormula(unittest.TestCase):
    def setUp(self):
        self._relational_formulas = [
            x == y,
            x != y,
            x > y,
            x >= y,
            x < y,
            x <= y,
        ]
        self._compound_formulas = [
            sym.logical_and((x > y), (x > z)),
            sym.logical_or((x > y), (x > z)),
            sym.logical_not(x < y),
        ]
        self._misc_formulas = [
            sym.Formula.False_(),
            sym.Formula.True_(),
            sym.Formula(boolean),
            sym.forall(vars=sym.Variables([x]), f=(x > 0)),
            sym.isnan(e_x),
            sym.positive_semidefinite([e_x]),
        ]
        self._all_formulas = (
            self._relational_formulas
            + self._compound_formulas
            + self._misc_formulas
        )

    def test_round_trip(self):
        """Focused unit test to check each kind of Formula at least once."""
        for f in self._all_formulas:
            with self.subTest(f=f):
                self._check_one_round_trip(f)

    def _check_one_round_trip(self, f):
        ctor, args = f.Unapply()
        result = ctor(*args)
        self.assertTrue(f.EqualTo(result), msg=repr(result))

    def test_all_relational(self):
        """Relational formulas should have Expression args."""
        for f in self._relational_formulas:
            with self.subTest(f=f):
                self._check_one_relational(f)

    def _check_one_relational(self, f):
        ctor, args = f.Unapply()
        self.assertEqual(len(args), 2)
        self.assertIsInstance(args[0], sym.Expression)
        self.assertIsInstance(args[1], sym.Expression)

    def test_all_compound(self):
        """Relational formulas should have Formula args."""
        for f in self._compound_formulas:
            with self.subTest(f=f):
                self._check_one_compound(f)

    def _check_one_compound(self, f):
        ctor, args = f.Unapply()
        self.assertGreaterEqual(len(args), 1)
        for arg in args:
            self.assertIsInstance(arg, sym.Formula)
