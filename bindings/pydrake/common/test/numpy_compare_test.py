import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.test_utilities import numpy_compare
from pydrake.symbolic import Expression, Formula, Variable


class Custom:
    def __init__(self, value):
        assert isinstance(value, str)
        self._str = value

    def __str__(self):
        return self._str

    def assert_eq(self, other):
        assert self._str == other._str, (self, other)

    def assert_ne(self, other):
        if self._str == other._str:
            raise numpy_compare._UnwantedEquality(str((self, other)))


# Hack into private API to register custom comparisons.
registry = numpy_compare._registry
registry.register_to_float(Custom, lambda x: float(str(x)))
registry.register_comparator(
    Custom, Custom, Custom.assert_eq, Custom.assert_ne)
registry.register_comparator(
    Custom, str, numpy_compare._str_eq, numpy_compare._str_ne)


class TestNumpyCompareSimple(unittest.TestCase):
    def test_to_float(self):
        # Scalar.
        xi = 1
        xf = numpy_compare.to_float(xi)
        self.assertEqual(xf.dtype, float)
        self.assertEqual(xi, xf)
        # Array.
        Xi = np.array([1, 2, 3], np.int)
        Xf = numpy_compare.to_float(Xi)
        self.assertEqual(Xf.dtype, float)
        np.testing.assert_equal(Xi, Xf)
        # Custom.
        a = Custom("1.")
        b = Custom("2.")
        self.assertEqual(numpy_compare.to_float(a), 1.)
        A = np.array([a, b])
        np.testing.assert_equal(numpy_compare.to_float(A), [1., 2.])
        # - Convenience float comparators.
        numpy_compare.assert_float_equal(a, 1.)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_float_equal(a, 2.)
        numpy_compare.assert_float_not_equal(a, 2.)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_float_not_equal(a, 1.)
        numpy_compare.assert_float_equal(A, [1., 2.])
        # Check nearness.
        Af_delta = numpy_compare.to_float(A) + 5e-16
        numpy_compare.assert_float_not_equal(A, Af_delta)
        numpy_compare.assert_float_allclose(A, Af_delta)

    def test_resolve_type(self):
        Af = np.array([1., 2.])
        self.assertEqual(numpy_compare.resolve_type(Af), float)
        Ac = np.array([Custom("a"), Custom("b")])
        self.assertEqual(numpy_compare.resolve_type(Ac), Custom)
        with self.assertRaises(AssertionError):
            numpy_compare.resolve_type([])
        with self.assertRaises(AssertionError):
            numpy_compare.resolve_type(["a", 1., None])

    def test_asserts_builtin(self):
        a = 1.
        b = 0.
        # Scalar.
        numpy_compare.assert_equal(a, a)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_equal(a, b)
        numpy_compare.assert_not_equal(a, b)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_not_equal(a, a)
        # Array.
        A = np.array([a, a])
        C = np.array([1., 2.])
        numpy_compare.assert_equal(A, a)
        numpy_compare.assert_equal(C, C)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_equal(A, b)
        numpy_compare.assert_not_equal(A, A + [0, 0.1])
        numpy_compare.assert_not_equal(A, b)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_not_equal(C, C)

    def test_asserts_custom(self):
        a = Custom("a")
        b = Custom("b")
        # Scalar.
        numpy_compare.assert_equal(a, a)
        numpy_compare.assert_equal(a, "a")
        with self.assertRaises(AssertionError):
            numpy_compare.assert_equal(a, b)
        numpy_compare.assert_not_equal(a, b)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_not_equal(a, a)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_not_equal(a, "a")
        # Array.
        A = np.array([a, a])
        C = np.array([Custom("c0"), Custom("c1")])
        numpy_compare.assert_equal(A, a)
        numpy_compare.assert_equal(A, "a")
        numpy_compare.assert_equal(C, C)
        numpy_compare.assert_equal(C, ["c0", "c1"])
        with self.assertRaises(AssertionError):
            numpy_compare.assert_equal(A, b)
        numpy_compare.assert_not_equal(A, [a, b])
        numpy_compare.assert_not_equal(A, ["a", "b"])
        with self.assertRaises(AssertionError):
            numpy_compare.assert_not_equal(C, C)

    def test_asserts_autodiff(self):
        # Test only scalar; other cases are handled by above test case.
        a = AutoDiffXd(1., [1., 0.])
        b = AutoDiffXd(1., [0., 1.])
        c = AutoDiffXd(2., [3., 4.])
        numpy_compare.assert_equal(a, a)
        numpy_compare.assert_allclose(a, a)
        numpy_compare.assert_not_equal(a, b)
        numpy_compare.assert_not_equal(a, c)

    def test_asserts_symbolic(self):
        x = Variable("x")
        y = Variable("y")
        e = x + y
        numpy_compare.assert_equal(x, x)
        numpy_compare.assert_equal(x, "x")
        numpy_compare.assert_not_equal(x, y)
        numpy_compare.assert_equal(e, x + y)
        numpy_compare.assert_equal(e, "(x + y)")
        numpy_compare.assert_not_equal(e, x - y)
        numpy_compare.assert_not_equal(e, "(x - y)")
        numpy_compare.assert_equal(Formula.True_(), True)
        numpy_compare.assert_equal(Formula.False_(), False)
        numpy_compare.assert_not_equal(Formula.True_(), False)
        with self.assertRaises(AssertionError):
            numpy_compare.assert_allclose(x, x)

    def test_decorators(self):
        T_list_all = []
        T_list_nonsymbolic = []

        @numpy_compare.check_all_types
        def decorated_all(arg, T):
            self.assertEqual(arg, 1)
            T_list_all.append(T)

        @numpy_compare.check_nonsymbolic_types
        def decorated_nonsymbolic(arg, T):
            self.assertEqual(arg, 2)
            T_list_nonsymbolic.append(T)

        decorated_all(1)
        self.assertEqual(T_list_all, [float, AutoDiffXd, Expression])

        decorated_nonsymbolic(2)
        self.assertEqual(T_list_nonsymbolic, [float, AutoDiffXd])
