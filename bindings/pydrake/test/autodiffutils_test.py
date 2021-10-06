import pydrake.autodiffutils as mut

from pydrake.autodiffutils import (
    AutoDiffXd,
    ExtractGradient,
    ExtractValue,
    InitializeAutoDiff,
    InitializeAutoDiffTuple,
)

# TODO(sherm1) To be deprecated asap.
from pydrake.autodiffutils import (
    autoDiffToGradientMatrix,
    autoDiffToValueMatrix,
    initializeAutoDiff,
    initializeAutoDiffGivenGradientMatrix,
    initializeAutoDiffTuple,
)

import copy
import itertools
import unittest

import numpy as np
import pydrake.math as drake_math

from pydrake.test.algebra_test_util import ScalarAlgebra, VectorizedAlgebra
from pydrake.test.autodiffutils_test_util import (
    autodiff_scalar_pass_through,
    autodiff_vector_pass_through,
    autodiff_vector3_pass_through,
)
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.test_utilities.pickle_compare import assert_pickle

# Use convenience abbreviation.
AD = AutoDiffXd


def check_logical(func, a, b, expected):
    # Checks logical operations, with broadcasting, checking that `a` and `b`
    # (of type `T`) have compatible logical operators when the left or right
    # operands are `float`s. Specifically, tests:
    # - f(T, T)
    # - f(T, float)
    # - f(float, T)
    numpy_compare.assert_equal(func(a, b), expected)
    af = numpy_compare.to_float(a)
    bf = numpy_compare.to_float(b)
    numpy_compare.assert_equal(func(a, bf), expected)
    numpy_compare.assert_equal(func(af, b), expected)


class TestAutoDiffXd(unittest.TestCase):
    def test_scalar_api(self):
        a = AD(1, [1., 0])
        self.assertEqual(a.value(), 1.)
        numpy_compare.assert_equal(a.derivatives(), [1., 0])
        self.assertEqual(str(a), "AD{1.0, nderiv=2}")
        self.assertEqual(repr(a), "<AutoDiffXd 1.0 nderiv=2>")
        numpy_compare.assert_equal(a, a)
        # Test construction from `float` and `int`.
        numpy_compare.assert_equal(AD(1), AD(1., []))
        numpy_compare.assert_equal(AD(1.), AD(1., []))
        # Test implicit conversion from a simple dtype to AutoDiff.
        numpy_compare.assert_equal(
            autodiff_scalar_pass_through(1),  # int
            AD(1., []))
        numpy_compare.assert_equal(
            autodiff_scalar_pass_through(1.),  # float
            AD(1., []))
        # Test explicit conversion to float.
        with self.assertRaises(TypeError) as cm:
            float(a)
        self.assertIn(
            "not 'pydrake.autodiffutils.AutoDiffXd'", str(cm.exception))
        a_scalar = np.array(a)
        with self.assertRaises(TypeError) as cm:
            float(a_scalar)
        if np.lib.NumpyVersion(np.__version__) < "1.14.0":
            self.assertEqual(
                "don't know how to convert scalar number to float",
                str(cm.exception))
        else:
            self.assertEqual(
                "float() argument must be a string or a number, not "
                "'pydrake.autodiffutils.AutoDiffXd'",
                str(cm.exception))
        # Test multi-element pass-through.
        x = np.array([AD(1.), AD(2.), AD(3.)])
        numpy_compare.assert_equal(autodiff_vector_pass_through(x), x)
        # Ensure fixed-size vectors are correctly converted (#9886).
        numpy_compare.assert_equal(autodiff_vector3_pass_through(x), x)
        # Ensure we can copy.
        numpy_compare.assert_equal(copy.copy(a), a)
        numpy_compare.assert_equal(copy.deepcopy(a), a)
        # Ensure that we can pickle.
        assert_pickle(self, a, lambda x: x)

    def test_array_api(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        x = np.array([a, b])
        self.assertEqual(x.dtype, object)
        # Idempotent check.
        numpy_compare.assert_equal(x, x)
        # Conversion.
        with self.assertRaises(TypeError):
            # Avoid implicit coercion, as this will imply information loss.
            xf = np.zeros(2, dtype=np.float)
            xf[:] = x
        with self.assertRaises(TypeError):
            # We could define `__float__` to allow this, but then that will
            # enable implicit coercion, which we should avoid.
            xf = x.astype(dtype=np.float)
        # Presently, does not convert.
        x = np.zeros((3, 3), dtype=AD)
        self.assertFalse(isinstance(x[0, 0], AD))
        x = np.eye(3).astype(AD)
        self.assertFalse(isinstance(x[0, 0], AD))
        # Test implicit conversion.
        numpy_compare.assert_equal(
            autodiff_vector_pass_through([1, 2]),  # int
            [AD(1., []), AD(2., [])])
        numpy_compare.assert_equal(
            autodiff_vector_pass_through([1., 2.]),  # float
            [AD(1., []), AD(2., [])])

    def _check_algebra(self, algebra):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        c_scalar = AD(0, [1., 0])
        d_scalar = AD(1, [0, 1.])
        a, b, c, d = map(
            algebra.to_algebra, (a_scalar, b_scalar, c_scalar, d_scalar))

        # Arithmetic
        numpy_compare.assert_equal(-a, AD(-1, [-1., 0]))
        numpy_compare.assert_equal(a + b, AD(3, [1, 1]))
        numpy_compare.assert_equal(a + 1, AD(2, [1, 0]))
        numpy_compare.assert_equal(1 + a, AD(2, [1, 0]))
        numpy_compare.assert_equal(a - b, AD(-1, [1, -1]))
        numpy_compare.assert_equal(a - 1, AD(0, [1, 0]))
        numpy_compare.assert_equal(1 - a, AD(0, [-1, 0]))
        numpy_compare.assert_equal(a * b, AD(2, [2, 1]))
        numpy_compare.assert_equal(a * 2, AD(2, [2, 0]))
        numpy_compare.assert_equal(2 * a, AD(2, [2, 0]))
        numpy_compare.assert_equal(a / b, AD(1./2, [1./2, -1./4]))
        numpy_compare.assert_equal(a / 2, AD(0.5, [0.5, 0]))
        numpy_compare.assert_equal(2 / a, AD(2, [-2, 0]))
        # Logical
        check_logical(lambda x, y: x == y, a, a, True)
        check_logical(algebra.eq, a, a, True)
        check_logical(lambda x, y: x != y, a, a, False)
        check_logical(algebra.ne, a, a, False)
        check_logical(lambda x, y: x < y, a, b, True)
        check_logical(algebra.lt, a, b, True)
        check_logical(lambda x, y: x <= y, a, b, True)
        check_logical(algebra.le, a, b, True)
        check_logical(lambda x, y: x > y, a, b, False)
        check_logical(algebra.gt, a, b, False)
        check_logical(lambda x, y: x >= y, a, b, False)
        check_logical(algebra.ge, a, b, False)
        # Additional math
        # - See `math_overloads_test` for scalar overloads.
        numpy_compare.assert_equal(a**2, AD(1, [2., 0]))
        numpy_compare.assert_equal(algebra.log(a), AD(0, [1., 0]))
        numpy_compare.assert_equal(algebra.abs(-a), AD(1, [1., 0]))
        numpy_compare.assert_equal(algebra.exp(a), AD(np.e, [np.e, 0]))
        numpy_compare.assert_equal(algebra.sqrt(a), AD(1, [0.5, 0]))
        numpy_compare.assert_equal(algebra.pow(a, 2), AD(1, [2., 0]))
        numpy_compare.assert_equal(algebra.pow(a, 0.5), AD(1, [0.5, 0]))
        numpy_compare.assert_equal(algebra.sin(c), AD(0, [1, 0]))
        numpy_compare.assert_equal(algebra.cos(c), AD(1, [0, 0]))
        numpy_compare.assert_equal(algebra.tan(c), AD(0, [1, 0]))
        numpy_compare.assert_equal(algebra.arcsin(c), AD(0, [1, 0]))
        numpy_compare.assert_equal(algebra.arccos(c), AD(np.pi / 2, [-1, 0]))
        numpy_compare.assert_equal(algebra.arctan2(c, d), AD(0, [1, 0]))
        numpy_compare.assert_equal(algebra.sinh(c), AD(0, [1, 0]))
        numpy_compare.assert_equal(algebra.cosh(c), AD(1, [0, 0]))
        numpy_compare.assert_equal(algebra.tanh(c), AD(0, [1, 0]))
        numpy_compare.assert_equal(algebra.min(a, b), a_scalar)
        numpy_compare.assert_equal(algebra.max(a, b), b_scalar)
        # Because `ceil` and `floor` return `double`, we have to special case
        # this comparison since the matrix is `dtype=object`, even though the
        # elements are all doubles. We must cast it to float.
        # N.B. This would be fixed if we registered a UFunc for these
        # methods, so NumPy would have already returned a `float` array.
        ceil_a = algebra.ceil(a)
        floor_a = algebra.floor(a)
        if isinstance(algebra, VectorizedAlgebra):
            self.assertEqual(ceil_a.dtype, object)
            self.assertIsInstance(ceil_a[0], float)
            ceil_a = ceil_a.astype(float)
            floor_a = floor_a.astype(float)
        numpy_compare.assert_equal(ceil_a, a_scalar.value())
        numpy_compare.assert_equal(floor_a, a_scalar.value())
        # Return value so it can be inspected.
        return a

    def test_scalar_algebra(self):
        a = self._check_algebra(ScalarAlgebra())
        self.assertEqual(type(a), AD)

    def test_array_algebra(self):
        a = self._check_algebra(VectorizedAlgebra())
        self.assertEqual(type(a), np.ndarray)
        self.assertEqual(a.shape, (2,))

    def test_linear_algebra(self):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        A = np.array([[a_scalar, a_scalar]])
        B = np.array([[b_scalar, b_scalar]]).T
        C = np.dot(A, B)
        numpy_compare.assert_equal(C, [[AD(4, [4., 2])]])

        # Before NumPy 1.17, `matmul` was not supported for `dtype=object`
        # (#11332), and `np.dot` should be used instead.
        if np.lib.NumpyVersion(np.__version__) < "1.17.0":
            with self.assertRaises(TypeError):
                C2 = np.matmul(A, B)
        else:
            C2 = np.matmul(A, B)
            numpy_compare.assert_equal(C2, C)

        # Type mixing
        Bf = np.array([[2., 2]]).T
        C2 = np.dot(A, Bf)  # Leverages implicit casting.
        numpy_compare.assert_equal(C2, [[AD(4, [4., 0])]])

        # Other methods.
        X = np.array([[a_scalar, b_scalar], [b_scalar, a_scalar]])
        numpy_compare.assert_equal(np.trace(X), AD(2, [2., 0]))

        # `inv` is a ufunc that we must implement, if possible. However, given
        # that this is currently `dtype=object`, it would be extremely unwise
        # to do so. See #8116 for alternative.
        with self.assertRaises(TypeError):
            Y = np.linalg.inv(X)

        # Use workaround for inverse. For now, just check values.
        X_float = numpy_compare.to_float(X)
        Xinv_float = np.linalg.inv(X_float)
        Xinv = drake_math.inv(X)
        np.testing.assert_equal(numpy_compare.to_float(Xinv), Xinv_float)

    def test_math_utils(self):
        a = InitializeAutoDiff(value=[1, 2, 3], num_derivatives=3,
                               deriv_num_start=0)
        np.testing.assert_array_equal(ExtractValue(auto_diff_matrix=a),
                                      np.array([[1, 2, 3]]).T)
        np.testing.assert_array_equal(ExtractGradient(auto_diff_matrix=a),
                                      np.eye(3))

        a, b = InitializeAutoDiffTuple([1], [2, 3])
        np.testing.assert_array_equal(ExtractValue(a),
                                      np.array([[1]]))
        np.testing.assert_array_equal(ExtractValue(b),
                                      np.array([[2, 3]]).T)
        np.testing.assert_array_equal(ExtractGradient(a),
                                      np.eye(1, 3))
        np.testing.assert_array_equal(ExtractGradient(b),
                                      np.hstack((np.zeros((2, 1)), np.eye(2))))

        c_grad = [[2, 4, 5], [1, -1, 0]]
        c = InitializeAutoDiff(value=[2, 3], gradient=c_grad)
        np.testing.assert_array_equal(ExtractValue(c),
                                      np.array([2, 3]).reshape((2, 1)))
        np.testing.assert_array_equal(ExtractGradient(c),
                                      np.array(c_grad))

    def test_deprecated_math_utils(self):
        with catch_drake_warnings(expected_count=1):
            a = initializeAutoDiff([1, 2, 3])
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToValueMatrix(a),
                                          np.array([[1, 2, 3]]).T)
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToGradientMatrix(a),
                                          np.eye(3))

        with catch_drake_warnings(expected_count=1):
            a, b = initializeAutoDiffTuple([1], [2, 3])

        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToValueMatrix(a),
                                          np.array([[1]]))
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToValueMatrix(b),
                                          np.array([[2, 3]]).T)
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToGradientMatrix(a),
                                          np.eye(1, 3))
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToGradientMatrix(b),
                                          np.hstack((np.zeros((2, 1)),
                                                     np.eye(2))))

        c_grad = [[2, 4, 5], [1, -1, 0]]
        with catch_drake_warnings(expected_count=1):
            c = initializeAutoDiffGivenGradientMatrix([2, 3], c_grad)
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToValueMatrix(c),
                                          np.array([2, 3]).reshape((2, 1)))
        with catch_drake_warnings(expected_count=1):
            np.testing.assert_array_equal(autoDiffToGradientMatrix(c),
                                          np.array(c_grad))

    def test_autodiff_equal_to(self):
        a = AD(1.0, [1.0, 2.0])
        b = AD(1.0, [3.0, 4.0])
        # Test expected contract in terms of `np.vectorize` and derivatives.
        np.testing.assert_array_equal(
            mut.autodiff_equal_to(a, a),
            True,
        )
        np.testing.assert_array_equal(
            mut.autodiff_equal_to([a, a], a),
            [True, True],
        )
        np.testing.assert_array_equal(
            mut.autodiff_equal_to(a, b),
            False,
        )
        np.testing.assert_array_equal(
            mut.autodiff_equal_to([a], [b]),
            [False],
        )
        with self.assertRaises(AssertionError):
            mut.autodiff_equal_to(a, 1.0)

        # Test contract for `semantic=True`.
        empty_deriv = AD(1.0)
        zero_deriv1 = AD(1.0, [0.0])
        zero_deriv2 = AD(1.0, [0.0, 0.0])
        nonzero_deriv2 = AD(1.0, [0.0, 1e-8])
        self.assertTrue(mut.autodiff_equal_to(empty_deriv, empty_deriv))
        self.assertFalse(mut.autodiff_equal_to(empty_deriv, zero_deriv1))
        self.assertTrue(
            mut.autodiff_equal_to(empty_deriv, zero_deriv1, semantic=True)
        )
        self.assertTrue(
            mut.autodiff_equal_to(empty_deriv, zero_deriv2, semantic=True)
        )
        self.assertFalse(
            mut.autodiff_equal_to(zero_deriv1, zero_deriv2, semantic=True)
        )
        self.assertFalse(
            mut.autodiff_equal_to(empty_deriv, nonzero_deriv2, semantic=True)
        )

    def test_vectorized_binary_operator_type_combinatorics(self):
        """
        Tests vectorized binary operator via brute-force combinatorics per
        #15549.

        This complements test with the same name in ``symbolic_test.py``.
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

        x = AD(2.0, [1.0, 2.0])
        y = AD(0.0, [3.0, 4.0])
        T_operands_x = expand_values(x)
        T_operands_y = expand_values(y)

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

        def check_operands(op, lhs_operands, rhs_operands):
            operand_combinatorics_iter = itertools.product(
                lhs_operands, rhs_operands
            )
            op_reverse = operators_reverse[op]
            for lhs, rhs in operand_combinatorics_iter:
                hint_for_error = f"{op.__doc__}: {repr(lhs)}, {repr(rhs)}"
                with numpy_compare.soft_sub_test(hint_for_error):
                    value = op(lhs, rhs)
                    reverse_value = op_reverse(rhs, lhs)
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
