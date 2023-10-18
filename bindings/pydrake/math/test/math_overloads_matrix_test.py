import pydrake.math as mut

import itertools
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.test_utilities import meta, numpy_compare
from pydrake.symbolic import (
    Expression,
    MakeMatrixContinuousVariable,
    Monomial,
    Polynomial,
    Variable,
)


def _matmul_dtype_pairs():
    """Returns the list of type pairs (T1, T2) to use for testing functions
    that operate on a pair of matrix inputs. We'll test all pairs *except* we
    won't mix autodiff with symbolic.
    """
    types = (float, AutoDiffXd, Variable, Expression, Monomial, Polynomial)
    for T1, T2 in itertools.product(types, types):
        any_autodiff = any([T in (AutoDiffXd,) for T in (T1, T2)])
        all_nonsymbolic = all([T in (float, AutoDiffXd) for T in (T1, T2)])
        if any_autodiff and not all_nonsymbolic:
            continue
        yield dict(T1=T1, T2=T2)


class MathOverloadsMatrixTest(unittest.TestCase,
                              metaclass=meta.ValueParameterizedTest):

    def _astype(self, M, dtype, name):
        """Returns matrix like M but with a new dtype."""
        assert M.dtype == np.float64, M.dtype
        if dtype is float:
            return M
        if dtype is AutoDiffXd:
            # Return a copy with a non-zero gradient.
            result = M.astype(dtype=AutoDiffXd)
            result[0, 0] = AutoDiffXd(M[0, 0], np.array([1.0]))
            return result
        if dtype is Variable:
            # Return a like-sized matrix of variables.
            return MakeMatrixContinuousVariable(*M.shape, name)
        if dtype in (Expression, Polynomial, Monomial):
            # Return a like-sized matrix of variables promoted to the dtype.
            return self._astype(M, Variable, name).astype(dtype=dtype)
        assert False

    @meta.run_with_multiple_values(_matmul_dtype_pairs())
    def test_matmul(self, *, T1, T2):
        # Create some sample data for A @ B == [[11.0]].
        A = np.array([[1.0, 2.0]])
        B = np.array([[3.0, 4.0]]).T

        # Convert the dtypes.
        A_T1 = self._astype(A, T1, "A")
        B_T2 = self._astype(B, T2, "B")

        # Compare the fast overload the slow fallback.
        actual = mut.matmul(A_T1, B_T2)
        expected = A_T1 @ B_T2
        self.assertEqual(actual.dtype, expected.dtype)
        numpy_compare.assert_equal(actual, expected)
