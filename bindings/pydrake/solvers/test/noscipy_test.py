import unittest
import warnings

import numpy as np

from pydrake.solvers import (
    LinearConstraint,
)


class TestMathematicalProgram(unittest.TestCase):
    """Confirms that solvers code may be used without scipy, as long as
    the user does not call a sparse-matrix function specifically.

    Note that this test is only effective at finding bugs when scipy is not
    actually available. If you have scipy installed system-wide (e.g., as
    python3-scipy on Ubuntu) then you will need to uninstall that package
    before you can trust the results of this test.
    """

    def test_linear_constraint(self):
        # The LinearConstraint constructor is overloaded to accept the A matrix
        # as either sparse or dense. If the user doesn't have scipy installed
        # they should still be able to call the dense overload, even when the
        # matrix requires conversion (from numpy's row-major default to the
        # col-major requirement for passing a `const Eigen::MatrixXd`).
        A = np.array([[1, 3, 4], [2., 4., 5]])
        lb = np.array([1, 2.])
        ub = np.array([3., 4.])
        dut = LinearConstraint(A=A, lb=lb, ub=ub)
        self.assertEqual(dut.num_constraints(), 2)
        self.assertEqual(dut.num_vars(), 3)

        # UpdateCoefficients is similarly overloaded for sparse or dense, and
        # the user should be able to invoke the dense overload without scipy.
        dut.UpdateCoefficients(
            new_A=np.array([[1E-10, 0, 0], [0, 1, 1]]),
            new_lb=np.array([2, 3]), new_ub=np.array([3, 4]))
        dut.RemoveTinyCoefficient(tol=1E-5)
        np.testing.assert_array_equal(
            dut.GetDenseA(), np.array([[0, 0, 0], [0, 1, 1]]))

        # When testing in Drake CI, scipy will not be installed. When drake
        # developers run this test locally, they might have scipy installed
        # for use outside of Drake. In that case, we should double-check
        # that our Bazel logic for disabling scipy is working properly.
        try:
            import scipy
            self.assertNotIn(
                "/scipy_stub/", scipy.__file__,
                "The BUILD dependencies for this test MUST NOT depend on"
                " :scipy_stub_py, either directly or transitively.")
            self.fail("Somehow, the scipy_none dependency isn't working?!")
        except ModuleNotFoundError:
            pass
