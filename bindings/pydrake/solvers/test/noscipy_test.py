import unittest

import numpy as np

from pydrake.solvers.mathematicalprogram import (
    LinearConstraint,
)


class TestMathematicalProgram(unittest.TestCase):
    """Confirms that solvers code may be used without scipy, as long as
    the user does not call a sparse-matrix function specifically.
    """

    def test_linear_constraint(self):
        # Constructor with dense A.
        A = np.array([[1, 3, 4], [2., 4., 5]])
        lb = np.array([1, 2.])
        ub = np.array([3., 4.])
        dut = LinearConstraint(A=A, lb=lb, ub=ub)
        self.assertEqual(dut.num_constraints(), 2)
        self.assertEqual(dut.num_vars(), 3)

        # Update with dense A.
        dut.UpdateCoefficients(
            new_A=np.array([[1E-10, 0, 0], [0, 1, 1]]),
            new_lb=np.array([2, 3]), new_ub=np.array([3, 4]))
        dut.RemoveTinyCoefficient(tol=1E-5)
        np.testing.assert_array_equal(
            dut.GetDenseA(), np.array([[0, 0, 0], [0, 1, 1]]))
