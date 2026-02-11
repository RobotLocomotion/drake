import copy
import unittest

from pydrake.multibody.contact_solvers import (
    IcfSolverParameters,
)


class TestContactSolvers(unittest.TestCase):
    def test_icf_solver_parameters(self):
        dut = IcfSolverParameters(max_iterations=1)
        self.assertIn("max_iterations", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)
