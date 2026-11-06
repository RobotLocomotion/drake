import copy
import unittest

from pydrake.multibody.contact_solvers import (
    CenicIntegratorParameters,
    IcfSolverParameters,
)


class TestContactSolvers(unittest.TestCase):
    def test_icf_solver_parameters(self):
        dut = IcfSolverParameters(max_iterations=1)
        self.assertIn("max_iterations", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_cenic_integrator_parameters(self):
        dut = CenicIntegratorParameters()
        dut.icf_solver = IcfSolverParameters(max_iterations=7)
        self.assertEqual(dut.icf_solver.max_iterations, 7)
        self.assertIn("icf_solver", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)
