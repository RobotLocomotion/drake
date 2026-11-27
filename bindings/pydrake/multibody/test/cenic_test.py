import copy
import unittest

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
import pydrake.multibody.cenic as mut
from pydrake.multibody.contact_solvers import IcfSolverParameters
from pydrake.planning import RobotDiagramBuilder_


class TestCenic(unittest.TestCase):
    @numpy_compare.check_nonsymbolic_types
    def test_cenic(self, T):
        # Create a diagram.
        builder = RobotDiagramBuilder_[T](time_step=0.0)
        diagram = builder.Build()

        # The context argument is not required.
        CenicIntegrator = mut.CenicIntegrator_[T]
        CenicIntegrator(system=diagram)

        # Create the device under test.
        dut = CenicIntegrator(system=diagram, context=None)

        # Confirm parameter operations.
        full = dut.GetParameters()
        self.assertFalse(full.explicit_external_inputs)
        full.explicit_external_inputs = True
        full.icf_solver.max_iterations = 7
        dut.SetParameters(parameters=full)
        self.assertTrue(dut.GetParameters().explicit_external_inputs)
        self.assertEqual(dut.GetParameters().icf_solver.max_iterations, 7)

        # Confirm the deprecated solver-only parameter operations.
        # Remove on 2027-03-01 per deprecation.
        with catch_drake_warnings(expected_count=1):
            parameters = dut.get_solver_parameters()
        self.assertEqual(parameters.max_iterations, 7)
        parameters.max_iterations = 9
        with catch_drake_warnings(expected_count=1):
            dut.SetSolverParameters(parameters=parameters)
        self.assertTrue(dut.GetParameters().explicit_external_inputs)
        self.assertEqual(dut.GetParameters().icf_solver.max_iterations, 9)

        # Confirm the IntegratorBase inheritance.
        dut.set_fixed_step_mode(True)

    def test_cenic_integrator_parameters(self):
        # The class is bound by the analysis module but is public here.
        self.assertEqual(
            mut.CenicIntegratorParameters.__name__, "CenicIntegratorParameters"
        )
        self.assertEqual(
            mut.CenicIntegratorParameters.__module__, "pydrake.multibody.cenic"
        )
        dut = mut.CenicIntegratorParameters()
        dut.icf_solver = IcfSolverParameters(max_iterations=7)
        self.assertEqual(dut.icf_solver.max_iterations, 7)
        self.assertIn("CenicIntegratorParameters(", repr(dut))
        self.assertIn("icf_solver", repr(dut))
        copy.copy(dut)
        copy.deepcopy(dut)
