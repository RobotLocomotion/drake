import unittest

from pydrake.common.test_utilities import numpy_compare
import pydrake.multibody.cenic as mut
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
        parameters = dut.get_solver_parameters()
        self.assertIsNotNone(parameters)
        dut.SetSolverParameters(parameters=parameters)

        # Confirm the IntegratorBase inheritance.
        dut.set_fixed_step_mode(True)
