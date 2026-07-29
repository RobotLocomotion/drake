import unittest

from pydrake.common.test_utilities import numpy_compare
import pydrake.multibody.cenic as mut
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    MultibodyPlant,
)
from pydrake.planning import RobotDiagramBuilder_
from pydrake.systems.framework import DiagramBuilder

_ONE_BODY_MJCF = """
<mujoco model="one_body">
  <worldbody>
    <body name="body">
      <joint name="free" type="free"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


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

    def test_icf_continuous_force_reporting(self):
        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        Parser(plant).AddModelsFromString(_ONE_BODY_MJCF, "xml")
        plant.Finalize()

        # Smoke test: the binding is present and callable on a finalized,
        # continuous plant.
        mut.AddIcfContinuousForceReporting(plant=plant)
        builder.Build()

        # An unfinalized plant is rejected.
        plant2 = MultibodyPlant(time_step=0.0)
        with self.assertRaises(Exception):
            mut.AddIcfContinuousForceReporting(plant=plant2)
