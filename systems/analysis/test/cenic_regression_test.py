import unittest

import numpy as np

from pydrake.geometry import SceneGraphConfig
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder


class CenicRegressionTest(unittest.TestCase):
    """Some basic high-level simulation tests for the CENIC integrator."""

    def setUp(self):
        """Define some little MJCF models for testing."""
        self.cylinder_xml = """
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <geom name="table_top" type="box" size="0.6 1.1 0.05"/>
    <body>
      <joint type="free"/>
      <geom name="object" type="cylinder" euler="80 0 0" size="0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>"""

        self.damped_double_pendulum_xml = """
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <body>
      <joint type="hinge" axis="0 1 0" pos="0 0 0.1" damping="1e-2"/>
      <geom type="capsule" size="0.01 0.1"/>
      <body>
        <joint type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="1e-2"/>
        <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

    def create_system_setup(self, xml, time_step, use_hydroelastics):
        """Do some boilerplate system setup for the given MJCF model."""
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
        parser = Parser(plant)
        parser.AddModelsFromString(xml, "xml")
        plant.Finalize()

        if use_hydroelastics:
            sg_config = SceneGraphConfig()
            sg_config.default_proximity_properties.compliance_type = "compliant"
            scene_graph.set_config(sg_config)

        return builder, plant

    def create_cenic_simulation(self, builder, plant):
        """Configure a Simulator to use the CENIC integrator."""
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, context)

        config = SimulatorConfig()
        config.integration_scheme = "cenic"
        config.accuracy = 1e-3
        config.max_step_size = 0.1
        config.use_error_control = True
        config.publish_every_time_step = True
        ApplySimulatorConfig(config, simulator)

        ci = simulator.get_mutable_integrator()
        ci.set_plant(plant)
        plant_context = plant.GetMyContextFromRoot(context)

        return simulator, plant_context

    def create_discrete_simulation(self, builder, plant):
        """Configure a Simulator for discrete-time simulation."""
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, context)
        plant_context = plant.GetMyContextFromRoot(context)
        return simulator, plant_context

    def compare_with_discrete_sap(self, xml, x0, sim_time, use_hydro, tol):
        """Run a short simulation comparing CENIC with discrete SAP."""
        # Run a short simulation with CENIC
        builder, plant = self.create_system_setup(
            xml=xml, time_step=0.0, use_hydroelastics=use_hydro
        )
        simulator, plant_context = self.create_cenic_simulation(builder, plant)
        plant.SetPositionsAndVelocities(plant_context, x0)
        simulator.AdvanceTo(sim_time)
        q_cenic = plant.GetPositions(plant_context)

        # Run the same simulation with discrete SAP
        builder, plant = self.create_system_setup(
            xml=xml, time_step=0.001, use_hydroelastics=use_hydro
        )
        simulator, plant_context = self.create_discrete_simulation(
            builder, plant
        )
        plant.SetPositionsAndVelocities(plant_context, x0)
        simulator.AdvanceTo(sim_time)
        q_discrete = plant.GetPositions(plant_context)

        # Check that the final configurations are similar.
        np.testing.assert_allclose(q_cenic, q_discrete, atol=tol, rtol=tol)

    def test_cylinder_hydro(self):
        """Drop a simple cylinder with point contact on a table."""
        # Initial state x = [q, v], with some non-trivial sideways velocity
        x0 = np.array(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        )

        self.compare_with_discrete_sap(
            xml=self.cylinder_xml,
            x0=x0,
            sim_time=1.0,
            use_hydro=True,
            tol=5e-2,
        )

    def test_cylinder_point(self):
        """Drop a simple cylinder with point contact on a table."""
        # Initial state x = [q, v], with some non-trivial sideways velocity
        x0 = np.array(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        )

        self.compare_with_discrete_sap(
            xml=self.cylinder_xml,
            x0=x0,
            sim_time=1.0,
            use_hydro=False,
            tol=5e-2,
        )

    def test_damped_double_pendulum(self):
        """Simulate a damped double pendulum without contact."""
        x0 = np.array([0.5, -0.5, 0.1, 0.2])

        self.compare_with_discrete_sap(
            xml=self.damped_double_pendulum_xml,
            x0=x0,
            sim_time=2.0,
            use_hydro=False,
            tol=5e-2,
        )
