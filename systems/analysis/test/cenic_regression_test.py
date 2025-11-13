import unittest
import numpy as np
from pydrake.systems.analysis import Simulator, SimulatorConfig, ApplySimulatorConfig
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.geometry import SceneGraphConfig

class CenicRegressionTest(unittest.TestCase):
    """Some basic high-level simulation tests for the CENIC integrator."""

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

    def test_cylinder_on_table(self):
        """Drop a simple cylinder on a table."""
        # MJCF model of the system
        xml = """
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

        # Initial state x = [q, v], with some non-trivial sideways velocity
        x0 = np.array(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        )

        # Run a short simulation with CENIC
        builder, plant = self.create_system_setup(xml=xml, time_step=0.0, use_hydroelastics=True)
        simulator, plant_context = self.create_cenic_simulation(builder, plant)
        plant.SetPositionsAndVelocities(plant_context, x0)
        simulator.AdvanceTo(1.0)
        q_cenic = plant.GetPositions(plant_context)

        # Run the same simulation with discrete SAP
        builder, plant = self.create_system_setup(xml=xml, time_step=0.001, use_hydroelastics=True)
        simulator, plant_context = self.create_discrete_simulation(builder, plant)
        plant.SetPositionsAndVelocities(plant_context, x0)
        simulator.AdvanceTo(1.0)
        q_discrete = plant.GetPositions(plant_context)

        # Check that the final configurations are similar
        self.assertTrue(np.allclose(q_cenic, q_discrete, atol=5e-2))