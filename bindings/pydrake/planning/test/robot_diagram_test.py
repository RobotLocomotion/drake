import pydrake.planning as mut

import gc
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities import numpy_compare
from pydrake.geometry import SceneGraph_
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant_, MultibodyPlantConfig
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Context_, DiagramBuilder_


class TestRobotDiagram(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_robot_diagram_builder_default_time_step(self, T):
        Class = mut.RobotDiagramBuilder_[T]
        dut = Class()
        self.assertEqual(dut.plant().time_step(),
                         MultibodyPlantConfig().time_step)

    @numpy_compare.check_all_types
    def test_robot_diagram_builder(self, T):
        """Tests the full RobotDiagramBuilder API.
        """
        Class = mut.RobotDiagramBuilder_[T]
        dut = Class(time_step=0.0)
        if T == float:
            dut.parser().AddModels(url=(
                "package://drake_models/iiwa_description/urdf/"
                + "iiwa14_spheres_dense_collision.urdf"))
        else:
            # TODO(jwnimmer-tri) Use dut.plant() to manually add some
            # models, bodies, and geometries here.
            pass

        # Sanity check all of the accessors.
        self.assertIsInstance(dut.builder(), DiagramBuilder_[T])
        self.assertIsInstance(dut.plant(), MultibodyPlant_[T])
        self.assertIsInstance(dut.scene_graph(), SceneGraph_[T])

        # Build.
        self.assertFalse(dut.IsDiagramBuilt())
        diagram = dut.Build()
        self.assertTrue(dut.IsDiagramBuilt())
        self.assertIsInstance(diagram, mut.RobotDiagram_[T])

    @numpy_compare.check_all_types
    def test_robot_diagram(self, T):
        """Tests the full RobotDiagram API.
        """
        builder = mut.RobotDiagramBuilder_[T]()
        dut = builder.Build()

        self.assertIsInstance(dut.plant(), MultibodyPlant_[T])
        self.assertIsInstance(dut.mutable_scene_graph(), SceneGraph_[T])
        self.assertIsInstance(dut.scene_graph(), SceneGraph_[T])

        root_context = dut.CreateDefaultContext()
        self.assertIsInstance(
            dut.mutable_plant_context(root_context=root_context),
            Context_[T])
        self.assertIsInstance(
            dut.plant_context(root_context=root_context),
            Context_[T])
        self.assertIsInstance(
            dut.mutable_scene_graph_context(root_context=root_context),
            Context_[T])
        self.assertIsInstance(
            dut.scene_graph_context(root_context=root_context),
            Context_[T])

    def test_issue14355_robot(self):
        """
        XXX rewrite doc
        DiagramBuilder.AddSystem() may not propagate keep alive relationships.
        We use this test to show resolution at a known concrete point of
        failure.
        https://github.com/RobotLocomotion/drake/issues/14355
        """

        def make_diagram():
            # Use a nested function to ensure that all locals get garbage
            # collected quickly.

            # Construct a trivial plant and ID controller.
            # N.B. We explicitly do *not* add this plant to the diagram.
            controller_plant = MultibodyPlant_[float](time_step=0.002)
            controller_plant.Finalize()
            builder = mut.RobotDiagramBuilder()
            controller = builder.builder().AddSystem(
                InverseDynamicsController(
                    controller_plant,
                    kp=[],
                    ki=[],
                    kd=[],
                    has_reference_acceleration=False,
                )
            )
            # Forward ports for ease of testing.
            builder.builder().ExportInput(
                controller.get_input_port_estimated_state(), "x_estimated")
            builder.builder().ExportInput(
                controller.get_input_port_desired_state(), "x_desired")
            builder.builder().ExportOutput(
                controller.get_output_port_control(), "u")
            diagram = builder.Build()
            return diagram

        diagram = make_diagram()
        gc.collect()
        # N.B. Without the workaround for #14355, we get a segfault when
        # creating the context.
        context = diagram.CreateDefaultContext()
        diagram.GetInputPort("x_estimated").FixValue(context, [])
        diagram.GetInputPort("x_desired").FixValue(context, [])
        u = diagram.GetOutputPort("u").Eval(context)
        np.testing.assert_equal(u, [])
