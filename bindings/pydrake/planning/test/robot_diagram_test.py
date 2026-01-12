import pydrake.planning as mut  # ruff: isort: skip

import gc
import sys
import unittest
import weakref

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.geometry import SceneGraph_
from pydrake.multibody.plant import MultibodyPlant_, MultibodyPlantConfig
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Context_, DiagramBuilder_


class TestRobotDiagram(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_robot_diagram_builder_default_time_step(self, T):
        Class = mut.RobotDiagramBuilder_[T]
        dut = Class()
        self.assertEqual(
            dut.plant().time_step(), MultibodyPlantConfig().time_step
        )

    @numpy_compare.check_all_types
    def test_robot_diagram_builder(self, T):
        """Tests the full RobotDiagramBuilder API."""
        Class = mut.RobotDiagramBuilder_[T]
        dut = Class(time_step=0.0)
        if T is float:
            dut.parser().AddModels(
                url=(
                    "package://drake_models/iiwa_description/urdf/"
                    + "iiwa14_spheres_dense_collision.urdf"
                )
            )
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
        """Tests the full RobotDiagram API."""
        builder = mut.RobotDiagramBuilder_[T]()
        dut = builder.Build()

        self.assertIsInstance(dut.plant(), MultibodyPlant_[T])
        self.assertIsInstance(dut.mutable_scene_graph(), SceneGraph_[T])
        self.assertIsInstance(dut.scene_graph(), SceneGraph_[T])

        root_context = dut.CreateDefaultContext()
        self.assertIsInstance(
            dut.mutable_plant_context(root_context=root_context), Context_[T]
        )
        self.assertIsInstance(
            dut.plant_context(root_context=root_context), Context_[T]
        )
        self.assertIsInstance(
            dut.mutable_scene_graph_context(root_context=root_context),
            Context_[T],
        )
        self.assertIsInstance(
            dut.scene_graph_context(root_context=root_context), Context_[T]
        )

    def test_lifetime_robot(self):
        """Ensure that diagrams built using RobotDiagram/Builder neither become
        immortal (leak memory forever), nor have unprotected object lifetimes,
        leading to crashes.

        For examples of crashes, see:
        https://github.com/RobotLocomotion/drake/issues/14355
        For examples of immortality, see:
        https://github.com/RobotLocomotion/drake/issues/14387

        Owing, in part, to the immortality problems, there is now the
        expectation that any system reference will keep the diagram
        alive. Ensure that either a system reference or a diagram reference
        will work.
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
                controller.get_input_port_estimated_state(), "estimated_state"
            )
            builder.builder().ExportInput(
                controller.get_input_port_desired_state(), "desired_state"
            )
            builder.builder().ExportOutput(
                controller.get_output_port_control(), "generalized_force"
            )
            diagram = builder.Build()
            return diagram, controller

        # Manage lifetime via diagram reference.
        diagram, controller = make_diagram()
        del controller
        gc.collect()
        # N.B. Without fixes for #14355, we get a segfault when
        # creating the context.
        context = diagram.CreateDefaultContext()
        diagram.GetInputPort("estimated_state").FixValue(context, [])
        diagram.GetInputPort("desired_state").FixValue(context, [])
        u = diagram.GetOutputPort("generalized_force").Eval(context)
        np.testing.assert_equal(u, [])

        # N.B. Without fixes for #14387, the diagram survives all garbage
        # collection attempts.
        spy = weakref.finalize(diagram, lambda: None)
        del diagram
        gc.collect()
        self.assertFalse(spy.alive)

        # Manage lifetime via controller reference
        diagram, controller = make_diagram()
        del diagram
        gc.collect()
        # N.B. Without fixes for #14355, we get a segfault when
        # creating the context.
        context = controller.CreateDefaultContext()
        controller.GetInputPort("estimated_state").FixValue(context, [])
        controller.GetInputPort("desired_state").FixValue(context, [])
        u = controller.GetOutputPort("generalized_force").Eval(context)
        np.testing.assert_equal(u, [])

        # N.B. Without fixes for #14387, the controller survives all garbage
        # collection attempts.
        spy = weakref.finalize(controller, lambda: None)
        del controller
        gc.collect()
        self.assertFalse(spy.alive)

    def test_issue_23161(self):
        # Check that RobotDiagramBuilder resists cross-generational memory
        # leaks facilitated by pybind11 address aliasing hazards.
        for i in range(10):
            builder = mut.RobotDiagramBuilder(time_step=0.05)
            # In the original bug, the reference count of the builder, and the
            # ref-cycle graph it participated in, would grow without bound.
            self.assertEqual(
                sys.getrefcount(builder.builder()),
                1,
                msg=f"at iteration {i}",
            )
            diagram = builder.Build()  # noqa: F841 (unused-variable)
            gc.collect()
