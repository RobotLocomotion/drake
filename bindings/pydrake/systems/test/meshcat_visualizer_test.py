import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import SceneGraph
from pydrake.multibody.tree import UniformGravityFieldElement
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph)
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import (
    MeshcatVisualizer,
    MeshcatContactVisualizer
)
from pydrake.common.eigen_geometry import Isometry3
from pydrake.multibody.plant import MultibodyPlant


class TestMeshcat(unittest.TestCase):
    def test_cart_pole(self):
        """Cart-Pole with simple geometry."""
        file_name = FindResourceOrThrow(
            "drake/examples/multibody/cart_pole/cart_pole.sdf")

        builder = DiagramBuilder()
        cart_pole, scene_graph = AddMultibodyPlantSceneGraph(builder)
        Parser(plant=cart_pole).AddModelFromFile(file_name)
        cart_pole.AddForceElement(UniformGravityFieldElement())
        cart_pole.Finalize()
        assert cart_pole.geometry_source_is_registered()

        visualizer = builder.AddSystem(MeshcatVisualizer(scene_graph,
                                                         zmq_url=None,
                                                         open_browser=False))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        visualizer.get_input_port(0))

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        cart_pole_context = diagram.GetMutableSubsystemContext(
            cart_pole, diagram_context)

        cart_pole_context.FixInputPort(
            cart_pole.get_actuation_input_port().get_index(), [0])

        cart_slider = cart_pole.GetJointByName("CartSlider")
        pole_pin = cart_pole.GetJointByName("PolePin")
        cart_slider.set_translation(context=cart_pole_context, translation=0.)
        pole_pin.set_angle(context=cart_pole_context, angle=2.)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.StepTo(.1)

    def test_kuka(self):
        """Kuka IIWA with mesh geometry."""
        file_name = FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf")
        builder = DiagramBuilder()
        kuka, scene_graph = AddMultibodyPlantSceneGraph(builder)
        Parser(plant=kuka).AddModelFromFile(file_name)
        kuka.AddForceElement(UniformGravityFieldElement())
        kuka.Finalize()

        visualizer = builder.AddSystem(MeshcatVisualizer(scene_graph,
                                                         zmq_url=None,
                                                         open_browser=False))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        visualizer.get_input_port(0))

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        kuka_context = diagram.GetMutableSubsystemContext(
            kuka, diagram_context)

        kuka_context.FixInputPort(
            kuka.get_actuation_input_port().get_index(), np.zeros(
                kuka.get_actuation_input_port().size()))

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.StepTo(.1)

    def test_contact_force(self):
        """A block sitting on a table."""
        object_file_path = FindResourceOrThrow(
            "drake/examples/manipulation_station/models/061_foam_brick.sdf")
        table_file_path = FindResourceOrThrow(
            "drake/examples/kuka_iiwa_arm/models/table/"
            "extra_heavy_duty_table_surface_only_collision.sdf")

        # T: tabletop frame.
        X_TObject = Isometry3.Identity()
        X_TObject.set_translation([0, 0, 0.2])

        builder = DiagramBuilder()
        plant = MultibodyPlant(0.002)
        _, scene_graph = AddMultibodyPlantSceneGraph(builder, plant)
        object_model = Parser(plant=plant).AddModelFromFile(object_file_path)
        table_model = Parser(plant=plant).AddModelFromFile(table_file_path)

        # Weld table to world.
        plant.WeldFrames(
            A=plant.world_frame(),
            B=plant.GetFrameByName("link", table_model))

        plant.AddForceElement(UniformGravityFieldElement())
        plant.Finalize()

        # Add meshcat visualizer.
        viz = builder.AddSystem(
            MeshcatVisualizer(scene_graph,
                              zmq_url=None,
                              open_browser=False))
        builder.Connect(
            scene_graph.get_pose_bundle_output_port(),
            viz.get_input_port(0))

        # Add contact visualizer.
        contact_viz = builder.AddSystem(
            MeshcatContactVisualizer(
                meshcat_viz=viz,
                force_threshold=0,
                contact_force_scale=10,
                plant=plant))
        contact_input_port = contact_viz.GetInputPort("contact_results")
        builder.Connect(
            plant.GetOutputPort("contact_results"),
            contact_input_port)
        builder.Connect(
            scene_graph.get_pose_bundle_output_port(),
            contact_viz.GetInputPort("pose_bundle"))

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        mbp_context = diagram.GetMutableSubsystemContext(
            plant, diagram_context)

        X_WT = plant.CalcRelativeTransform(
            mbp_context,
            plant.world_frame(),
            plant.GetFrameByName("top_center"))

        plant.SetFreeBodyPose(
            mbp_context,
            plant.GetBodyByName("base_link", object_model),
            X_WT.multiply(X_TObject))

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.StepTo(1.0)

        contact_viz_context = (
            diagram.GetMutableSubsystemContext(contact_viz, diagram_context))
        contact_results = contact_viz.EvalAbstractInput(
            contact_viz_context,
            contact_input_port.get_index()).get_value()

        self.assertGreater(contact_results.num_contacts(), 0)
        self.assertEqual(contact_viz._contact_key_counter, 4)
