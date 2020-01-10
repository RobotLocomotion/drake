import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph, CoulombFriction)
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import (
    PlanarSceneGraphVisualizer)


class TestPlanarSceneGraphVisualizer(unittest.TestCase):
    def test_cart_pole(self):
        """Cart-Pole with simple geometry."""
        file_name = FindResourceOrThrow(
            "drake/examples/multibody/cart_pole/cart_pole.sdf")
        builder = DiagramBuilder()
        cart_pole, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant=cart_pole).AddModelFromFile(file_name)
        cart_pole.Finalize()
        self.assertTrue(cart_pole.geometry_source_is_registered())

        visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        visualizer.get_input_port(0))

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        cart_pole_context = diagram.GetMutableSubsystemContext(
            cart_pole, diagram_context)
        vis_context = diagram.GetMutableSubsystemContext(
            visualizer, diagram_context)

        cart_pole.get_actuation_input_port().FixValue(cart_pole_context, 0)

        cart_slider = cart_pole.GetJointByName("CartSlider")
        pole_pin = cart_pole.GetJointByName("PolePin")
        cart_slider.set_translation(context=cart_pole_context, translation=0.)
        pole_pin.set_angle(context=cart_pole_context, angle=2.)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.AdvanceTo(.1)

        visualizer.draw(vis_context)
        self.assertEqual(visualizer.ax.get_title(), "t = 0.1",)

    def test_kuka(self):
        """Kuka IIWA with mesh geometry."""
        file_name = FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf")
        builder = DiagramBuilder()
        kuka, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant=kuka).AddModelFromFile(file_name)
        kuka.Finalize()

        # Make sure that the frames to visualize exist.
        iiwa = kuka.GetModelInstanceByName("iiwa14")
        kuka.GetFrameByName("iiwa_link_7", iiwa)
        kuka.GetFrameByName("iiwa_link_6", iiwa)

        frames_to_draw = {"iiwa14": {"iiwa_link_7", "iiwa_link_6"}}
        visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        visualizer.get_input_port(0))

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        kuka_context = diagram.GetMutableSubsystemContext(
            kuka, diagram_context)
        vis_context = diagram.GetMutableSubsystemContext(
            visualizer, diagram_context)

        kuka_actuation_port = kuka.get_actuation_input_port()
        kuka_actuation_port.FixValue(kuka_context,
                                     np.zeros(kuka_actuation_port.size()))

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.AdvanceTo(.1)

        visualizer.draw(vis_context)
        self.assertEqual(visualizer.ax.get_title(), "t = 0.1",)

    def test_procedural_geometry(self):
        """
        This test ensures we can draw procedurally added primitive
        geometry that is added to the world model instance (which has
        a slightly different naming scheme than geometry with a
        non-default / non-world model instance).
        """
        builder = DiagramBuilder()
        mbp, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        world_body = mbp.world_body()
        box_shape = Box(1., 2., 3.)
        # This rigid body will be added to the world model instance since
        # the model instance is not specified.
        box_body = mbp.AddRigidBody("box", SpatialInertia(
            mass=1.0, p_PScm_E=np.array([0., 0., 0.]),
            G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        mbp.WeldFrames(world_body.body_frame(), box_body.body_frame(),
                       RigidTransform())
        mbp.RegisterVisualGeometry(
            box_body, RigidTransform.Identity(), box_shape, "ground_vis",
            np.array([0.5, 0.5, 0.5, 1.]))
        mbp.RegisterCollisionGeometry(
            box_body, RigidTransform.Identity(), box_shape, "ground_col",
            CoulombFriction(0.9, 0.8))
        mbp.Finalize()

        frames_to_draw = {"world": {"box"}}
        visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        visualizer.get_input_port(0))

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        vis_context = diagram.GetMutableSubsystemContext(
            visualizer, diagram_context)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.AdvanceTo(.1)

        visualizer.draw(vis_context)
        self.assertEqual(visualizer.ax.get_title(), "t = 0.1",)
