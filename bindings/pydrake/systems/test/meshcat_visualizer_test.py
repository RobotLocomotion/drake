import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import SceneGraph
from pydrake.multibody.multibody_tree import UniformGravityFieldElement
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer


class TestMeshcat(unittest.TestCase):
    def test_cart_pole(self):
        """Cart-Pole with simple geometry."""
        file_name = FindResourceOrThrow(
            "drake/examples/multibody/cart_pole/cart_pole.sdf")

        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(SceneGraph())
        cart_pole = builder.AddSystem(MultibodyPlant())
        AddModelFromSdfFile(
            file_name=file_name, plant=cart_pole, scene_graph=scene_graph)
        cart_pole.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        cart_pole.Finalize(scene_graph)
        assert cart_pole.geometry_source_is_registered()

        builder.Connect(
            cart_pole.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(cart_pole.get_source_id()))

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
        scene_graph = builder.AddSystem(SceneGraph())
        kuka = builder.AddSystem(MultibodyPlant())
        AddModelFromSdfFile(
            file_name=file_name, plant=kuka, scene_graph=scene_graph)
        kuka.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        kuka.Finalize(scene_graph)
        assert kuka.geometry_source_is_registered()

        builder.Connect(
            kuka.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(kuka.get_source_id()))

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
