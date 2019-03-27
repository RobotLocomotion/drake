"""
Tests for meshcat functionality.

Example workflow visualize a specific test case:

    # Terminal 1: Run Server.
    bazel run @meshcat_python//:meshcat-server -- \
        --zmq-url=tcp://127.0.0.1:6000

    # Terminal 2: Run test case.
    bazel run --run_under='env TEST_ZMQ_URL=tcp://127.0.0.1:6000' \
        //bindings/pydrake/systems:py/meshcat_visualizer_test -- \
        'TestMeshcat.test_point_cloud_visualization'
"""

import os
import unittest

import meshcat
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import SceneGraph
from pydrake.multibody.tree import UniformGravityFieldElement
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph)
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import AbstractValue, DiagramBuilder
from pydrake.systems.meshcat_visualizer import (
    MeshcatVisualizer,
    MeshcatContactVisualizer,
    MeshcatPointCloudVisualizer
)
from pydrake.common.eigen_geometry import Isometry3
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant

import pydrake.perception as mut

ZMQ_URL = os.environ.get("TEST_ZMQ_URL")


class TestMeshcat(unittest.TestCase):
    def setUp(self):
        np.random.seed(42)

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
                                                         zmq_url=ZMQ_URL,
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
        simulator.AdvanceTo(.1)

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
                                                         zmq_url=ZMQ_URL,
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
        simulator.AdvanceTo(.1)

    def test_contact_force(self):
        """A block sitting on a table."""
        object_file_path = FindResourceOrThrow(
            "drake/examples/manipulation_station/models/061_foam_brick.sdf")
        table_file_path = FindResourceOrThrow(
            "drake/examples/kuka_iiwa_arm/models/table/"
            "extra_heavy_duty_table_surface_only_collision.sdf")

        # T: tabletop frame.
        X_TObject = RigidTransform([0, 0, 0.2])

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
                              zmq_url=ZMQ_URL,
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
        simulator.AdvanceTo(1.0)

        contact_viz_context = (
            diagram.GetMutableSubsystemContext(contact_viz, diagram_context))
        contact_results = contact_viz.EvalAbstractInput(
            contact_viz_context,
            contact_input_port.get_index()).get_value()

        self.assertGreater(contact_results.num_contacts(), 0)
        self.assertEqual(contact_viz._contact_key_counter, 4)

    def test_texture_override(self):
        """Draws a textured box to test the texture override pathway."""
        object_file_path = FindResourceOrThrow(
            "drake/systems/sensors/test/models/box_with_mesh.sdf")
        # Find the texture path just to ensure it exists and
        # we're testing the code path we want to.
        FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.png")

        builder = DiagramBuilder()
        plant = MultibodyPlant(0.002)
        _, scene_graph = AddMultibodyPlantSceneGraph(builder, plant)
        object_model = Parser(plant=plant).AddModelFromFile(object_file_path)
        plant.Finalize()

        # Add meshcat visualizer.
        viz = builder.AddSystem(
            MeshcatVisualizer(scene_graph,
                              zmq_url=None,
                              open_browser=False))
        builder.Connect(
            scene_graph.get_pose_bundle_output_port(),
            viz.get_input_port(0))

        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.AdvanceTo(1.0)

    def test_point_cloud_visualization(self):
        """A small point cloud"""

        draw_period = 1 / 30.
        sim_time = draw_period * 3.

        def se3_from_xyz(xyz):
            return Isometry3(np.eye(3), xyz)

        def show_cloud(pc, use_native=False, **kwargs):
            # kwargs go to ctor.
            builder = DiagramBuilder()
            # Add point cloud visualization.
            if use_native:
                viz = meshcat.Visualizer(zmq_url=ZMQ_URL)
            else:
                plant, scene_graph = AddMultibodyPlantSceneGraph(builder)
                plant.Finalize()
                viz = builder.AddSystem(MeshcatVisualizer(
                    scene_graph, zmq_url=ZMQ_URL, open_browser=False))
                builder.Connect(
                    scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))
            pc_viz = builder.AddSystem(
                MeshcatPointCloudVisualizer(viz, **kwargs))
            # Make sure the system runs.
            diagram = builder.Build()
            diagram_context = diagram.CreateDefaultContext()
            context = diagram.GetMutableSubsystemContext(
                pc_viz, diagram_context)
            context.FixInputPort(
                pc_viz.GetInputPort("point_cloud_P").get_index(),
                AbstractValue.Make(pc))
            simulator = Simulator(diagram, diagram_context)
            simulator.set_publish_every_time_step(False)
            simulator.AdvanceTo(sim_time)

        # Generate some random points that are visually noticeable.
        # ~300000 makes the visualizer less responsive on my (Eric) machine.
        count = 100000
        xyzs = np.random.uniform(-0.1, 0.1, (3, count))
        rgbs = np.random.uniform(0., 255.0, (3, count))

        pc = mut.PointCloud(
            count, mut.Fields(mut.BaseField.kXYZs | mut.BaseField.kRGBs))
        pc.mutable_xyzs()[:] = xyzs
        pc.mutable_rgbs()[:] = rgbs

        pc_no_rgbs = mut.PointCloud(count, mut.Fields(mut.BaseField.kXYZs))
        pc_no_rgbs.mutable_xyzs()[:] = xyzs

        show_cloud(pc)
        # Exercise arguments.
        show_cloud(
            pc, use_native=True, name="point_cloud_2",
            X_WP=se3_from_xyz([0.3, 0, 0]))
        show_cloud(pc, name="point_cloud_3", X_WP=se3_from_xyz([0.6, 0, 0]))
        show_cloud(
            pc_no_rgbs, name="point_cloud_4", X_WP=se3_from_xyz([0.9, 0, 0]))
        show_cloud(
            pc_no_rgbs, name="point_cloud_5", X_WP=se3_from_xyz([1.2, 0, 0]),
            default_rgb=[255., 0., 0.])
