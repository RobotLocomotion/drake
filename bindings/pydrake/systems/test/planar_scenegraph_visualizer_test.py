import unittest

from python.runfiles import Create as CreateRunfiles
import numpy as np
import os

from pydrake.common import (
    FindResourceOrThrow,
    MemoryFile,
)
from pydrake.geometry import (
    Box,
    InMemoryMesh,
    Mesh,
    MeshSource,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph, CoulombFriction)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import (
   ConnectPlanarSceneGraphVisualizer, PlanarSceneGraphVisualizer)


class TestPlanarSceneGraphVisualizer(unittest.TestCase):
    def test_cart_pole(self):
        """Cart-Pole with simple geometry."""
        file_name = FindResourceOrThrow(
            "drake/examples/multibody/cart_pole/cart_pole.sdf")
        builder = DiagramBuilder()
        cart_pole, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant=cart_pole).AddModels(file_name)
        cart_pole.Finalize()
        self.assertTrue(cart_pole.geometry_source_is_registered())

        visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph))
        builder.Connect(scene_graph.get_query_output_port(),
                        visualizer.get_geometry_query_input_port())

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
        url = (
            "package://drake_models/iiwa_description/sdf/"
            + "iiwa14_no_collision.sdf")
        builder = DiagramBuilder()
        kuka, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant=kuka).AddModels(url=url)
        kuka.Finalize()

        # Make sure that the frames to visualize exist.
        iiwa = kuka.GetModelInstanceByName("iiwa14")
        kuka.GetFrameByName("iiwa_link_7", iiwa)
        kuka.GetFrameByName("iiwa_link_6", iiwa)

        visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph))
        builder.Connect(scene_graph.get_query_output_port(),
                        visualizer.get_geometry_query_input_port())

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
        box_body = mbp.AddRigidBody("box")
        mbp.WeldFrames(world_body.body_frame(), box_body.body_frame(),
                       RigidTransform())
        mbp.RegisterVisualGeometry(
            box_body, RigidTransform.Identity(), box_shape, "ground_vis",
            np.array([0.5, 0.5, 0.5, 1.]))
        mbp.RegisterCollisionGeometry(
            box_body, RigidTransform.Identity(), box_shape, "ground_col",
            CoulombFriction(0.9, 0.8))
        mbp.Finalize()

        visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph))
        builder.Connect(scene_graph.get_query_output_port(),
                        visualizer.get_geometry_query_input_port())

        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        vis_context = diagram.GetMutableSubsystemContext(
            visualizer, diagram_context)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.AdvanceTo(.1)

        visualizer.draw(vis_context)
        self.assertEqual(visualizer.ax.get_title(), "t = 0.1",)

    def test_mesh_file_parsing(self):
        """
        This test ensures we can load obj files or provide a reasonable error
        message.
        """
        def scene_graph_with_mesh(mesh_shape: Mesh):
            builder = DiagramBuilder()
            mbp, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
            world_body = mbp.world_body()

            mesh_body = mbp.AddRigidBody("mesh")
            mbp.WeldFrames(world_body.body_frame(), mesh_body.body_frame(),
                           RigidTransform())
            mbp.RegisterVisualGeometry(
                mesh_body, RigidTransform.Identity(), mesh_shape, "mesh_vis",
                np.array([0.5, 0.5, 0.5, 1.]))
            mbp.Finalize()

            return scene_graph

        # This mesh should load correctly.
        runfiles = CreateRunfiles()
        mesh_name = runfiles.Rlocation(
            "drake_models/iiwa_description/meshes/iiwa14/visual/"
            "link_0.gltf")
        scene_graph = scene_graph_with_mesh(Mesh(mesh_name))
        PlanarSceneGraphVisualizer(scene_graph)

        # This should load correctly, too, by substituting the .gltf.
        mesh_name_wrong_ext = os.path.splitext(mesh_name)[0] + ".STL"
        scene_graph = scene_graph_with_mesh(Mesh(mesh_name_wrong_ext))
        PlanarSceneGraphVisualizer(scene_graph)

        # This should report that the file does not exist:
        with self.assertRaises(RuntimeError) as cm:
            PlanarSceneGraphVisualizer(
                scene_graph, substitute_collocated_mesh_files=False)
        self.assertIn("has an unsupported extension", str(cm.exception))
        self.assertNotIn("No supported alternative could be found.",
                         str(cm.exception))

        # This should report that the file cannot be read (because it doesn't
        # exist).
        scene_graph = scene_graph_with_mesh(Mesh("garbage.obj"))
        with self.assertRaises(RuntimeError) as cm:
            PlanarSceneGraphVisualizer(scene_graph)
        self.assertIn("cannot read the file", str(cm.exception))

        # This should report that the extension was wrong and no .obj was
        # found.
        scene_graph = scene_graph_with_mesh(Mesh("garbage.STL"))
        with self.assertRaises(RuntimeError) as cm:
            PlanarSceneGraphVisualizer(scene_graph)
        self.assertIn("No supported alternative could be found.",
                      str(cm.exception))

        # This should load correctly and yield a very large patch.
        scene_graph = scene_graph_with_mesh(Mesh(mesh_name, 1e3))
        visualizer = PlanarSceneGraphVisualizer(scene_graph)
        _, _, width, height = visualizer.ax.dataLim.bounds
        self.assertTrue(width > 10.0)
        self.assertTrue(height > 10.0)

        # A valid in-memory mesh happily succeeds.
        obj_name = FindResourceOrThrow("drake/geometry/test/quad_cube.obj")
        source = MeshSource(InMemoryMesh(mesh_file=MemoryFile.Make(obj_name)))
        scene_graph = scene_graph_with_mesh(Mesh(source=source, scale=1.0))
        PlanarSceneGraphVisualizer(scene_graph)

        # An in-memory mesh of known extension but with badly formatted data
        # throws.
        source = MeshSource(InMemoryMesh(
            mesh_file=MemoryFile(contents="Invalid content", extension=".obj",
                                 filename_hint="bad.obj")))
        scene_graph = scene_graph_with_mesh(Mesh(source=source, scale=1.0))
        with self.assertRaises(RuntimeError) as cm:
            PlanarSceneGraphVisualizer(scene_graph)
        self.assertIn("found 0 vertices", str(cm.exception))

        # An in-memory mesh of unknown extension complains about an unsupported
        # extension without searching for fix.
        source = MeshSource(InMemoryMesh(
            mesh_file=MemoryFile(contents="Invalid content", extension=".stl",
                                 filename_hint="bad.obj")))
        scene_graph = scene_graph_with_mesh(Mesh(source=source, scale=1.0))
        with self.assertRaises(RuntimeError) as cm:
            PlanarSceneGraphVisualizer(scene_graph)
        self.assertIn("has an unsupported extension", str(cm.exception))
        self.assertNotIn("No supported alternative could be found.",
                         str(cm.exception))

    def testConnectPlanarSceneGraphVisualizer(self):
        """Cart-Pole with simple geometry."""
        file_name = FindResourceOrThrow(
            "drake/examples/multibody/cart_pole/cart_pole.sdf")
        builder = DiagramBuilder()
        cart_pole, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant=cart_pole).AddModels(file_name)
        cart_pole.Finalize()

        # The function auto connects to the scene graph query object port.
        vis_auto_connect = ConnectPlanarSceneGraphVisualizer(
            builder=builder, scene_graph=scene_graph, xlim=[0.3, 1.2])
        self.assertIsInstance(vis_auto_connect, PlanarSceneGraphVisualizer)
        # Confirm that arguments are passed through.
        self.assertEqual(vis_auto_connect.ax.get_xlim(), (0.3, 1.2))

        # The function connects visualizer to provided query object port.
        vis_port_connect = ConnectPlanarSceneGraphVisualizer(
            builder=builder,
            scene_graph=scene_graph,
            output_port=scene_graph.get_query_output_port())
        vis_port_connect.set_name("vis_port_connect")
        self.assertIsInstance(vis_port_connect, PlanarSceneGraphVisualizer)

        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        vis_auto_connect_context = diagram.GetMutableSubsystemContext(
            vis_auto_connect, diagram_context)
        vis_port_connect_context = diagram.GetMutableSubsystemContext(
            vis_port_connect, diagram_context)

        # Note: we can't simply call diagram.Publish(diagram_context) because
        # the visualizer isn't set to be visible; as such, no drawing work
        # will be done.
        vis_auto_connect.draw(vis_auto_connect_context)
        vis_port_connect.draw(vis_port_connect_context)
