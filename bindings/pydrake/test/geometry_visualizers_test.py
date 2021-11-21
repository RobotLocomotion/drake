import pydrake.geometry as mut

import unittest

import numpy as np

from drake import lcmt_viewer_load_robot, lcmt_viewer_draw
from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.value import AbstractValue
from pydrake.common.test_utilities import numpy_compare
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.math import RigidTransform
from pydrake.perception import PointCloud
from pydrake.systems.analysis import Simulator_
from pydrake.systems.framework import DiagramBuilder_, InputPort_


class TestGeometryVisualizers(unittest.TestCase):
    @numpy_compare.check_nonsymbolic_types
    def test_drake_visualizer(self, T):
        # Test visualization API.
        SceneGraph = mut.SceneGraph_[T]
        DiagramBuilder = DiagramBuilder_[T]
        Simulator = Simulator_[T]
        lcm = DrakeLcm()
        role = mut.Role.kIllustration
        params = mut.DrakeVisualizerParams(
            publish_period=0.1, role=mut.Role.kIllustration,
            default_color=mut.Rgba(0.1, 0.2, 0.3, 0.4),
            show_hydroelastic=False)
        self.assertEqual(repr(params), "".join([
            "DrakeVisualizerParams("
            "publish_period=0.1, "
            "role=Role.kIllustration, "
            "default_color=Rgba(r=0.1, g=0.2, b=0.3, a=0.4), "
            "show_hydroelastic=False)"]))

        # Add some subscribers to detect message broadcast.
        load_channel = "DRAKE_VIEWER_LOAD_ROBOT"
        draw_channel = "DRAKE_VIEWER_DRAW"
        load_subscriber = Subscriber(
            lcm, load_channel, lcmt_viewer_load_robot)
        draw_subscriber = Subscriber(
            lcm, draw_channel, lcmt_viewer_draw)

        # There are three ways to configure DrakeVisualizer.
        def by_hand(builder, scene_graph, params):
            visualizer = builder.AddSystem(
                mut.DrakeVisualizer_[T](lcm=lcm, params=params))
            builder.Connect(scene_graph.get_query_output_port(),
                            visualizer.query_object_input_port())

        def auto_connect_to_system(builder, scene_graph, params):
            mut.DrakeVisualizer_[T].AddToBuilder(builder=builder,
                                                 scene_graph=scene_graph,
                                                 lcm=lcm, params=params)

        def auto_connect_to_port(builder, scene_graph, params):
            mut.DrakeVisualizer_[T].AddToBuilder(
                builder=builder,
                query_object_port=scene_graph.get_query_output_port(),
                lcm=lcm, params=params)

        for func in [by_hand, auto_connect_to_system, auto_connect_to_port]:
            # Build the diagram.
            builder = DiagramBuilder()
            scene_graph = builder.AddSystem(SceneGraph())
            func(builder, scene_graph, params)

            # Simulate to t = 0 to send initial load and draw messages.
            diagram = builder.Build()
            Simulator(diagram).AdvanceTo(0)
            lcm.HandleSubscriptions(0)
            self.assertEqual(load_subscriber.count, 1)
            self.assertEqual(draw_subscriber.count, 1)
            load_subscriber.clear()
            draw_subscriber.clear()

        # Ad hoc broadcasting.
        scene_graph = SceneGraph()

        mut.DrakeVisualizer_[T].DispatchLoadMessage(
            scene_graph, lcm, params)
        lcm.HandleSubscriptions(0)
        self.assertEqual(load_subscriber.count, 1)
        self.assertEqual(draw_subscriber.count, 0)
        load_subscriber.clear()
        draw_subscriber.clear()

    def test_meshcat(self):
        meshcat = mut.Meshcat(port=7051)
        self.assertEqual(meshcat.port(), 7051)
        with self.assertRaises(RuntimeError):
            meshcat2 = mut.Meshcat(port=7051)
        self.assertIn("http", meshcat.web_url())
        self.assertIn("ws", meshcat.ws_url())
        meshcat.SetObject(path="/test/box",
                          shape=mut.Box(1, 1, 1),
                          rgba=mut.Rgba(.5, .5, .5))
        meshcat.SetTransform(path="/test/box", X_ParentPath=RigidTransform())
        meshcat.SetTransform(path="/test/box", matrix=np.eye(4))
        cloud = PointCloud(4)
        cloud.mutable_xyzs()[:] = np.zeros((3, 4))
        meshcat.SetObject(path="/test/cloud", cloud=cloud,
                          point_size=0.01, rgba=mut.Rgba(.5, .5, .5))
        mesh = mut.TriangleSurfaceMesh(
            triangles=[mut.SurfaceTriangle(
                0, 1, 2), mut.SurfaceTriangle(3, 0, 2)],
            vertices=[[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]])
        meshcat.SetObject(path="/test/triangle_surface_mesh", mesh=mesh,
                          rgba=mut.Rgba(0.3, 0.3, 0.3), wireframe=True,
                          wireframe_line_width=2.0)
        meshcat.SetLine(path="/test/line", vertices=np.eye(3),
                        line_width=2.0, rgba=mut.Rgba(.3, .3, .3))
        meshcat.SetLineSegments(path="/test/line_segments", start=np.eye(3),
                                end=2*np.eye(3), line_width=2.0,
                                rgba=mut.Rgba(.3, .3, .3))
        meshcat.SetTriangleMesh(
            path="/test/triangle_mesh",
            vertices=np.array([[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]]).T,
            faces=np.array([[0, 1, 2], [3, 0, 2]]).T,
            rgba=mut.Rgba(0.3, 0.3, 0.3),
            wireframe=True,
            wireframe_line_width=2.0)
        meshcat.SetProperty(path="/Background",
                            property="visible",
                            value=True)
        meshcat.SetProperty(path="/Lights/DirectionalLight/<object>",
                            property="intensity", value=1.0)
        meshcat.SetProperty(path="/Background", property="top_color",
                            value=[0, 0, 0])
        meshcat.Set2dRenderMode(
            X_WC=RigidTransform(), xmin=-1, xmax=1, ymin=-1, ymax=1)
        meshcat.ResetRenderMode()
        meshcat.AddButton(name="button")
        self.assertEqual(meshcat.GetButtonClicks(name="button"), 0)
        meshcat.DeleteButton(name="button")
        meshcat.AddSlider(name="slider", min=0, max=1, step=0.01, value=0.5)
        meshcat.SetSliderValue(name="slider", value=0.7)
        self.assertAlmostEqual(meshcat.GetSliderValue(
            name="slider"), 0.7, delta=1e-14)
        meshcat.DeleteSlider(name="slider")
        meshcat.DeleteAddedControls()
        self.assertIn("data:application/octet-binary;base64",
                      meshcat.StaticHtml())
        meshcat.Flush()

    def test_meshcat_animation(self):
        animation = mut.MeshcatAnimation(frames_per_second=64)
        self.assertEqual(animation.frames_per_second(), 64)
        self.assertEqual(animation.frame(1.0), 64)
        animation.set_autoplay(play=False)
        self.assertEqual(animation.autoplay(), False)
        animation.set_loop_mode(mode=mut.MeshcatAnimation.LoopMode.kLoopOnce)
        animation.set_loop_mode(mode=mut.MeshcatAnimation.LoopMode.kLoopRepeat)
        animation.set_loop_mode(
            mode=mut.MeshcatAnimation.LoopMode.kLoopPingPong)
        self.assertEqual(animation.loop_mode(),
                         mut.MeshcatAnimation.LoopMode.kLoopPingPong)
        animation.set_repetitions(repetitions=20)
        self.assertEqual(animation.repetitions(), 20)
        animation.set_clamp_when_finished(clamp=False)
        self.assertEqual(animation.clamp_when_finished(), False)
        animation.SetTransform(frame=0, path="test",
                               X_ParentPath=RigidTransform())
        animation.SetProperty(frame=0, path="test", property="bool",
                              value=True)
        animation.SetProperty(frame=0, path="test", property="double",
                              value=32.0)
        animation.SetProperty(frame=0, path="test", property="vector_double",
                              value=[1., 2., 3.])
        meshcat = mut.Meshcat()
        meshcat.SetAnimation(animation)

    @numpy_compare.check_nonsymbolic_types
    def test_meshcat_visualizer(self, T):
        meshcat = mut.Meshcat()
        params = mut.MeshcatVisualizerParams()
        params.publish_period = 0.123
        params.role = mut.Role.kIllustration
        params.default_color = mut.Rgba(0.5, 0.5, 0.5)
        params.prefix = "py_visualizer"
        params.delete_on_initialization_event = False
        self.assertNotIn("object at 0x", repr(params))
        vis = mut.MeshcatVisualizerCpp_[T](meshcat=meshcat, params=params)
        vis.Delete()
        self.assertIsInstance(vis.query_object_input_port(), InputPort_[T])
        animation = vis.StartRecording()
        self.assertIsInstance(animation, mut.MeshcatAnimation)
        self.assertEqual(animation, vis.get_mutable_recording())
        vis.StopRecording()
        vis.PublishRecording()
        vis.DeleteRecording()

        builder = DiagramBuilder_[T]()
        scene_graph = builder.AddSystem(mut.SceneGraph_[T]())
        mut.MeshcatVisualizerCpp_[T].AddToBuilder(builder=builder,
                                                  scene_graph=scene_graph,
                                                  meshcat=meshcat,
                                                  params=params)
        mut.MeshcatVisualizerCpp_[T].AddToBuilder(
            builder=builder,
            query_object_port=scene_graph.get_query_output_port(),
            meshcat=meshcat,
            params=params)

    def test_meshcat_visualizer_scalar_conversion(self):
        meshcat = mut.Meshcat()
        vis = mut.MeshcatVisualizerCpp(meshcat)
        vis_autodiff = vis.ToAutoDiffXd()
        self.assertIsInstance(vis_autodiff,
                              mut.MeshcatVisualizerCpp_[AutoDiffXd])

    @numpy_compare.check_nonsymbolic_types
    def test_meshcat_point_cloud_visualizer(self, T):
        meshcat = mut.Meshcat()
        visualizer = mut.MeshcatPointCloudVisualizerCpp_[T](
            meshcat=meshcat, path="cloud", publish_period=1/12.0)
        visualizer.set_point_size(0.1)
        visualizer.set_default_rgba(mut.Rgba(0, 0, 1, 1))
        context = visualizer.CreateDefaultContext()
        cloud = PointCloud(4)
        cloud.mutable_xyzs()[:] = np.zeros((3, 4))
        visualizer.cloud_input_port().FixValue(
          context, AbstractValue.Make(cloud))
        self.assertIsInstance(visualizer.pose_input_port(), InputPort_[T])
        visualizer.Publish(context)
        visualizer.Delete()
        if T == float:
            ad_visualizer = visualizer.ToAutoDiffXd()
            self.assertIsInstance(
                ad_visualizer, mut.MeshcatPointCloudVisualizerCpp_[AutoDiffXd])
