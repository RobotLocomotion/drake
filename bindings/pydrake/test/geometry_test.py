import pydrake.geometry as mut

import unittest
import warnings
from math import pi

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities import numpy_compare
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform_
from pydrake.symbolic import Expression
from pydrake.systems.framework import DiagramBuilder_, InputPort_, OutputPort_
from pydrake.systems.sensors import (
    ImageRgba8U,
    ImageDepth32F,
    ImageLabel16I
    )
from pydrake.common.deprecation import DrakeDeprecationWarning


class TestGeometry(unittest.TestCase):
    def setUp(self):
        warnings.simplefilter('error', DrakeDeprecationWarning)

    @numpy_compare.check_nonsymbolic_types
    def test_scene_graph_api(self, T):
        SceneGraph = mut.SceneGraph_[T]
        InputPort = InputPort_[T]
        OutputPort = OutputPort_[T]

        scene_graph = SceneGraph()
        global_source = scene_graph.RegisterSource("anchored")
        self.assertIsInstance(
            scene_graph.get_source_pose_port(global_source), InputPort)
        self.assertIsInstance(
            scene_graph.get_pose_bundle_output_port(), OutputPort)
        self.assertIsInstance(
            scene_graph.get_query_output_port(), OutputPort)

        # Test limited rendering API.
        scene_graph.AddRenderer("test_renderer",
                                mut.render.MakeRenderEngineVtk(
                                    mut.render.RenderEngineVtkParams()))

    def test_connect_drake_visualizer(self):
        # Test visualization API.
        # Use a mockable so that we can make a smoke test without side
        # effects.
        T = float
        SceneGraph = mut.SceneGraph_[T]
        DiagramBuilder = DiagramBuilder_[T]
        lcm = DrakeMockLcm()

        for role in [mut.Role.kProximity, mut.Role.kIllustration]:
            for i in range(2):
                builder = DiagramBuilder()
                scene_graph = builder.AddSystem(SceneGraph())
                if i == 1:
                    mut.ConnectDrakeVisualizer(
                        builder=builder, scene_graph=scene_graph,
                        lcm=lcm, role=role)
                else:
                    mut.ConnectDrakeVisualizer(
                        builder=builder, scene_graph=scene_graph,
                        pose_bundle_output_port=(
                            scene_graph.get_pose_bundle_output_port()),
                        lcm=lcm, role=role)
                mut.DispatchLoadMessage(
                    scene_graph=scene_graph, lcm=lcm, role=role)

    @numpy_compare.check_nonsymbolic_types
    def test_frame_pose_vector_api(self, T):
        FramePoseVector = mut.FramePoseVector_[T]
        RigidTransform = RigidTransform_[T]
        obj = FramePoseVector()
        frame_id = mut.FrameId.get_new_id()

        obj.set_value(id=frame_id, value=RigidTransform.Identity())
        self.assertEqual(obj.size(), 1)
        self.assertIsInstance(obj.value(id=frame_id), RigidTransform)
        self.assertTrue(obj.has_id(id=frame_id))
        self.assertIsInstance(obj.frame_ids(), list)
        self.assertIsInstance(obj.frame_ids()[0], mut.FrameId)
        obj.clear()
        self.assertEqual(obj.size(), 0)

    def test_query_object_api(self):
        # TODO(eric.cousineau): Create self-contained unittests (#9899).
        # Pending that, the relevant API is exercised via
        # `test_scene_graph_queries` in `plant_test.py`.
        pass

    def test_identifier_api(self):
        cls_list = [
            mut.SourceId,
            mut.FrameId,
            mut.GeometryId,
        ]

        for cls in cls_list:
            with self.assertRaises(DrakeDeprecationWarning) as exc:
                cls()
            self.assertIn(cls.__name__, str(exc.exception))
            a = cls.get_new_id()
            self.assertTrue(a.is_valid())
            b = cls.get_new_id()
            self.assertTrue(a == a)
            self.assertFalse(a == b)
            # N.B. Creation order does not imply value.
            self.assertTrue(a < b or b > a)

    @numpy_compare.check_nonsymbolic_types
    def test_penetration_as_point_pair_api(self, T):
        obj = mut.PenetrationAsPointPair_[T]()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_WCa.shape, (3,))
        self.assertTupleEqual(obj.p_WCb.shape, (3,))
        self.assertEqual(obj.depth, -1.)

    @numpy_compare.check_nonsymbolic_types
    def test_signed_distance_api(self, T):
        obj = mut.SignedDistancePair_[T]()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_ACa.shape, (3,))
        self.assertTupleEqual(obj.p_BCb.shape, (3,))
        self.assertIsInstance(obj.distance, T)
        self.assertTupleEqual(obj.nhat_BA_W.shape, (3,))

    @numpy_compare.check_nonsymbolic_types
    def test_signed_distance_to_point_api(self, T):
        obj = mut.SignedDistanceToPoint_[T]()
        self.assertIsInstance(obj.id_G, mut.GeometryId)
        self.assertTupleEqual(obj.p_GN.shape, (3,))
        self.assertIsInstance(obj.distance, T)
        self.assertTupleEqual(obj.grad_W.shape, (3,))

    def test_shape_constructors(self):
        box_mesh_path = FindResourceOrThrow(
            "drake/systems/sensors/test/models/meshes/box.obj")
        shapes = [
            mut.Sphere(radius=1.0),
            mut.Cylinder(radius=1.0, length=2.0),
            mut.Box(width=1.0, depth=2.0, height=3.0),
            mut.HalfSpace(),
            mut.Mesh(absolute_filename=box_mesh_path, scale=1.0),
            mut.Convex(absolute_filename=box_mesh_path, scale=1.0)
        ]
        for shape in shapes:
            self.assertIsInstance(shape, mut.Shape)

    def test_render_engine_vtk_params(self):
        # Confirm default construction of params.
        params = mut.render.RenderEngineVtkParams()
        self.assertEqual(params.default_label, None)
        self.assertEqual(params.default_diffuse, None)

        label = mut.render.RenderLabel(10)
        diffuse = np.array((1.0, 0.0, 0.0, 0.0))
        params = mut.render.RenderEngineVtkParams(
            default_label=label, default_diffuse=diffuse)
        self.assertEqual(params.default_label, label)
        self.assertTrue((params.default_diffuse == diffuse).all())

    def test_render_depth_camera_properties(self):
        obj = mut.render.DepthCameraProperties(width=320, height=240,
                                               fov_y=pi/6,
                                               renderer_name="test_renderer",
                                               z_near=0.1, z_far=5.0)
        self.assertEqual(obj.width, 320)
        self.assertEqual(obj.height, 240)
        self.assertEqual(obj.fov_y, pi/6)
        self.assertEqual(obj.renderer_name, "test_renderer")
        self.assertEqual(obj.z_near, 0.1)
        self.assertEqual(obj.z_far, 5.0)

    def test_render_label(self):
        RenderLabel = mut.render.RenderLabel
        value = 10
        obj = RenderLabel(value)

        self.assertEqual(value, obj)
        self.assertEqual(obj, value)

        self.assertFalse(obj.is_reserved())
        self.assertTrue(RenderLabel.kEmpty.is_reserved())
        self.assertTrue(RenderLabel.kDoNotRender.is_reserved())
        self.assertTrue(RenderLabel.kDontCare.is_reserved())
        self.assertTrue(RenderLabel.kUnspecified.is_reserved())
        self.assertEqual(RenderLabel(value), RenderLabel(value))
        self.assertNotEqual(RenderLabel(value), RenderLabel.kEmpty)

    @numpy_compare.check_nonsymbolic_types
    def test_query_object(self, T):
        RigidTransform = RigidTransform_[float]
        SceneGraph = mut.SceneGraph_[T]
        QueryObject = mut.QueryObject_[T]
        SceneGraphInspector = mut.SceneGraphInspector_[T]

        scene_graph = SceneGraph()
        render_params = mut.render.RenderEngineVtkParams()
        renderer_name = "test_renderer"
        scene_graph.AddRenderer(renderer_name,
                                mut.render.MakeRenderEngineVtk(render_params))

        context = scene_graph.CreateDefaultContext()
        query_object = scene_graph.get_query_output_port().Eval(context)

        self.assertIsInstance(query_object.inspector(), SceneGraphInspector)

        # Proximity queries -- all of these will produce empty results.
        results = query_object.ComputeSignedDistancePairwiseClosestPoints()
        self.assertEqual(len(results), 0)
        results = query_object.ComputePointPairPenetration()
        self.assertEqual(len(results), 0)
        results = query_object.ComputeSignedDistanceToPoint(p_WQ=(1, 2, 3))
        self.assertEqual(len(results), 0)
        results = query_object.FindCollisionCandidates()
        self.assertEqual(len(results), 0)

        # Confirm rendering API returns images of appropriate type.
        d_camera = mut.render.DepthCameraProperties(
            width=320, height=240, fov_y=pi/6, renderer_name=renderer_name,
            z_near=0.1, z_far=5.0)
        image = query_object.RenderColorImage(
            camera=d_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageRgba8U)
        image = query_object.RenderDepthImage(
            camera=d_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageDepth32F)
        image = query_object.RenderLabelImage(
            camera=d_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageLabel16I)
