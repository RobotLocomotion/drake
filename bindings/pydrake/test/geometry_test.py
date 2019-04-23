import pydrake.geometry as mut

import unittest
import warnings

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.lcm import DrakeMockLcm
from pydrake.systems.framework import DiagramBuilder, InputPort, OutputPort
from pydrake.common.deprecation import DrakeDeprecationWarning


class TestGeometry(unittest.TestCase):
    def setUp(self):
        warnings.simplefilter('error', DrakeDeprecationWarning)

    def test_scene_graph_api(self):
        scene_graph = mut.SceneGraph()
        global_source = scene_graph.RegisterSource("anchored")
        self.assertIsInstance(
            scene_graph.get_source_pose_port(global_source), InputPort)
        self.assertIsInstance(
            scene_graph.get_pose_bundle_output_port(), OutputPort)
        self.assertIsInstance(
            scene_graph.get_query_output_port(), OutputPort)
        # Test visualization API.
        # Use a mockable so that we can make a smoke test without side effects.
        lcm = DrakeMockLcm()
        for i in range(2):
            builder = DiagramBuilder()
            scene_graph = builder.AddSystem(mut.SceneGraph())
            if i == 1:
                mut.ConnectDrakeVisualizer(
                    builder=builder, scene_graph=scene_graph)
            else:
                mut.ConnectDrakeVisualizer(
                    builder=builder, scene_graph=scene_graph,
                    pose_bundle_output_port=(
                        scene_graph.get_pose_bundle_output_port()))
            mut.DispatchLoadMessage(
                scene_graph=scene_graph, lcm=lcm)

    def test_frame_pose_vector_api(self):
        mut.FramePoseVector()
        with catch_drake_warnings(expected_count=1):
            mut.FramePoseVector(source_id=mut.SourceId.get_new_id(),
                                ids=[mut.FrameId.get_new_id()])

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

    def test_penetration_as_point_pair_api(self):
        obj = mut.PenetrationAsPointPair()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_WCa.shape, (3,))
        self.assertTupleEqual(obj.p_WCb.shape, (3,))
        self.assertEqual(obj.depth, -1.)

    def test_signed_distance_api(self):
        obj = mut.SignedDistancePair()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_ACa.shape, (3,))
        self.assertTupleEqual(obj.p_BCb.shape, (3,))
        self.assertIsInstance(obj.distance, float)
        self.assertTupleEqual(obj.nhat_BA_W.shape, (3,))

    def test_signed_distance_to_point_api(self):
        obj = mut.SignedDistanceToPoint()
        self.assertIsInstance(obj.id_G, mut.GeometryId)
        self.assertTupleEqual(obj.p_GN.shape, (3,))
        self.assertIsInstance(obj.distance, float)
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
