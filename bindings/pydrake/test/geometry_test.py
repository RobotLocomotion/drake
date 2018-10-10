import pydrake.geometry as mut

import unittest

from pydrake.lcm import DrakeMockLcm
from pydrake.systems.framework import DiagramBuilder, InputPort, OutputPort


class TestGeometry(unittest.TestCase):
    def test_scene_graph(self):
        scene_graph = mut.SceneGraph()
        with self.assertRaises(SystemExit) as exc:
            scene_graph.get_source_pose_port(mut.SourceId())
            self.assertIn("unknown source", str(exc.exception))
        self.assertIsInstance(
            scene_graph.get_pose_bundle_output_port(), OutputPort)
        self.assertIsInstance(
            scene_graph.get_query_output_port(), OutputPort)

        # Test visualization API.
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

    def _check_identifier(self, cls):
        obj = cls()
        self.assertFalse(obj.is_valid())
        a = cls.get_new_id()
        b = cls.get_new_id()
        self.assertTrue(a == a)
        self.assertFalse(a == b)
        self.assertTrue(a < b)

    def test_identifiers(self):
        cls_list = [
            mut.SourceId,
            mut.FrameId,
            mut.GeometryId,
        ]
        for cls in cls_list:
            self._check_identifier(cls)

    def test_penetration_as_point_pair(self):
        obj = mut.PenetrationAsPointPair()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_WCa.shape, (3,))
        self.assertTupleEqual(obj.p_WCb.shape, (3,))
        self.assertEqual(obj.depth, -1.)
