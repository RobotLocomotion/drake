import pydrake.geometry as mut

import unittest


class TestFoo(unittest.TestCase):
    def test_one(self):
        """
        Every acquired FrameId must be unique. If this test executes first, it
        will fail. scene_graph's world_frame_id() will match the id of the
        newly added geometry frame. If, however, test_two executes first, this
        test will pass.

        This is because both SceneGraph and GeometryFrame call
        FrameId::get_new_id(), but they are somehow accessing *different*
        FrameId classes -- they are drawing from different static sources.
        """
        scene_graph = mut.SceneGraph()
        f = mut.GeometryFrame("frame")
        f_id = mut.FrameId.get_new_id()
        self.assertNotEqual(scene_graph.world_frame_id().get_value(),
                            f.id().get_value())
        self.assertNotEqual(scene_graph.world_frame_id().get_value(), f_id)
        self.assertNotEqual(f.id().get_value(), f_id)

    def test_two(self):
        """
        Same exact test as test_one, fails if executed first, passes if
        executed second.
        """
        scene_graph = mut.SceneGraph()
        f = mut.GeometryFrame("frame")
        f_id = mut.FrameId.get_new_id()
        self.assertNotEqual(scene_graph.world_frame_id().get_value(),
                            f.id().get_value())
        self.assertNotEqual(scene_graph.world_frame_id().get_value(), f_id)
        self.assertNotEqual(f.id().get_value(), f_id)

    def test_three(self):
        """
        This test also fails under the conditions and the reasons outlined
        above.
        """
        scene_graph = mut.SceneGraph_[float]()
        f = mut.GeometryFrame("frame")
        f_id = mut.FrameId.get_new_id()
        self.assertNotEqual(scene_graph.world_frame_id().get_value(),
                            f.id().get_value())
        self.assertNotEqual(scene_graph.world_frame_id().get_value(), f_id)
        self.assertNotEqual(f.id().get_value(), f_id)
