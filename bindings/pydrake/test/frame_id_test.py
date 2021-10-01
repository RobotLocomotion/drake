import pydrake.geometry as mut

import unittest


class TestFrameId(unittest.TestCase):
    """Checks that FrameId values are globally unique.  This test case must
    be isolated as its own test program (not mixed with the other FrameId
    unit tests) because it relies on global state (the ID counter).
    """

    def test_id_uniqueness(self):
        """Every FrameId must be unique."""
        scene_graph = mut.SceneGraph()
        f = mut.FrameId.get_new_id().get_value()
        g = mut.GeometryFrame("geometry").id().get_value()
        w = scene_graph.world_frame_id().get_value()
        self.assertNotEqual(f, g)
        self.assertNotEqual(w, f)
        self.assertNotEqual(w, g)
