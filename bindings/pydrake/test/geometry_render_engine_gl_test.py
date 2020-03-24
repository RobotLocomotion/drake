import pydrake.geometry as mut

import sys
import unittest

from pydrake.common.test_utilities import numpy_compare


@numpy_compare.check_nonsymbolic_types
class TestGeometry(unittest.TestCase):

    def test_render_engine_gl_api(self, T):
        SceneGraph = mut.SceneGraph_[T]
        scene_graph = SceneGraph()
        if 'darwin' in sys.platform:  # OpenGL is not supported on macOS.
            self.assertRaises(RuntimeError, mut.render.MakeRenderEngineGl)
        else:
            scene_graph.AddRenderer("gl_renderer",
                                    mut.render.MakeRenderEngineGl())
            self.assertTrue(scene_graph.HasRenderer("gl_renderer"))
            self.assertEqual(scene_graph.RendererCount(), 1)
