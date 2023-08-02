import pydrake.geometry as mut

import sys
import unittest

from pydrake.common.test_utilities import numpy_compare


class TestGeometry(unittest.TestCase):

    @numpy_compare.check_nonsymbolic_types
    def test_render_engine_gl_api(self, T):
        SceneGraph = mut.SceneGraph_[T]
        scene_graph = SceneGraph()
        params = mut.RenderEngineGlParams()
        if 'darwin' in sys.platform:  # OpenGL is not supported on macOS.
            self.assertRaises(RuntimeError, mut.MakeRenderEngineGl)
        else:
            scene_graph.AddRenderer("gl_renderer",
                                    mut.MakeRenderEngineGl(params=params))
            self.assertTrue(scene_graph.HasRenderer("gl_renderer"))
            self.assertEqual(scene_graph.RendererCount(), 1)
