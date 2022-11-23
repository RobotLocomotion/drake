import pydrake.geometry as mut

import sys
import unittest


class TestGeometry(unittest.TestCase):

    def test_render_engine_gltf_client_api(self):
        SceneGraph = mut.SceneGraph_[float]
        scene_graph = SceneGraph()
        params = mut.render.RenderEngineGltfClientParams()
        if 'darwin' in sys.platform:  # Not supported on macOS.
            self.assertRaises(RuntimeError,
                              mut.render.MakeRenderEngineGltfClient)
        else:
            scene_graph.AddRenderer("gltf_renderer",
                                    mut.render.MakeRenderEngineGltfClient(
                                        params=params))
            self.assertTrue(scene_graph.HasRenderer("gltf_renderer"))
            self.assertEqual(scene_graph.RendererCount(), 1)
