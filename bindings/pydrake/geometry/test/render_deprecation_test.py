from pydrake.common.test_utilities.deprecation import catch_drake_warnings

import unittest


class TestDeprecation(unittest.TestCase):
    """
    2023-08-01 Remove this file upon completion of deprecation.
    """
    def test_deprecated_symbols_exist(self):
        # The whole point of this test is that we can access all the symbols in
        # the deprecated module path.
        with catch_drake_warnings(expected_count=1) as w:
            from pydrake.geometry.render import (
                ClippingRange,
                ColorRenderCamera,
                DepthRange,
                DepthRenderCamera,
                MakeRenderEngineGl,
                MakeRenderEngineGltfClient,
                MakeRenderEngineVtk,
                RenderCameraCore,
                RenderEngine,
                RenderEngineGlParams,
                RenderEngineGltfClientParams,
                RenderEngineVtkParams,
                RenderLabel,
            )
            self.assertIn("2023-08-01", str(w[0].message))
