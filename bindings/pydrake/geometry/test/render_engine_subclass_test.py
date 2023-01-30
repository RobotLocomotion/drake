"""For reasons not yet understood (see issue #14686), the unit tests where we
confirm the semantics of deriving a RenderEngine implementation in python are
flaky and are therefore being isolated (see issue #14720)."""

import pydrake.geometry as mut

from math import pi
import unittest

from pydrake.math import RigidTransform, RigidTransform_
from pydrake.systems.sensors import (
    CameraInfo,
    ImageRgba8U,
    ImageDepth32F,
    ImageLabel16I,
)


class TestRenderEngineSubclass(unittest.TestCase):

    def test_unimplemented_rendering(self):
        """The RenderEngine API throws exceptions for derived implementations
        that don't override DoRender*Image. This test confirms that behavior
        propagates down to Python."""
        class MinimalEngine(mut.render.RenderEngine):
            """Minimal implementation of the RenderEngine virtual API"""
            def UpdateViewpoint(self, X_WC):
                pass

            def DoRegisterVisual(self, id, shape, properties, X_WG):
                pass

            def DoUpdateVisualPose(self, id, X_WG):
                pass

            def DoRemoveGeometry(self, id):
                pass

            def DoClone(self):
                pass

        class ColorOnlyEngine(MinimalEngine):
            """Rendering Depth and Label images should throw"""
            def DoRenderColorImage(self, camera, image_out):
                pass

        class DepthOnlyEngine(MinimalEngine):
            """Rendering Color and Label images should throw"""
            def DoRenderDepthImage(self, camera, image_out):
                pass

        class LabelOnlyEngine(MinimalEngine):
            """Rendering Color and Depth images should throw"""
            def DoRenderLabelImage(self, camera, image_out):
                pass

        identity = RigidTransform_[float]()
        intrinsics = CameraInfo(10, 10, pi / 4)
        core = mut.render.RenderCameraCore("n/a", intrinsics,
                                           mut.render.ClippingRange(0.1, 10),
                                           identity)
        color_cam = mut.render.ColorRenderCamera(core, False)
        depth_cam = mut.render.DepthRenderCamera(
                        core, mut.render.DepthRange(0.1, 9))
        color_image = ImageRgba8U(intrinsics.width(), intrinsics.height())
        depth_image = ImageDepth32F(intrinsics.width(), intrinsics.height())
        label_image = ImageLabel16I(intrinsics.width(), intrinsics.height())

        color_only = ColorOnlyEngine()
        color_only.RenderColorImage(color_cam, color_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            color_only.RenderDepthImage(depth_cam, depth_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            color_only.RenderLabelImage(color_cam, label_image)

        depth_only = DepthOnlyEngine()
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            depth_only.RenderColorImage(color_cam, color_image)
        depth_only.RenderDepthImage(depth_cam, depth_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            depth_only.RenderLabelImage(color_cam, label_image)

        label_only = LabelOnlyEngine()
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            label_only.RenderColorImage(color_cam, color_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            label_only.RenderDepthImage(depth_cam, depth_image)
        label_only.RenderLabelImage(color_cam, label_image)
