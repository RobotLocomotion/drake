"""For reasons not yet understood (see issue #14686), the unit tests where we
confirm the semantics of deriving a RenderEngine implementation in python are
flaky and are therefore being isolated (see issue #14720)."""

import pydrake.geometry as mut  # ruff: isort: skip

import gc
from math import pi
import unittest
import weakref

from pydrake.math import RigidTransform
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import (
    CameraInfo,
    ImageDepth32F,
    ImageLabel16I,
    ImageRgba8U,
)


class TestRenderEngineSubclass(unittest.TestCase):
    def test_unimplemented_rendering(self):
        """The RenderEngine API throws exceptions for derived implementations
        that don't override DoRender*Image. This test confirms that behavior
        propagates down to Python."""

        class MinimalEngine(mut.RenderEngine):
            """Minimal implementation of the RenderEngine virtual API"""

            def UpdateViewpoint(self, X_WC):
                pass

            def DoRegisterVisual(self, id, shape, properties, X_WG):
                pass

            def DoUpdateVisualPose(self, id, X_WG):
                pass

            def DoRemoveGeometry(self, id):
                pass

            def __deepcopy__(self, memo):
                return type(self)()

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

        identity = RigidTransform()
        intrinsics = CameraInfo(10, 10, pi / 4)
        core = mut.RenderCameraCore(
            "n/a", intrinsics, mut.ClippingRange(0.1, 10), identity
        )
        color_cam = mut.ColorRenderCamera(core, False)
        depth_cam = mut.DepthRenderCamera(core, mut.DepthRange(0.1, 9))
        color_image = ImageRgba8U(intrinsics.width(), intrinsics.height())
        depth_image = ImageDepth32F(intrinsics.width(), intrinsics.height())
        label_image = ImageLabel16I(intrinsics.width(), intrinsics.height())

        color_only = ColorOnlyEngine()
        color_only.RenderColorImage(color_cam, color_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            color_only.RenderDepthImage(depth_cam, depth_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            color_only.RenderLabelImage(color_cam, label_image)
        self.assertIsInstance(color_only.Clone(), ColorOnlyEngine)

        depth_only = DepthOnlyEngine()
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            depth_only.RenderColorImage(color_cam, color_image)
        depth_only.RenderDepthImage(depth_cam, depth_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            depth_only.RenderLabelImage(color_cam, label_image)
        self.assertIsInstance(depth_only.Clone(), DepthOnlyEngine)

        label_only = LabelOnlyEngine()
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            label_only.RenderColorImage(color_cam, color_image)
        with self.assertRaisesRegex(RuntimeError, ".+pure virtual function.+"):
            label_only.RenderDepthImage(depth_cam, depth_image)
        label_only.RenderLabelImage(color_cam, label_image)
        self.assertIsInstance(label_only.Clone(), LabelOnlyEngine)

    def test_legacy_DoClone(self):
        """Sanity checks that DoClone (without __deepcopy__) is sufficient."""

        class CloneableEngine(mut.RenderEngine):
            def DoClone(self):
                return CloneableEngine()

        dut = CloneableEngine()
        clone = dut.Clone()
        self.assertIsInstance(clone, CloneableEngine)
        self.assertIsNot(clone, dut)

    def test_lifecycle(self):
        """Tests lifecycle, keep_alive, ownership, etc."""

        num_engines = 0
        num_renders = 0

        class MyEngine(mut.RenderEngine):
            def __init__(self):
                super().__init__()
                nonlocal num_engines
                num_engines = num_engines + 1

            def __del__(self):
                nonlocal num_engines
                num_engines = num_engines - 1

            def UpdateViewpoint(self, X_WC):
                pass

            def DoRenderColorImage(self, camera, image_out):
                nonlocal num_renders
                num_renders = num_renders + 1

            def __deepcopy__(self, memo):
                return type(self)()

        # Wrap a SceneGraph in a Diagram.
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(mut.SceneGraph())
        builder.ExportOutput(scene_graph.get_query_output_port(), name="query")
        diagram = builder.Build()
        del builder
        gc.collect()
        world_frame = scene_graph.world_frame_id()

        # Add a render engine. It will be *deep copied* into scene_graph, so
        # the original engine will be GC'd.
        self.assertEqual(num_engines, 0)
        engine = MyEngine()
        spy = weakref.finalize(engine, lambda: None)
        scene_graph.AddRenderer("name", engine)
        del engine
        gc.collect()
        self.assertFalse(spy.alive)
        self.assertEqual(num_engines, 1)
        del scene_graph
        del spy
        gc.collect()

        # Check that the cloned MyEngine instance can still be called.
        diagram_context = diagram.CreateDefaultContext()
        self.assertGreater(num_engines, 1)
        query = diagram.GetOutputPort("query").Eval(diagram_context)
        self.assertEqual(num_renders, 0)
        query.RenderColorImage(
            camera=mut.ColorRenderCamera(
                mut.RenderCameraCore(
                    renderer_name="name",
                    intrinsics=CameraInfo(640, 480, 0.5),
                    clipping=mut.ClippingRange(0.1, 10.0),
                    X_BS=RigidTransform(),
                ),
            ),
            parent_frame=world_frame,
            X_PC=RigidTransform(),
        )
        self.assertEqual(num_renders, 1)

        # Release everything and ensure we get back to zero engines.
        del diagram
        del diagram_context
        del query
        gc.collect()
        self.assertEqual(num_engines, 0)
