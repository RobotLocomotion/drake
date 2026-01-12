import pydrake.geometry as mut  # ruff: isort: skip

import copy
import logging
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.value import Value
from pydrake.math import RigidTransform
from pydrake.systems.framework import (
    DiagramBuilder,
)
from pydrake.systems.sensors import (
    CameraInfo,
    ImageDepth16U,
    ImageDepth32F,
    ImageLabel16I,
    ImageRgba8U,
    RgbdSensor,
)


class TestGeometryRender(unittest.TestCase):
    def test_light_param(self):
        # A default constructor exists.
        mut.LightParameter()

        # The kwarg constructor also works.
        light = mut.LightParameter(
            type="spot",
            color=mut.Rgba(0.1, 0.2, 0.3),
            attenuation_values=(1, 2, 3),
            position=(-1, -2, -3),
            frame="camera",
            intensity=0.5,
            direction=(0, 1, 0),
            cone_angle=85,
        )
        # Attributes are bound explicitly, so we'll test them explicitly.
        self.assertEqual(light.type, "spot")
        self.assertEqual(light.color, mut.Rgba(0.1, 0.2, 0.3))
        self.assertTupleEqual(tuple(light.attenuation_values), (1, 2, 3))
        self.assertTupleEqual(tuple(light.position), (-1, -2, -3))
        self.assertEqual(light.frame, "camera")
        self.assertEqual(light.intensity, 0.5)
        self.assertTupleEqual(tuple(light.direction), (0, 1, 0))
        self.assertEqual(light.cone_angle, 85)

        self.assertIn("spot", repr(light))
        copy.copy(light)

    def test_ssao_param(self):
        # A default constructor exists.
        mut.SsaoParameter()

        # The kwarg constructor also works.
        params = mut.SsaoParameter(
            radius=0.2,
            bias=0.01,
            sample_count=8,
            intensity_scale=1.5,
            intensity_shift=0.05,
            blur=False,
        )
        self.assertEqual(params.radius, 0.2)
        self.assertEqual(params.bias, 0.01)
        self.assertEqual(params.sample_count, 8)
        self.assertEqual(params.intensity_scale, 1.5)
        self.assertEqual(params.intensity_shift, 0.05)
        self.assertFalse(params.blur)

        self.assertIn("bias", repr(params))
        copy.copy(params)

    def test_equirectangular_map(self):
        # A default constructor exists.
        mut.EquirectangularMap()

        # The kwarg constructor also works.
        map = mut.EquirectangularMap(path="test.hdr")
        self.assertEqual(map.path, "test.hdr")

        self.assertIn("path", repr(map))
        copy.copy(map)

    def test_environment_map(self):
        # A default constructor exists.
        mut.EnvironmentMap()

        # The kwarg constructor also works.
        params = mut.EnvironmentMap(skybox=False)
        self.assertFalse(params.skybox)
        self.assertIsInstance(params.texture, mut.NullTexture)

        params = mut.EnvironmentMap(
            texture=mut.EquirectangularMap(path="test.hdr")
        )
        self.assertIn("EquirectangularMap", repr(params))
        copy.copy(params)

    def test_gltf_extension(self):
        # A default constructor exists.
        mut.GltfExtension()

        # The kwarg constructor also works.
        params = mut.GltfExtension(warn_unimplemented=False)
        self.assertFalse(params.warn_unimplemented)

        # The repr and copy both work.
        self.assertIn("warn_unimplemented=", repr(params))
        copy.copy(params)

    def test_render_engine_vtk_api(self):
        self.assertTrue(mut.kHasRenderEngineVtk)

        scene_graph = mut.SceneGraph()
        params = mut.RenderEngineVtkParams()
        scene_graph.AddRenderer(
            "vtk_renderer", mut.MakeRenderEngineVtk(params=params)
        )
        self.assertTrue(scene_graph.HasRenderer("vtk_renderer"))
        self.assertEqual(scene_graph.RendererCount(), 1)

    def test_render_engine_vtk_params(self):
        # Confirm default construction of params.
        params = mut.RenderEngineVtkParams()
        self.assertEqual(params.default_diffuse, None)

        diffuse = np.array((1.0, 0.0, 0.0, 0.0))
        params = mut.RenderEngineVtkParams(
            default_diffuse=diffuse,
            environment_map=mut.EnvironmentMap(
                skybox=False, texture=mut.EquirectangularMap(path="local.hdr")
            ),
        )
        self.assertTrue((params.default_diffuse == diffuse).all())

        self.assertIn("default_diffuse", repr(params))
        copy.copy(params)

    def test_render_vtk_gltf_warnings(self):
        """The fully_textured_pyramid.gltf offers the basisu extension but our
        RenderEngineVtk doesn't implement that. The "not implemented" warning
        is suppressed by default, but if we reset the suppressions it should
        show up.
        """
        for should_warn in [False, True]:
            with self.subTest(should_warn=should_warn):
                self._do_test_render_vtk_gltf_warnings(should_warn=should_warn)

    def _do_test_render_vtk_gltf_warnings(self, *, should_warn):
        # Create the render engine.
        expected_level = logging.WARNING if should_warn else logging.DEBUG
        params = mut.RenderEngineVtkParams()
        if should_warn:
            # All unimplemented extensions will warn.
            params.gltf_extensions = dict()
        renderer = mut.MakeRenderEngineVtk(params=params)

        # Prepare the mesh to be loaded.
        material = mut.PerceptionProperties()
        material.AddProperty("label", "id", mut.RenderLabel(1))
        geom_id = mut.GeometryId.get_new_id()
        filename = FindResourceOrThrow(
            "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf"
        )

        # Load the mesh, which should emit exactly one log message.
        with self.assertLogs("drake", logging.DEBUG) as cm:
            renderer.RegisterVisual(
                id=geom_id,
                shape=mut.Mesh(filename),
                properties=material,
                X_WG=RigidTransform.Identity(),
                needs_updates=False,
            )
        self.assertEqual(len(cm.records), 1, cm)
        record = cm.records[0]

        # The message mentioned basisu using the proper severity level.
        self.assertIn("KHR_texture_basisu", record.msg)
        self.assertEqual(record.levelno, expected_level)

    def test_render_engine_gl_params(self):
        # A default constructor exists.
        mut.RenderEngineGlParams()

        # The kwarg constructor also works.
        diffuse = mut.Rgba(1.0, 0.0, 0.0, 0.0)
        params = mut.RenderEngineGlParams(
            default_clear_color=diffuse,
            default_diffuse=diffuse,
        )
        self.assertEqual(params.default_clear_color, diffuse)
        self.assertEqual(params.default_diffuse, diffuse)

        self.assertIn("default_clear_color", repr(params))
        copy.copy(params)

    def test_render_engine_gltf_client_params(self):
        # A default constructor exists.
        mut.RenderEngineGltfClientParams()

        # The kwarg constructor also works.
        base_url = "http://127.0.0.1:8888"
        render_endpoint = "render"
        params = mut.RenderEngineGltfClientParams(
            base_url=base_url,
            render_endpoint=render_endpoint,
        )
        self.assertEqual(params.render_endpoint, render_endpoint)
        self.assertEqual(params.base_url, base_url)

        self.assertIn("render_endpoint", repr(params))
        copy.copy(params)

    def test_render_label(self):
        RenderLabel = mut.RenderLabel
        value = 10
        obj = RenderLabel(value)

        self.assertIs(value, int(obj))
        self.assertEqual(value, obj)
        self.assertEqual(obj, value)

        self.assertFalse(obj.is_reserved())
        self.assertTrue(RenderLabel.kEmpty.is_reserved())
        self.assertTrue(RenderLabel.kDoNotRender.is_reserved())
        self.assertTrue(RenderLabel.kDontCare.is_reserved())
        self.assertTrue(RenderLabel.kUnspecified.is_reserved())
        self.assertEqual(RenderLabel(value), RenderLabel(value))
        self.assertNotEqual(RenderLabel(value), RenderLabel.kEmpty)

        # Confirm value instantiation.
        Value[mut.RenderLabel]

    def test_render_label_repr(self):
        RenderLabel = mut.RenderLabel

        # Special labels should use a non-numeric spelling.
        special_labels = [
            RenderLabel.kEmpty,
            RenderLabel.kDoNotRender,
            RenderLabel.kDontCare,
            RenderLabel.kUnspecified,
        ]
        for label in special_labels:
            self.assertIn("RenderLabel.k", repr(label))

        # Any label should round-trip via 'eval'.
        all_labels = special_labels + [RenderLabel(10)]
        for label in all_labels:
            roundtrip = eval(repr(label), dict(RenderLabel=RenderLabel))
            self.assertEqual(label, roundtrip)

    def test_render_engine_api(self):
        class DummyRenderEngine(mut.RenderEngine):
            """Mirror of C++ DummyRenderEngine."""

            # See comment below about `rgbd_sensor_test.cc`.
            latest_instance = None

            def __init__(self, render_label=None):
                mut.RenderEngine.__init__(self)
                # N.B. We do not hide these because this is a test class.
                # Normally, you would want to hide this.
                self.force_accept = False
                self.registered_geometries = set()
                self.updated_ids = {}
                self.include_group_name = "in_test"
                self.X_WC = RigidTransform()
                self.color_count = 0
                self.depth_count = 0
                self.label_count = 0
                self.color_camera = None
                self.depth_camera = None
                self.label_camera = None

            def UpdateViewpoint(self, X_WC):
                DummyRenderEngine.latest_instance = self
                self.X_WC = X_WC

            def ImplementGeometry(self, shape, user_data):
                DummyRenderEngine.latest_instance = self

            def DoRegisterVisual(self, id, shape, properties, X_WG):
                DummyRenderEngine.latest_instance = self
                mut.GetRenderLabelOrThrow(properties)
                if self.force_accept or properties.HasGroup(
                    self.include_group_name
                ):
                    self.registered_geometries.add(id)
                    return True
                return False

            def DoUpdateVisualPose(self, id, X_WG):
                DummyRenderEngine.latest_instance = self
                self.updated_ids[id] = X_WG

            def DoRemoveGeometry(self, id):
                DummyRenderEngine.latest_instance = self
                self.registered_geometries.remove(id)

            def __deepcopy__(self, memo):
                DummyRenderEngine.latest_instance = self
                new = DummyRenderEngine()
                new.force_accept = copy.deepcopy(self.force_accept, memo=memo)
                new.registered_geometries = copy.deepcopy(
                    self.registered_geometries, memo=memo
                )
                new.updated_ids = copy.deepcopy(self.updated_ids, memo=memo)
                new.include_group_name = copy.deepcopy(
                    self.include_group_name, memo=memo
                )
                new.X_WC = copy.deepcopy(self.X_WC, memo=memo)
                new.color_count = copy.deepcopy(self.color_count, memo=memo)
                new.depth_count = copy.deepcopy(self.depth_count, memo=memo)
                new.label_count = copy.deepcopy(self.label_count, memo=memo)
                new.color_camera = copy.deepcopy(self.color_camera, memo=memo)
                new.depth_camera = copy.deepcopy(self.depth_camera, memo=memo)
                new.label_camera = copy.deepcopy(self.label_camera, memo=memo)
                return new

            def DoRenderColorImage(self, camera, color_image_out):
                DummyRenderEngine.latest_instance = self
                self.color_count += 1
                self.color_camera = camera

            def DoRenderDepthImage(self, camera, depth_image_out):
                DummyRenderEngine.latest_instance = self
                self.depth_count += 1
                self.depth_camera = camera

            def DoRenderLabelImage(self, camera, label_image_out):
                DummyRenderEngine.latest_instance = self
                self.label_count += 1
                self.label_camera = camera

            def DoGetParameterYaml(self):
                return "PyDummyRenderEngine: {}"

        engine = DummyRenderEngine()
        self.assertIsInstance(engine, mut.RenderEngine)
        self.assertIsInstance(engine.Clone(), DummyRenderEngine)

        # Test implementation of C++ interface by using RgbdSensor.
        renderer_name = "renderer"
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(mut.SceneGraph())
        scene_graph.AddRenderer(renderer_name, engine)
        sensor = builder.AddSystem(
            RgbdSensor(
                parent_id=scene_graph.world_frame_id(),
                X_PB=RigidTransform(),
                depth_camera=mut.DepthRenderCamera(
                    mut.RenderCameraCore(
                        renderer_name,
                        CameraInfo(640, 480, np.pi / 4),
                        mut.ClippingRange(0.1, 5.0),
                        RigidTransform(),
                    ),
                    mut.DepthRange(0.1, 5.0),
                ),
            )
        )
        builder.Connect(
            scene_graph.get_query_output_port(),
            sensor.query_object_input_port(),
        )
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        sensor_context = sensor.GetMyContextFromRoot(diagram_context)
        image = sensor.color_image_output_port().Eval(sensor_context)
        # N.B. Because there's context cloning going on under the hood, we
        # won't be interacting with our originally registered instance.
        # See `rgbd_sensor_test.cc` as well.
        current_engine = DummyRenderEngine.latest_instance
        self.assertIsNot(current_engine, engine)
        self.assertIsInstance(image, ImageRgba8U)
        self.assertEqual(current_engine.color_count, 1)

        image = sensor.depth_image_32F_output_port().Eval(sensor_context)
        self.assertIsInstance(image, ImageDepth32F)
        self.assertEqual(current_engine.depth_count, 1)

        image = sensor.depth_image_16U_output_port().Eval(sensor_context)
        self.assertIsInstance(image, ImageDepth16U)
        self.assertEqual(current_engine.depth_count, 2)

        image = sensor.label_image_output_port().Eval(sensor_context)
        self.assertIsInstance(image, ImageLabel16I)
        self.assertEqual(current_engine.label_count, 1)

        param_yaml = engine.GetParameterYaml()
        self.assertEqual(param_yaml, "PyDummyRenderEngine: {}")

        # TODO(eric, duy): Test more properties.

    def test_render_engine_gltf_client_api(self):
        self.assertTrue(mut.kHasRenderEngineGltfClient)

        scene_graph = mut.SceneGraph()
        params = mut.RenderEngineGltfClientParams()
        scene_graph.AddRenderer(
            "gltf_renderer", mut.MakeRenderEngineGltfClient(params=params)
        )
        self.assertTrue(scene_graph.HasRenderer("gltf_renderer"))
        self.assertEqual(scene_graph.RendererCount(), 1)
