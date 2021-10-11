import pydrake.geometry as mut

import copy
import unittest
from math import pi

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import Value
from pydrake.math import RigidTransform, RigidTransform_
from pydrake.systems.analysis import (
    Simulator_,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    InputPort_,
    OutputPort_,
)
from pydrake.systems.sensors import (
    CameraInfo,
    ImageRgba8U,
    ImageDepth16U,
    ImageDepth32F,
    ImageLabel16I,
    RgbdSensor,
)

PROPERTY_CLS_LIST = [
    mut.ProximityProperties,
    mut.IllustrationProperties,
    mut.PerceptionProperties,
]


class TestGeometry(unittest.TestCase):
    @numpy_compare.check_nonsymbolic_types
    def test_scene_graph_api(self, T):
        SceneGraph = mut.SceneGraph_[T]
        InputPort = InputPort_[T]
        OutputPort = OutputPort_[T]

        scene_graph = SceneGraph()
        global_source = scene_graph.RegisterSource("anchored")
        global_frame = scene_graph.RegisterFrame(
            source_id=global_source, frame=mut.GeometryFrame("anchored_frame"))
        scene_graph.RegisterFrame(
            source_id=global_source, parent_id=global_frame,
            frame=mut.GeometryFrame("anchored_frame"))
        global_geometry = scene_graph.RegisterGeometry(
            source_id=global_source, frame_id=global_frame,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere1"))
        # We'll explicitly give sphere_2 a rigid hydroelastic representation.
        sphere_2 = scene_graph.RegisterGeometry(
            source_id=global_source, geometry_id=global_geometry,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere2"))
        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(resolution_hint=1, properties=props)
        scene_graph.AssignRole(source_id=global_source, geometry_id=sphere_2,
                               properties=props)
        # We'll explicitly give sphere_3 a soft hydroelastic representation.
        sphere_3 = scene_graph.RegisterAnchoredGeometry(
            source_id=global_source,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere3"))
        props = mut.ProximityProperties()
        mut.AddContactMaterial(elastic_modulus=1e8, properties=props)
        mut.AddSoftHydroelasticProperties(resolution_hint=1, properties=props)
        scene_graph.AssignRole(source_id=global_source, geometry_id=sphere_3,
                               properties=props)

        self.assertIsInstance(
            scene_graph.get_source_pose_port(global_source), InputPort)

        with catch_drake_warnings(expected_count=1):
            self.assertIsInstance(
                scene_graph.get_pose_bundle_output_port(), OutputPort)
        self.assertIsInstance(
            scene_graph.get_query_output_port(), OutputPort)

        # Test limited rendering API.
        scene_graph.AddRenderer("test_renderer",
                                mut.render.MakeRenderEngineVtk(
                                    mut.render.RenderEngineVtkParams()))
        self.assertTrue(scene_graph.HasRenderer("test_renderer"))
        self.assertEqual(scene_graph.RendererCount(), 1)

        # Test SceneGraphInspector API
        inspector = scene_graph.model_inspector()
        self.assertEqual(inspector.num_sources(), 2)
        self.assertEqual(inspector.num_frames(), 3)
        with catch_drake_warnings(expected_count=3):
            self.assertEqual(len(inspector.all_frame_ids()), 3)
            self.assertTrue(inspector.world_frame_id()
                            in inspector.all_frame_ids())
            self.assertTrue(global_frame in inspector.all_frame_ids())
        self.assertEqual(len(inspector.GetAllFrameIds()), 3)
        self.assertTrue(inspector.world_frame_id()
                        in inspector.GetAllFrameIds())
        self.assertTrue(global_frame in inspector.GetAllFrameIds())
        self.assertIsInstance(inspector.world_frame_id(), mut.FrameId)
        self.assertEqual(inspector.num_geometries(), 3)
        self.assertEqual(len(inspector.GetAllGeometryIds()), 3)

        # Test both GeometrySet API as well as SceneGraphInspector's
        # GeometrySet API.
        empty_set = mut.GeometrySet()
        self.assertEqual(
            len(inspector.GetGeometryIds(empty_set)),
            0)
        self.assertEqual(
            len(inspector.GetGeometryIds(empty_set, mut.Role.kProximity)),
            0)
        # Cases 1.a: Explicit frame, constructor
        # N.B. Only in this case (1.a), do we test for non-kwarg usages of
        # functions. In other tests,
        frame_set_options = [
            # Frame scalar.
            mut.GeometrySet(frame_id=global_frame),
            # Frame list.
            mut.GeometrySet(frame_ids=[global_frame]),
            # Frame list, no kwargs.
            mut.GeometrySet([global_frame]),
            # Frame list w/ (empty) geometry list.
            mut.GeometrySet(geometry_ids=[], frame_ids=[global_frame]),
            # Frame list w/ (empty) geometry list, no kwargs.
            mut.GeometrySet([], [global_frame]),
        ]
        # Case 1.b: Explicit frame, via Add().
        # - Frame scalar.
        cur = mut.GeometrySet()
        cur.Add(frame_id=global_frame)
        frame_set_options.append(cur)
        # - Frame list.
        cur = mut.GeometrySet()
        cur.Add(frame_ids=[global_frame])
        frame_set_options.append(cur)
        # - Frame list w/ (empty) geometry list.
        cur = mut.GeometrySet()
        cur.Add(geometry_ids=[], frame_ids=[global_frame])
        frame_set_options.append(cur)
        # Cases 1.*: Test 'em all.
        for frame_set in frame_set_options:
            ids = inspector.GetGeometryIds(frame_set)
            # N.B. Per above, we have 2 geometries that have been affixed to
            # global frame ("sphere1" and "sphere2").
            self.assertEqual(len(ids), 2)
        # Cases 2.a: Explicit geometry, constructor (with non-kwarg check).
        geometry_set_options = [
            # Geometry scalar.
            mut.GeometrySet(geometry_id=global_geometry),
            # Geometry list.
            mut.GeometrySet(geometry_ids=[global_geometry]),
            # Geometry list, no kwargs.
            mut.GeometrySet([global_geometry]),
            # Geometry list w/ (empty) frame list.
            mut.GeometrySet(geometry_ids=[global_geometry], frame_ids=[]),
            # Geometry list w/ (empty) frame list, no kwargs.
            mut.GeometrySet([global_geometry], []),
        ]
        # Cases 2.b: Explicit geometry, via Add().
        # - Geometry scalar.
        cur = mut.GeometrySet()
        cur.Add(geometry_id=global_geometry)
        geometry_set_options.append(cur)
        # - Geometry list.
        cur = mut.GeometrySet()
        cur.Add(geometry_ids=[global_geometry])
        geometry_set_options.append(cur)
        # - Geometry list w/ (empty) frame list.
        cur = mut.GeometrySet()
        cur.Add(geometry_ids=[global_geometry], frame_ids=[])
        geometry_set_options.append(cur)
        # Cases 1.*: Test 'em all.
        for geometry_set in geometry_set_options:
            ids = inspector.GetGeometryIds(geometry_set)
            self.assertEqual(len(ids), 1)

        # Only the first sphere has no proximity properties. The latter two
        # have hydroelastic properties (rigid and compliant, respectively).
        self.assertEqual(
            inspector.NumGeometriesWithRole(role=mut.Role.kUnassigned), 1)
        self.assertIsNone(
            inspector.maybe_get_hydroelastic_mesh(
                geometry_id=global_geometry))
        self.assertIsInstance(
            inspector.maybe_get_hydroelastic_mesh(
                geometry_id=sphere_2), mut.SurfaceMesh)
        self.assertIsInstance(
            inspector.maybe_get_hydroelastic_mesh(
                geometry_id=sphere_3), mut.VolumeMesh)
        self.assertEqual(inspector.NumDynamicGeometries(), 2)
        self.assertEqual(inspector.NumAnchoredGeometries(), 1)
        # Sphere 2 and 3 have proximity roles; the pair is a candidate.
        self.assertEqual(len(inspector.GetCollisionCandidates()), 1)
        self.assertTrue(inspector.SourceIsRegistered(source_id=global_source))
        # TODO(SeanCurtis-TRI) Remove this call at the same time as deprecating
        # the subsequent deprecation tests; it is only here to show that the
        # non-keyword call invokes the non-deprecated overload.
        self.assertTrue(inspector.SourceIsRegistered(global_source))
        self.assertEqual(inspector.NumFramesForSource(source_id=global_source),
                         2)
        self.assertTrue(global_frame in inspector.FramesForSource(
            source_id=global_source))
        self.assertTrue(inspector.BelongsToSource(
            frame_id=global_frame, source_id=global_source))
        self.assertEqual(inspector.GetOwningSourceName(frame_id=global_frame),
                         "anchored")
        self.assertEqual(
            inspector.GetName(frame_id=global_frame), "anchored_frame")
        self.assertEqual(inspector.GetFrameGroup(frame_id=global_frame), 0)
        self.assertEqual(
            inspector.NumGeometriesForFrame(frame_id=global_frame), 2)
        self.assertEqual(inspector.NumGeometriesForFrameWithRole(
            frame_id=global_frame, role=mut.Role.kProximity), 1)
        self.assertEqual(len(inspector.GetGeometries(frame_id=global_frame)),
                         2)
        self.assertTrue(
            global_geometry in inspector.GetGeometries(frame_id=global_frame))
        self.assertEqual(
            len(inspector.GetGeometries(frame_id=global_frame,
                                        role=mut.Role.kProximity)),
            1)
        self.assertEqual(
            inspector.GetGeometryIdByName(frame_id=global_frame,
                                          role=mut.Role.kUnassigned,
                                          name="sphere1"),
            global_geometry)
        self.assertTrue(inspector.BelongsToSource(
            geometry_id=global_geometry, source_id=global_source))
        self.assertEqual(
            inspector.GetOwningSourceName(geometry_id=global_geometry),
            "anchored")
        self.assertEqual(inspector.GetFrameId(global_geometry), global_frame)
        self.assertEqual(
            inspector.GetName(geometry_id=global_geometry), "sphere1")
        self.assertIsInstance(inspector.GetShape(geometry_id=global_geometry),
                              mut.Sphere)
        self.assertIsInstance(
            inspector.GetPoseInParent(geometry_id=global_geometry),
            RigidTransform_[float])
        self.assertIsInstance(
            inspector.GetPoseInFrame(geometry_id=global_geometry),
            RigidTransform_[float])
        self.assertIsInstance(inspector.geometry_version(),
                              mut.GeometryVersion)

        # Check AssignRole bits.
        proximity = mut.ProximityProperties()
        perception = mut.PerceptionProperties()
        perception.AddProperty("label", "id", mut.render.RenderLabel(0))
        illustration = mut.IllustrationProperties()
        props = [
            proximity,
            perception,
            illustration,
        ]
        context = scene_graph.CreateDefaultContext()
        for prop in props:
            # Check SceneGraph mutating variant.
            scene_graph.AssignRole(
                source_id=global_source, geometry_id=global_geometry,
                properties=prop, assign=mut.RoleAssign.kNew)
            # Check Context mutating variant.
            scene_graph.AssignRole(
                context=context, source_id=global_source,
                geometry_id=global_geometry, properties=prop,
                assign=mut.RoleAssign.kNew)

        # Check property accessors.
        self.assertIsInstance(
            inspector.GetProximityProperties(geometry_id=global_geometry),
            mut.ProximityProperties)
        self.assertIsInstance(
            inspector.GetProperties(geometry_id=global_geometry,
                                    role=mut.Role.kProximity),
            mut.ProximityProperties)
        self.assertIsInstance(
            inspector.GetIllustrationProperties(geometry_id=global_geometry),
            mut.IllustrationProperties)
        self.assertIsInstance(
            inspector.GetProperties(geometry_id=global_geometry,
                                    role=mut.Role.kIllustration),
            mut.IllustrationProperties)
        self.assertIsInstance(
            inspector.GetPerceptionProperties(geometry_id=global_geometry),
            mut.PerceptionProperties)
        self.assertIsInstance(
            inspector.GetProperties(geometry_id=global_geometry,
                                    role=mut.Role.kPerception),
            mut.PerceptionProperties)
        self.assertIsInstance(
            inspector.CloneGeometryInstance(geometry_id=global_geometry),
            mut.GeometryInstance)
        self.assertTrue(inspector.CollisionFiltered(
            geometry_id1=global_geometry, geometry_id2=global_geometry))

        roles = [
            mut.Role.kProximity,
            mut.Role.kPerception,
            mut.Role.kIllustration,
        ]
        for role in roles:
            self.assertEqual(
                scene_graph.RemoveRole(
                    source_id=global_source, geometry_id=global_geometry,
                    role=role),
                1)

    @numpy_compare.check_nonsymbolic_types
    def test_frame_pose_vector_api(self, T):
        FramePoseVector = mut.FramePoseVector_[T]
        RigidTransform = RigidTransform_[T]
        obj = FramePoseVector()
        frame_id = mut.FrameId.get_new_id()

        obj.set_value(id=frame_id, value=RigidTransform.Identity())
        self.assertEqual(obj.size(), 1)
        self.assertIsInstance(obj.value(id=frame_id), RigidTransform)
        self.assertTrue(obj.has_id(id=frame_id))
        self.assertIsInstance(obj.frame_ids(), list)
        self.assertIsInstance(obj.frame_ids()[0], mut.FrameId)
        obj.clear()
        self.assertEqual(obj.size(), 0)

    @numpy_compare.check_nonsymbolic_types
    def test_penetration_as_point_pair_api(self, T):
        obj = mut.PenetrationAsPointPair_[T]()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_WCa.shape, (3,))
        self.assertTupleEqual(obj.p_WCb.shape, (3,))
        self.assertEqual(obj.depth, -1.)

    @numpy_compare.check_nonsymbolic_types
    def test_signed_distance_api(self, T):
        obj = mut.SignedDistancePair_[T]()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_ACa.shape, (3,))
        self.assertTupleEqual(obj.p_BCb.shape, (3,))
        self.assertIsInstance(obj.distance, T)
        self.assertTupleEqual(obj.nhat_BA_W.shape, (3,))

    @numpy_compare.check_nonsymbolic_types
    def test_signed_distance_to_point_api(self, T):
        obj = mut.SignedDistanceToPoint_[T]()
        self.assertIsInstance(obj.id_G, mut.GeometryId)
        self.assertTupleEqual(obj.p_GN.shape, (3,))
        self.assertIsInstance(obj.distance, T)
        self.assertTupleEqual(obj.grad_W.shape, (3,))

    def test_render_engine_vtk_params(self):
        # Confirm default construction of params.
        params = mut.render.RenderEngineVtkParams()
        self.assertEqual(params.default_label, None)
        self.assertEqual(params.default_diffuse, None)

        label = mut.render.RenderLabel(10)
        diffuse = np.array((1.0, 0.0, 0.0, 0.0))
        params = mut.render.RenderEngineVtkParams(
            default_label=label, default_diffuse=diffuse)
        self.assertEqual(params.default_label, label)
        self.assertTrue((params.default_diffuse == diffuse).all())

    def test_render_label(self):
        RenderLabel = mut.render.RenderLabel
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

    @numpy_compare.check_nonsymbolic_types
    def test_query_object(self, T):
        RigidTransform = RigidTransform_[float]
        SceneGraph = mut.SceneGraph_[T]
        QueryObject = mut.QueryObject_[T]
        SceneGraphInspector = mut.SceneGraphInspector_[T]
        FramePoseVector = mut.FramePoseVector_[T]

        # First, ensure we can default-construct it.
        model = QueryObject()
        self.assertIsInstance(model, QueryObject)

        scene_graph = SceneGraph()
        source_id = scene_graph.RegisterSource("source")
        frame_id = scene_graph.RegisterFrame(
            source_id=source_id, frame=mut.GeometryFrame("frame"))
        geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransform(),
                                          shape=mut.Sphere(1.), name="sphere"))
        render_params = mut.render.RenderEngineVtkParams()
        renderer_name = "test_renderer"
        scene_graph.AddRenderer(renderer_name,
                                mut.render.MakeRenderEngineVtk(render_params))

        context = scene_graph.CreateDefaultContext()
        pose_vector = FramePoseVector()
        pose_vector.set_value(frame_id, RigidTransform_[T]())
        scene_graph.get_source_pose_port(source_id).FixValue(
            context, pose_vector)
        query_object = scene_graph.get_query_output_port().Eval(context)

        self.assertIsInstance(query_object.inspector(), SceneGraphInspector)
        self.assertIsInstance(
            query_object.GetPoseInWorld(frame_id=frame_id), RigidTransform_[T])
        self.assertIsInstance(
            query_object.GetPoseInParent(frame_id=frame_id),
            RigidTransform_[T])
        self.assertIsInstance(
            query_object.GetPoseInWorld(geometry_id=geometry_id),
            RigidTransform_[T])

        # Proximity queries -- all of these will produce empty results.
        results = query_object.ComputeSignedDistancePairwiseClosestPoints()
        self.assertEqual(len(results), 0)
        results = query_object.ComputePointPairPenetration()
        self.assertEqual(len(results), 0)
        results = query_object.ComputeSignedDistanceToPoint(p_WQ=(1, 2, 3))
        self.assertEqual(len(results), 0)
        results = query_object.FindCollisionCandidates()
        self.assertEqual(len(results), 0)
        self.assertFalse(query_object.HasCollisions())

        # ComputeSignedDistancePairClosestPoints() requires two valid geometry
        # ids. There are none in this SceneGraph instance. Rather than
        # populating the SceneGraph, we look for the exception thrown in
        # response to invalid ids as evidence of correct binding.
        with self.assertRaisesRegex(
            RuntimeError,
            r"The geometry given by id \d+ does not reference a geometry"
                + " that can be used in a signed distance query"):
            query_object.ComputeSignedDistancePairClosestPoints(
                geometry_id_A=mut.GeometryId.get_new_id(),
                geometry_id_B=mut.GeometryId.get_new_id())

        # Confirm rendering API returns images of appropriate type.
        camera_core = mut.render.RenderCameraCore(
            renderer_name=renderer_name,
            intrinsics=CameraInfo(width=10, height=10, fov_y=pi/6),
            clipping=mut.render.ClippingRange(0.1, 10.0),
            X_BS=RigidTransform())
        color_camera = mut.render.ColorRenderCamera(
            core=camera_core, show_window=False)
        depth_camera = mut.render.DepthRenderCamera(
            core=camera_core, depth_range=mut.render.DepthRange(0.1, 5.0))
        image = query_object.RenderColorImage(
                camera=color_camera, parent_frame=SceneGraph.world_frame_id(),
                X_PC=RigidTransform())
        self.assertIsInstance(image, ImageRgba8U)
        image = query_object.RenderDepthImage(
            camera=depth_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageDepth32F)
        image = query_object.RenderLabelImage(
            camera=color_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageLabel16I)

    @numpy_compare.check_nonsymbolic_types
    def test_value_instantiations(self, T):
        Value[mut.FramePoseVector_[T]]
        Value[mut.QueryObject_[T]]
        Value[mut.Rgba]
        Value[mut.render.RenderLabel]

    def test_render_engine_api(self):
        class DummyRenderEngine(mut.render.RenderEngine):
            """Mirror of C++ DummyRenderEngine."""

            # See comment below about `rgbd_sensor_test.cc`.
            latest_instance = None

            def __init__(self, render_label=None):
                mut.render.RenderEngine.__init__(self)
                # N.B. We do not hide these because this is a test class.
                # Normally, you would want to hide this.
                self.force_accept = False
                self.registered_geometries = set()
                self.updated_ids = {}
                self.include_group_name = "in_test"
                self.X_WC = RigidTransform_[float]()
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

            def DoClone(self):
                DummyRenderEngine.latest_instance = self
                new = DummyRenderEngine()
                new.force_accept = copy.copy(self.force_accept)
                new.registered_geometries = copy.copy(
                    self.registered_geometries)
                new.updated_ids = copy.copy(self.updated_ids)
                new.include_group_name = copy.copy(self.include_group_name)
                new.X_WC = copy.copy(self.X_WC)
                new.color_count = copy.copy(self.color_count)
                new.depth_count = copy.copy(self.depth_count)
                new.label_count = copy.copy(self.label_count)
                new.color_camera = copy.copy(self.color_camera)
                new.depth_camera = copy.copy(self.depth_camera)
                new.label_camera = copy.copy(self.label_camera)
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

        engine = DummyRenderEngine()
        self.assertIsInstance(engine, mut.render.RenderEngine)
        self.assertIsInstance(engine.Clone(), DummyRenderEngine)

        # Test implementation of C++ interface by using RgbdSensor.
        renderer_name = "renderer"
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(mut.SceneGraph())
        # N.B. This passes ownership.
        scene_graph.AddRenderer(renderer_name, engine)
        sensor = builder.AddSystem(RgbdSensor(
            parent_id=scene_graph.world_frame_id(),
            X_PB=RigidTransform(),
            depth_camera=mut.render.DepthRenderCamera(
                mut.render.RenderCameraCore(
                    renderer_name, CameraInfo(640, 480, np.pi/4),
                    mut.render.ClippingRange(0.1, 5.0), RigidTransform()),
                mut.render.DepthRange(0.1, 5.0))))
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

        # TODO(eric, duy): Test more properties.
