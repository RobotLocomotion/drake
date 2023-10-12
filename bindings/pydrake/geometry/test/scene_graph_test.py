import pydrake.geometry as mut

import unittest
from math import pi

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.value import Value
from pydrake.math import RigidTransform_
from pydrake.symbolic import Expression
from pydrake.systems.framework import InputPort_, OutputPort_
from pydrake.systems.sensors import (
    CameraInfo,
    ImageRgba8U,
    ImageDepth32F,
    ImageLabel16I,
)


class TestGeometrySceneGraph(unittest.TestCase):

    def test_hydroelastic_contact_representation_enum(self):
        mut.HydroelasticContactRepresentation.kTriangle
        mut.HydroelasticContactRepresentation.kPolygon

    @numpy_compare.check_all_types
    def test_scene_graph_api(self, T):
        SceneGraph = mut.SceneGraph_[T]
        InputPort = InputPort_[T]
        OutputPort = OutputPort_[T]

        scene_graph = SceneGraph()
        global_source = scene_graph.RegisterSource("anchored")
        global_frame = scene_graph.RegisterFrame(
            source_id=global_source,
            frame=mut.GeometryFrame("anchored_frame1"))
        scene_graph.RenameFrame(frame_id=global_frame, name="something")
        scene_graph.RenameFrame(frame_id=global_frame, name="anchored_frame1")
        scene_graph.RegisterFrame(
            source_id=global_source, parent_id=global_frame,
            frame=mut.GeometryFrame("anchored_frame2"))
        global_geometry = scene_graph.RegisterGeometry(
            source_id=global_source, frame_id=global_frame,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere1"))
        # We'll explicitly give sphere_2 a rigid hydroelastic representation.
        sphere_2 = scene_graph.RegisterGeometry(
            source_id=global_source, frame_id=global_frame,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere2"))
        scene_graph.RenameGeometry(geometry_id=sphere_2, name="else")
        scene_graph.RenameGeometry(geometry_id=sphere_2, name="sphere2")
        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(resolution_hint=1, properties=props)
        scene_graph.AssignRole(source_id=global_source, geometry_id=sphere_2,
                               properties=props)
        # We'll explicitly give sphere_3 a compliant hydroelastic
        # representation.
        sphere_3 = scene_graph.RegisterAnchoredGeometry(
            source_id=global_source,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere3"))
        props = mut.ProximityProperties()
        mut.AddCompliantHydroelasticProperties(
            resolution_hint=1, hydroelastic_modulus=1e8, properties=props)
        scene_graph.AssignRole(source_id=global_source, geometry_id=sphere_3,
                               properties=props)

        self.assertIsInstance(
            scene_graph.get_source_pose_port(global_source), InputPort)
        self.assertIsInstance(
            scene_graph.get_source_configuration_port(global_source),
            InputPort)

        self.assertIsInstance(
            scene_graph.get_query_output_port(), OutputPort)

        # Test limited rendering API.
        renderer_name = "test_renderer"
        scene_graph.AddRenderer(renderer_name,
                                mut.MakeRenderEngineVtk(
                                    mut.RenderEngineVtkParams()))
        self.assertTrue(scene_graph.HasRenderer(renderer_name))
        self.assertEqual(scene_graph.RendererCount(), 1)
        renderer_type_name = scene_graph.GetRendererTypeName(
            name=renderer_name)

        scene_graph.RemoveRenderer(renderer_name)
        self.assertFalse(scene_graph.HasRenderer(renderer_name))
        self.assertEqual(scene_graph.RendererCount(), 0)

        # Now add the renderer back.
        scene_graph.AddRenderer(renderer_name,
                                mut.MakeRenderEngineVtk(
                                    mut.RenderEngineVtkParams()))

        # Test SceneGraphInspector API
        inspector = scene_graph.model_inspector()
        self.assertEqual(inspector.num_sources(), 2)
        self.assertEqual(inspector.num_frames(), 3)
        self.assertEqual(len(inspector.GetAllSourceIds()), 2)
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
                geometry_id=sphere_2), mut.TriangleSurfaceMesh)
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
            inspector.GetName(frame_id=global_frame), "anchored_frame1")
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
            inspector.GetPoseInFrame(geometry_id=global_geometry),
            RigidTransform_[float])
        self.assertIsInstance(inspector.geometry_version(),
                              mut.GeometryVersion)

        # Check AssignRole bits.
        proximity = mut.ProximityProperties()
        perception = mut.PerceptionProperties()
        perception.AddProperty("label", "id", mut.RenderLabel(0))
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
        for i, role in enumerate(roles):
            # Check GeometryId SceneGraph mutating variant.
            self.assertEqual(
                scene_graph.RemoveRole(
                    source_id=global_source, geometry_id=global_geometry,
                    role=role),
                1)
            # Check GeometryId Context mutating variant.
            self.assertEqual(
                scene_graph.RemoveRole(
                    context=context, source_id=global_source,
                    geometry_id=global_geometry, role=role),
                1)
            # Check FrameId SceneGraph mutating variant.
            self.assertEqual(
                scene_graph.RemoveRole(
                    source_id=global_source, frame_id=global_frame,
                    role=role),
                1 if i == 0 else 0)
            # Check FrameId Context mutating variant.
            self.assertEqual(
                scene_graph.RemoveRole(
                    context=context, source_id=global_source,
                    frame_id=global_frame, role=role),
                1 if i == 0 else 0)

    @numpy_compare.check_all_types
    def test_scene_graph_renderer_with_context(self, T):
        SceneGraph = mut.SceneGraph_[T]
        scene_graph = SceneGraph()
        context = scene_graph.CreateDefaultContext()
        self.assertEqual(scene_graph.RendererCount(context), 0)
        render_params = mut.RenderEngineVtkParams()
        renderer_name = "test_renderer"
        self.assertFalse(
            scene_graph.HasRenderer(context=context, name=renderer_name))
        scene_graph.AddRenderer(
            context=context, name=renderer_name,
            renderer=mut.MakeRenderEngineVtk(params=render_params))
        self.assertEqual(scene_graph.RendererCount(context=context), 1)
        self.assertTrue(
            scene_graph.HasRenderer(context=context, name=renderer_name))
        scene_graph.RemoveRenderer(context=context, name=renderer_name)
        self.assertEqual(scene_graph.RendererCount(context=context), 0)
        renderer_type_name = scene_graph.GetRendererTypeName(
            context=context, name=renderer_name)

    @numpy_compare.check_all_types
    def test_scene_graph_register_geometry(self, T):
        SceneGraph = mut.SceneGraph_[T]
        scene_graph = SceneGraph()
        global_source = scene_graph.RegisterSource("anchored")
        global_frame = scene_graph.world_frame_id()
        context = scene_graph.CreateDefaultContext()
        model_inspector = scene_graph.model_inspector()
        query_object = scene_graph.get_query_output_port().Eval(context)
        context_inspector = query_object.inspector()
        self.assertEqual(model_inspector.num_geometries(), 0)
        self.assertEqual(context_inspector.num_geometries(), 0)

        # Register with context
        geometry = mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                        shape=mut.Sphere(1.),
                                        name="sphere1")
        global_geometry = scene_graph.RegisterGeometry(
            context=context, source_id=global_source, frame_id=global_frame,
            geometry=geometry)
        self.assertEqual(model_inspector.num_geometries(), 0)
        self.assertEqual(context_inspector.num_geometries(), 1)

        # Now register the geometry in scene_graph with a new geometry
        new_geometry = mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                            shape=mut.Sphere(1.),
                                            name="sphere1")
        scene_graph.RegisterGeometry(
            source_id=global_source, frame_id=global_frame,
            geometry=new_geometry)
        self.assertEqual(model_inspector.num_geometries(), 1)
        self.assertEqual(context_inspector.num_geometries(), 1)

    @numpy_compare.check_all_types
    def test_scene_graph_change_shape(self, T):
        SceneGraph = mut.SceneGraph_[T]
        RigidTransform = RigidTransform_[float]
        scene_graph = SceneGraph()
        source_id = scene_graph.RegisterSource("anchored")
        frame_id = scene_graph.world_frame_id()
        identity = RigidTransform()
        X_FG_alt = RigidTransform(p=(1, 2, 3))
        geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=mut.GeometryInstance(X_PG=identity,
                                          shape=mut.Sphere(1.),
                                          name="sphere1"))
        context = scene_graph.CreateDefaultContext()
        model_inspector = scene_graph.model_inspector()

        self.assertIsInstance(model_inspector.GetShape(geometry_id),
                              mut.Sphere)
        scene_graph.ChangeShape(source_id=source_id, geometry_id=geometry_id,
                                shape=mut.Box(1, 2, 3))
        self.assertIsInstance(model_inspector.GetShape(geometry_id),
                              mut.Box)
        scene_graph.ChangeShape(source_id=source_id, geometry_id=geometry_id,
                                shape=mut.Capsule(1, 2), X_FG=X_FG_alt)
        self.assertIsInstance(model_inspector.GetShape(geometry_id),
                              mut.Capsule)
        self.assertTrue(
            (model_inspector.GetPoseInFrame(geometry_id).translation()
             == X_FG_alt.translation()).all())

        query_object = scene_graph.get_query_output_port().Eval(context)
        context_inspector = query_object.inspector()

        self.assertIsInstance(context_inspector.GetShape(geometry_id),
                              mut.Sphere)
        scene_graph.ChangeShape(
            context=context, source_id=source_id, geometry_id=geometry_id,
            shape=mut.Box(1, 2, 3))
        self.assertIsInstance(context_inspector.GetShape(geometry_id),
                              mut.Box)
        scene_graph.ChangeShape(
            context=context, source_id=source_id, geometry_id=geometry_id,
            shape=mut.Capsule(1, 2), X_FG=X_FG_alt)
        self.assertIsInstance(context_inspector.GetShape(geometry_id),
                              mut.Capsule)
        self.assertTrue(
            (context_inspector.GetPoseInFrame(geometry_id).translation()
             == X_FG_alt.translation()).all())

    @numpy_compare.check_all_types
    def test_scene_graph_remove_geometry(self, T):
        SceneGraph = mut.SceneGraph_[T]

        scene_graph = SceneGraph()
        global_source = scene_graph.RegisterSource("anchored")
        global_frame = scene_graph.world_frame_id()
        global_geometry = scene_graph.RegisterGeometry(
            source_id=global_source, frame_id=global_frame,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere1"))
        context = scene_graph.CreateDefaultContext()
        model_inspector = scene_graph.model_inspector()
        query_object = scene_graph.get_query_output_port().Eval(context)
        context_inspector = query_object.inspector()
        # Call the version that only removes the geometry in the context.
        scene_graph.RemoveGeometry(
            context=context,
            source_id=global_source,
            geometry_id=global_geometry)
        self.assertEqual(model_inspector.num_geometries(), 1)
        self.assertEqual(context_inspector.num_geometries(), 0)
        # Now remove the geometry in scene_graph
        scene_graph.RemoveGeometry(
            source_id=global_source, geometry_id=global_geometry)
        self.assertEqual(model_inspector.num_geometries(), 0)

    @numpy_compare.check_all_types
    def test_frame_pose_vector_api(self, T):
        FramePoseVector = mut.FramePoseVector_[T]
        RigidTransform = RigidTransform_[T]
        obj = FramePoseVector()
        frame_id = mut.FrameId.get_new_id()

        obj.set_value(id=frame_id, value=RigidTransform.Identity())
        self.assertEqual(obj.size(), 1)
        self.assertIsInstance(obj.value(id=frame_id), RigidTransform)
        self.assertTrue(obj.has_id(id=frame_id))
        self.assertIsInstance(obj.ids(), list)
        self.assertIsInstance(obj.ids()[0], mut.FrameId)
        obj.clear()
        self.assertEqual(obj.size(), 0)

    @numpy_compare.check_all_types
    def test_penetration_as_point_pair_api(self, T):
        obj = mut.PenetrationAsPointPair_[T]()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_WCa.shape, (3,))
        self.assertTupleEqual(obj.p_WCb.shape, (3,))
        if T == Expression:
            self.assertTrue(obj.depth.EqualTo(-1.0))
        else:
            self.assertEqual(obj.depth, -1.0)

    @numpy_compare.check_all_types
    def test_signed_distance_api(self, T):
        obj = mut.SignedDistancePair_[T]()
        self.assertIsInstance(obj.id_A, mut.GeometryId)
        self.assertIsInstance(obj.id_B, mut.GeometryId)
        self.assertTupleEqual(obj.p_ACa.shape, (3,))
        self.assertTupleEqual(obj.p_BCb.shape, (3,))
        self.assertIsInstance(obj.distance, T)
        self.assertTupleEqual(obj.nhat_BA_W.shape, (3,))

    @numpy_compare.check_all_types
    def test_signed_distance_to_point_api(self, T):
        obj = mut.SignedDistanceToPoint_[T]()
        self.assertIsInstance(obj.id_G, mut.GeometryId)
        self.assertTupleEqual(obj.p_GN.shape, (3,))
        self.assertIsInstance(obj.distance, T)
        self.assertTupleEqual(obj.grad_W.shape, (3,))

    @numpy_compare.check_all_types
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
        render_params = mut.RenderEngineVtkParams()
        renderer_name = "test_renderer"
        scene_graph.AddRenderer(renderer_name,
                                mut.MakeRenderEngineVtk(params=render_params))

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
        if T != Expression:
            hydro_rep = mut.HydroelasticContactRepresentation.kTriangle
            results = query_object.ComputeContactSurfaces(
                representation=hydro_rep)
            self.assertEqual(len(results), 0)
            surfaces, results = query_object.ComputeContactSurfacesWithFallback(  # noqa
                representation=hydro_rep)
            self.assertEqual(len(surfaces), 0)
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
                "Referenced geometry .+ has not been registered."):
            query_object.ComputeSignedDistancePairClosestPoints(
                geometry_id_A=mut.GeometryId.get_new_id(),
                geometry_id_B=mut.GeometryId.get_new_id())

        # Confirm rendering API returns images of appropriate type.
        camera_core = mut.RenderCameraCore(
            renderer_name=renderer_name,
            intrinsics=CameraInfo(width=10, height=10, fov_y=pi/6),
            clipping=mut.ClippingRange(0.1, 10.0),
            X_BS=RigidTransform())
        color_camera = mut.ColorRenderCamera(
            core=camera_core, show_window=False)
        depth_camera = mut.DepthRenderCamera(
            core=camera_core, depth_range=mut.DepthRange(0.1, 5.0))
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

    @numpy_compare.check_all_types
    def test_value_instantiations(self, T):
        Value[mut.FramePoseVector_[T]]
        Value[mut.QueryObject_[T]]

    @numpy_compare.check_nonsymbolic_types
    def test_contact_surface(self, T):
        # We can't construct a ContactSurface directly. So, we need to evaluate
        # hydroelastic contact in order to get a result that we can assess.
        RigidTransform = RigidTransform_[T]
        RigidTransformd = RigidTransform_[float]
        SceneGraph = mut.SceneGraph_[T]
        FramePoseVector = mut.FramePoseVector_[T]

        scene_graph = SceneGraph()
        s_id = scene_graph.RegisterSource("source")

        # Add a compliant "moving" ball.
        f_id = scene_graph.RegisterFrame(
            source_id=s_id, frame=mut.GeometryFrame("frame"))
        g_id0 = scene_graph.RegisterGeometry(
            source_id=s_id, frame_id=f_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransformd(),
                                          shape=mut.Sphere(1.), name="sphere"))
        props = mut.ProximityProperties()
        mut.AddCompliantHydroelasticProperties(
            resolution_hint=1.0, hydroelastic_modulus=1e5, properties=props)
        scene_graph.AssignRole(s_id, g_id0, props)

        # Add a rigd half space.
        g_id1 = scene_graph.RegisterAnchoredGeometry(
            source_id=s_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransformd(),
                                          shape=mut.HalfSpace(), name="plane"))
        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(properties=props)
        scene_graph.AssignRole(s_id, g_id1, props)

        context = scene_graph.CreateDefaultContext()

        # Provide poses so we can evaluate the query object. The poses should
        # lead to a single collision.
        poses = FramePoseVector()
        poses.set_value(id=f_id, value=RigidTransform([0, 0, 0.5]))
        scene_graph.get_source_pose_port(s_id).FixValue(context, poses)
        query_object = scene_graph.get_query_output_port().Eval(context)

        # Test both mesh representations.
        for rep in (mut.HydroelasticContactRepresentation.kTriangle,
                    mut.HydroelasticContactRepresentation.kPolygon):
            expect_triangles = (
                rep == mut.HydroelasticContactRepresentation.kTriangle)

            results = query_object.ComputeContactSurfaces(rep)

            self.assertEqual(len(results), 1)
            contact_surface = results[0]
            self.assertLess(g_id0, g_id1)  # confirm M = 0 and N = 1.
            self.assertEqual(contact_surface.id_M(), g_id0)
            self.assertEqual(contact_surface.id_N(), g_id1)
            self.assertGreater(contact_surface.num_faces(), 0)
            self.assertGreater(contact_surface.num_vertices(), 0)
            self.assertGreater(contact_surface.area(face_index=0), 0)
            self.assertGreater(contact_surface.total_area(), 0)
            contact_surface.face_normal(face_index=0)
            contact_surface.centroid(face_index=0)
            contact_surface.centroid()

            self.assertEqual(contact_surface.is_triangle(), expect_triangles)
            self.assertEqual(contact_surface.representation(), rep)
            if expect_triangles:
                # Details of mesh are tested in geometry_hydro_test.py
                contact_surface.tri_mesh_W()
                field = contact_surface.tri_e_MN()
                # Only triangle mesh fields can evaluate barycentric coords.
                field.Evaluate(e=0, b=(0.25, 0.5, 0.25))
            else:
                # Details of mesh are tested in geometry_hydro_test.py
                contact_surface.poly_mesh_W()
                field = contact_surface.poly_e_MN()
                # Only the Polygonal mesh has gradients pre-computed.
                field.EvaluateGradient(e=0)
            # APIs available to both Triangle and Polygon-based fields.
            field.EvaluateAtVertex(v=0)
            field.EvaluateCartesian(e=0, p_MQ=(0.25, 0.25, 0))
