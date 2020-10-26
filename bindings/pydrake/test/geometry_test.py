import pydrake.geometry as mut
import pydrake.geometry._testing as mut_testing

import copy
import unittest
from math import pi

import numpy as np

from drake import lcmt_viewer_load_robot, lcmt_viewer_draw
from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.value import AbstractValue, Value
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.math import RigidTransform, RigidTransform_
from pydrake.systems.analysis import (
    Simulator_,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    DiagramBuilder_,
    InputPort_,
    OutputPort_,
)
from pydrake.systems.sensors import (
    ImageRgba8U,
    ImageDepth16U,
    ImageDepth32F,
    ImageLabel16I,
    RgbdSensor,
)


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
        scene_graph.RegisterGeometry(
            source_id=global_source, geometry_id=global_geometry,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere2"))
        scene_graph.RegisterAnchoredGeometry(
            source_id=global_source,
            geometry=mut.GeometryInstance(X_PG=RigidTransform_[float](),
                                          shape=mut.Sphere(1.),
                                          name="sphere3"))
        self.assertIsInstance(
            scene_graph.get_source_pose_port(global_source), InputPort)
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
        self.assertEqual(inspector.num_geometries(), 3)
        self.assertEqual(len(inspector.GetAllGeometryIds()), 3)
        self.assertEqual(
            inspector.NumGeometriesWithRole(role=mut.Role.kUnassigned), 3)
        self.assertEqual(inspector.NumDynamicGeometries(), 2)
        self.assertEqual(inspector.NumAnchoredGeometries(), 1)
        self.assertTrue(inspector.SourceIsRegistered(id=global_source))
        self.assertEqual(inspector.GetSourceName(id=global_source), "anchored")
        self.assertEqual(inspector.GetFrameId(global_geometry), global_frame)
        self.assertEqual(len(inspector.GetGeometries(frame_id=global_frame)),
                         2)
        self.assertTrue(
            global_geometry in inspector.GetGeometries(frame_id=global_frame))
        self.assertEqual(
            len(inspector.GetGeometries(frame_id=global_frame,
                                        role=mut.Role.kProximity)),
            0)
        self.assertEqual(
            inspector.GetGeometryIdByName(frame_id=global_frame,
                                          role=mut.Role.kUnassigned,
                                          name="sphere1"),
            global_geometry)
        self.assertEqual(
            inspector.GetName(frame_id=global_frame), "anchored_frame")
        self.assertEqual(
            inspector.GetName(geometry_id=global_geometry), "sphere1")

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
            inspector.GetIllustrationProperties(geometry_id=global_geometry),
            mut.IllustrationProperties)
        self.assertIsInstance(
            inspector.GetPerceptionProperties(geometry_id=global_geometry),
            mut.PerceptionProperties)
        self.assertIsInstance(
            inspector.CloneGeometryInstance(geometry_id=global_geometry),
            mut.GeometryInstance)

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

    def test_connect_drake_visualizer(self):
        # Test visualization API.
        T = float
        SceneGraph = mut.SceneGraph_[T]
        DiagramBuilder = DiagramBuilder_[T]
        Simulator = Simulator_[T]
        lcm = DrakeLcm()
        role = mut.Role.kIllustration

        def normal(builder, scene_graph):
            mut.ConnectDrakeVisualizer(
                builder=builder, scene_graph=scene_graph,
                lcm=lcm, role=role)
            mut.DispatchLoadMessage(
                scene_graph=scene_graph, lcm=lcm, role=role)

        def port(builder, scene_graph):
            mut.ConnectDrakeVisualizer(
                builder=builder, scene_graph=scene_graph,
                pose_bundle_output_port=(
                    scene_graph.get_pose_bundle_output_port()),
                lcm=lcm, role=role)
            mut.DispatchLoadMessage(
                scene_graph=scene_graph, lcm=lcm, role=role)

        for func in [normal, port]:
            # Create subscribers.
            load_channel = "DRAKE_VIEWER_LOAD_ROBOT"
            draw_channel = "DRAKE_VIEWER_DRAW"
            load_subscriber = Subscriber(
                lcm, load_channel, lcmt_viewer_load_robot)
            draw_subscriber = Subscriber(
                lcm, draw_channel, lcmt_viewer_draw)
            # Test sequence.
            builder = DiagramBuilder()
            scene_graph = builder.AddSystem(SceneGraph())
            # Only load will be published by `DispatchLoadMessage`.
            func(builder, scene_graph)
            lcm.HandleSubscriptions(0)
            self.assertEqual(load_subscriber.count, 1)
            self.assertEqual(draw_subscriber.count, 0)
            diagram = builder.Build()
            # Load and draw will be published.
            Simulator(diagram).Initialize()
            lcm.HandleSubscriptions(0)
            self.assertEqual(load_subscriber.count, 2)
            self.assertEqual(draw_subscriber.count, 1)

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

    def test_identifier_api(self):
        cls_list = [
            mut_testing.FakeId,
            mut.SourceId,
            mut.FrameId,
            mut.GeometryId,
        ]

        for cls in cls_list:
            a = cls.get_new_id()
            self.assertTrue(a.is_valid())
            b = cls.get_new_id()
            self.assertTrue(a == a)
            self.assertFalse(a == b)
            # N.B. Creation order does not imply value.
            self.assertTrue(a < b or b > a)

        fake_id_1 = mut_testing.get_fake_id_constant()
        fake_id_2 = mut_testing.get_fake_id_constant()
        self.assertIsNot(fake_id_1, fake_id_2)
        self.assertEqual(hash(fake_id_1), hash(fake_id_2))

        self.assertEqual(
            repr(fake_id_1),
            f"<FakeId value={fake_id_1.get_value()}>")

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

    def test_shape_constructors(self):
        box_mesh_path = FindResourceOrThrow(
            "drake/systems/sensors/test/models/meshes/box.obj")
        shapes = [
            mut.Sphere(radius=1.0),
            mut.Cylinder(radius=1.0, length=2.0),
            mut.Box(width=1.0, depth=2.0, height=3.0),
            mut.Capsule(radius=1.0, length=2.0),
            mut.Ellipsoid(a=1.0, b=2.0, c=3.0),
            mut.HalfSpace(),
            mut.Mesh(absolute_filename=box_mesh_path, scale=1.0),
            mut.Convex(absolute_filename=box_mesh_path, scale=1.0)
        ]
        for shape in shapes:
            self.assertIsInstance(shape, mut.Shape)
            shape_cls = type(shape)
            shape_copy = shape.Clone()
            self.assertIsInstance(shape_copy, shape_cls)
            self.assertIsNot(shape, shape_copy)

    def test_shapes(self):
        RigidTransform = RigidTransform_[float]
        sphere = mut.Sphere(radius=1.0)
        self.assertEqual(sphere.radius(), 1.0)
        cylinder = mut.Cylinder(radius=1.0, length=2.0)
        self.assertEqual(cylinder.radius(), 1.0)
        self.assertEqual(cylinder.length(), 2.0)
        box = mut.Box(width=1.0, depth=2.0, height=3.0)
        self.assertEqual(box.width(), 1.0)
        self.assertEqual(box.depth(), 2.0)
        self.assertEqual(box.height(), 3.0)
        numpy_compare.assert_float_equal(box.size(), np.array([1.0, 2.0, 3.0]))
        capsule = mut.Capsule(radius=1.0, length=2.0)
        self.assertEqual(capsule.radius(), 1.0)
        self.assertEqual(capsule.length(), 2.0)
        ellipsoid = mut.Ellipsoid(a=1.0, b=2.0, c=3.0)
        self.assertEqual(ellipsoid.a(), 1.0)
        self.assertEqual(ellipsoid.b(), 2.0)
        self.assertEqual(ellipsoid.c(), 3.0)
        X_FH = mut.HalfSpace.MakePose(Hz_dir_F=[0, 1, 0], p_FB=[1, 1, 1])
        self.assertIsInstance(X_FH, RigidTransform)
        box_mesh_path = FindResourceOrThrow(
            "drake/systems/sensors/test/models/meshes/box.obj")
        mesh = mut.Mesh(absolute_filename=box_mesh_path, scale=1.0)
        self.assertEqual(mesh.filename(), box_mesh_path)
        self.assertEqual(mesh.scale(), 1.0)
        convex = mut.Convex(absolute_filename=box_mesh_path, scale=1.0)
        self.assertEqual(convex.filename(), box_mesh_path)
        self.assertEqual(convex.scale(), 1.0)

    def test_geometry_frame_api(self):
        frame = mut.GeometryFrame(frame_name="test_frame")
        self.assertIsInstance(frame.id(), mut.FrameId)
        self.assertEqual(frame.name(), "test_frame")
        frame = mut.GeometryFrame(frame_name="test_frame", frame_group_id=1)
        self.assertEqual(frame.frame_group(), 1)

    def test_geometry_instance_api(self):
        RigidTransform = RigidTransform_[float]
        geometry = mut.GeometryInstance(X_PG=RigidTransform(),
                                        shape=mut.Sphere(1.), name="sphere")
        self.assertIsInstance(geometry.id(), mut.GeometryId)
        geometry.set_pose(RigidTransform([1, 0, 0]))
        self.assertIsInstance(geometry.pose(), RigidTransform)
        self.assertIsInstance(geometry.shape(), mut.Shape)
        self.assertIsInstance(geometry.release_shape(), mut.Shape)
        self.assertEqual(geometry.name(), "sphere")
        geometry.set_name("funky")
        self.assertEqual(geometry.name(), "funky")
        geometry.set_proximity_properties(mut.ProximityProperties())
        geometry.set_illustration_properties(mut.IllustrationProperties())
        geometry.set_perception_properties(mut.PerceptionProperties())
        self.assertIsInstance(geometry.mutable_proximity_properties(),
                              mut.ProximityProperties)
        self.assertIsInstance(geometry.proximity_properties(),
                              mut.ProximityProperties)
        self.assertIsInstance(geometry.mutable_illustration_properties(),
                              mut.IllustrationProperties)
        self.assertIsInstance(geometry.illustration_properties(),
                              mut.IllustrationProperties)
        self.assertIsInstance(geometry.mutable_perception_properties(),
                              mut.PerceptionProperties)
        self.assertIsInstance(geometry.perception_properties(),
                              mut.PerceptionProperties)

    def test_geometry_version_api(self):
        SceneGraph = mut.SceneGraph_[float]
        scene_graph = SceneGraph()
        inspector = scene_graph.model_inspector()
        version0 = inspector.geometry_version()
        version1 = copy.deepcopy(version0)
        self.assertTrue(version0.IsSameAs(other=version1,
                                          role=mut.Role.kProximity))
        self.assertTrue(version0.IsSameAs(other=version1,
                                          role=mut.Role.kPerception))
        self.assertTrue(version0.IsSameAs(other=version1,
                                          role=mut.Role.kIllustration))
        version2 = mut.GeometryVersion(other=version0)
        self.assertTrue(version0.IsSameAs(other=version2,
                                          role=mut.Role.kProximity))
        self.assertTrue(version0.IsSameAs(other=version2,
                                          role=mut.Role.kPerception))
        self.assertTrue(version0.IsSameAs(other=version2,
                                          role=mut.Role.kIllustration))

    def test_rgba_api(self):
        r, g, b, a = 0.75, 0.5, 0.25, 1.
        color = mut.Rgba(r=r, g=g, b=b)
        self.assertEqual(color.r(), r)
        self.assertEqual(color.g(), g)
        self.assertEqual(color.b(), b)
        self.assertEqual(color.a(), a)
        self.assertEqual(color, mut.Rgba(r, g, b, a))
        self.assertNotEqual(color, mut.Rgba(r, g, b, 0.))
        self.assertEqual(
            repr(color),
            "Rgba(r=0.75, g=0.5, b=0.25, a=1.0)")
        color.set(r=1., g=1., b=1., a=0.)
        self.assertEqual(color, mut.Rgba(1., 1., 1., 0.))

    def test_geometry_properties_api(self):
        # Test perception/ illustration properties (specifically Rgba).
        test_vector = [0., 0., 1., 1.]
        test_color = mut.Rgba(0., 0., 1., 1.)
        phong_props = mut.MakePhongIllustrationProperties(test_vector)
        self.assertIsInstance(phong_props, mut.IllustrationProperties)
        actual_color = phong_props.GetProperty("phong", "diffuse")
        self.assertEqual(actual_color, test_color)
        # Ensure that we can create it manually.
        phong_props = mut.IllustrationProperties()
        phong_props.AddProperty("phong", "diffuse", test_color)
        actual_color = phong_props.GetProperty("phong", "diffuse")
        self.assertEqual(actual_color, test_color)
        # Test proximity properties.
        prop = mut.ProximityProperties()
        self.assertEqual(str(prop), "[__default__]")
        default_group = prop.default_group_name()
        self.assertTrue(prop.HasGroup(group_name=default_group))
        self.assertEqual(prop.num_groups(), 1)
        self.assertTrue(default_group in prop.GetGroupNames())
        prop.AddProperty(group_name=default_group, name="test", value=3)
        self.assertTrue(prop.HasProperty(group_name=default_group,
                                         name="test"))
        self.assertEqual(
            prop.GetProperty(group_name=default_group, name="test"), 3)
        self.assertEqual(
            prop.GetPropertyOrDefault(
                group_name=default_group, name="empty", default_value=5),
            5)
        group_values = prop.GetPropertiesInGroup(group_name=default_group)
        for name, value in group_values.items():
            self.assertIsInstance(name, str)
            self.assertIsInstance(value, AbstractValue)
        # Remove the property.
        self.assertTrue(prop.RemoveProperty(group_name=default_group,
                                            name="test"))
        self.assertFalse(prop.HasProperty(group_name=default_group,
                                          name="test"))
        # Update a property.
        prop.AddProperty(group_name=default_group, name="to_update", value=17)
        self.assertTrue(prop.HasProperty(group_name=default_group,
                                         name="to_update"))
        self.assertEqual(
            prop.GetProperty(group_name=default_group, name="to_update"), 17)

        prop.UpdateProperty(group_name=default_group, name="to_update",
                            value=20)
        self.assertTrue(prop.HasProperty(group_name=default_group,
                                         name="to_update"))
        self.assertEqual(
            prop.GetProperty(group_name=default_group, name="to_update"),
            20)

        # Property copying.
        for PropertyType in [mut.ProximityProperties,
                             mut.IllustrationProperties,
                             mut.PerceptionProperties]:
            props = PropertyType()
            props.AddProperty("g", "p", 10)
            self.assertTrue(props.HasProperty("g", "p"))
            props_copy = PropertyType(other=props)
            self.assertTrue(props_copy.HasProperty("g", "p"))
            props_copy2 = copy.copy(props)
            self.assertTrue(props_copy2.HasProperty("g", "p"))
            props_copy3 = copy.deepcopy(props)
            self.assertTrue(props_copy3.HasProperty("g", "p"))

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

    def test_render_engine_opsray(self):
        params = mut.render.RenderEngineOsprayParams()
        self.assertEqual(params.mode, mut.render.OsprayMode.kPathTracer)
        self.assertIs(params.default_diffuse, None)
        self.assertIs(params.background_color, None)
        self.assertEqual(params.samples_per_pixel, 1)
        self.assertEqual(params.use_shadows, True)

        params = mut.render.RenderEngineOsprayParams(
            mode=mut.render.OsprayMode.kRayTracer,
            default_diffuse=[0.1, 0.2, 0.3, 0.4],
            background_color=[1., 1., 1.],
            samples_per_pixel=2,
            use_shadows=False,
        )
        self.assertEqual(params.mode, mut.render.OsprayMode.kRayTracer)
        np.testing.assert_array_equal(
            params.default_diffuse, [0.1, 0.2, 0.3, 0.4])
        np.testing.assert_array_equal(params.background_color, [1., 1., 1])
        self.assertEqual(params.samples_per_pixel, 2)
        self.assertEqual(params.use_shadows, False)

        # Acceptance-test construction of renderer.
        renderer = mut.render.MakeRenderEngineOspray(params=params)
        self.assertIsInstance(renderer, mut.render.RenderEngine)

    def test_render_depth_camera_properties(self):
        obj = mut.render.DepthCameraProperties(width=320, height=240,
                                               fov_y=pi/6,
                                               renderer_name="test_renderer",
                                               z_near=0.1, z_far=5.0)
        self.assertEqual(obj.width, 320)
        self.assertEqual(obj.height, 240)
        self.assertEqual(obj.fov_y, pi/6)
        self.assertEqual(obj.renderer_name, "test_renderer")
        self.assertEqual(obj.z_near, 0.1)
        self.assertEqual(obj.z_far, 5.0)

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
            query_object.X_WF(id=frame_id), RigidTransform_[T])
        self.assertIsInstance(
            query_object.X_PF(id=frame_id), RigidTransform_[T])
        self.assertIsInstance(
            query_object.X_WG(id=geometry_id), RigidTransform_[T])

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
        self.assertRaisesRegex(
            RuntimeError,
            "The geometry given by id \\d+ does not reference a geometry"
            + " that can be used in a signed distance query",
            query_object.ComputeSignedDistancePairClosestPoints,
            mut.GeometryId.get_new_id(), mut.GeometryId.get_new_id())

        # Confirm rendering API returns images of appropriate type.
        d_camera = mut.render.DepthCameraProperties(
            width=320, height=240, fov_y=pi/6, renderer_name=renderer_name,
            z_near=0.1, z_far=5.0)
        image = query_object.RenderColorImage(
            camera=d_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageRgba8U)
        image = query_object.RenderDepthImage(
            camera=d_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageDepth32F)
        image = query_object.RenderLabelImage(
            camera=d_camera, parent_frame=SceneGraph.world_frame_id(),
            X_PC=RigidTransform())
        self.assertIsInstance(image, ImageLabel16I)

    def test_read_obj_to_surface_mesh(self):
        mesh_path = FindResourceOrThrow("drake/geometry/test/quad_cube.obj")
        mesh = mut.ReadObjToSurfaceMesh(mesh_path)
        vertices = mesh.vertices()

        # This test relies on the specific content of the file quad_cube.obj.
        # These coordinates came from the first section of quad_cube.obj.
        expected_vertices = [
            [1.000000, -1.000000, -1.000000],
            [1.000000, -1.000000,  1.000000],
            [-1.000000, -1.000000,  1.000000],
            [-1.000000, -1.000000, -1.000000],
            [1.000000,  1.000000, -1.000000],
            [1.000000,  1.000000,  1.000000],
            [-1.000000,  1.000000,  1.000000],
            [-1.000000,  1.000000, -1.000000],
        ]
        for i, expected in enumerate(expected_vertices):
            self.assertListEqual(list(vertices[i].r_MV()), expected)

    def test_collision_filtering(self):
        sg = mut.SceneGraph()
        sg_context = sg.CreateDefaultContext()
        geometries = mut.GeometrySet()

        sg.ExcludeCollisionsBetween(geometries, geometries)
        sg.ExcludeCollisionsBetween(sg_context, geometries, geometries)
        sg.ExcludeCollisionsWithin(geometries)
        sg.ExcludeCollisionsWithin(sg_context, geometries)

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
                self.simple_color_count = 0
                self.simple_depth_count = 0
                self.simple_label_count = 0
                self.color_props = None
                self.depth_props = None
                self.label_props = None

            def UpdateViewpoint(self, X_WC):
                DummyRenderEngine.latest_instance = self
                self.X_WC = X_WC

            def DoRenderColorImage(self, camera, color_image_out):
                DummyRenderEngine.latest_instance = self
                self.simple_color_count += 1
                self.color_props = camera

            def DoRenderDepthImage(self, camera, depth_image_out):
                DummyRenderEngine.latest_instance = self
                self.simple_depth_count += 1
                self.depth_props = camera

            def DoRenderLabelImage(self, camera, label_image_out):
                DummyRenderEngine.latest_instance = self
                self.simple_label_count += 1
                self.label_props = camera

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
                new.simple_color_count = copy.copy(self.simple_color_count)
                new.simple_depth_count = copy.copy(self.simple_depth_count)
                new.simple_label_count = copy.copy(self.simple_label_count)
                new.color_props = copy.copy(self.color_props)
                new.depth_props = copy.copy(self.depth_props)
                new.label_props = copy.copy(self.label_props)
                return new

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
            properties=mut.render.DepthCameraProperties(
                width=640, height=480, fov_y=np.pi/4,
                renderer_name="renderer", z_near=0.1, z_far=5.0,
            ),
            camera_poses=RgbdSensor.CameraPoses(),
            show_window=False,
        ))
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
        self.assertEqual(current_engine.simple_color_count, 1)

        image = sensor.depth_image_32F_output_port().Eval(sensor_context)
        self.assertIsInstance(image, ImageDepth32F)
        self.assertEqual(current_engine.simple_depth_count, 1)

        image = sensor.depth_image_16U_output_port().Eval(sensor_context)
        self.assertIsInstance(image, ImageDepth16U)
        self.assertEqual(current_engine.simple_depth_count, 2)

        image = sensor.label_image_output_port().Eval(sensor_context)
        self.assertIsInstance(image, ImageLabel16I)
        self.assertEqual(current_engine.simple_label_count, 1)

        # TODO(eric, duy): Test more properties.
