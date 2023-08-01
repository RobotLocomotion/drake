import pydrake.geometry as mut
import pydrake.geometry._testing as mut_testing

import copy
import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue, Value
from pydrake.common.yaml import yaml_load_typed
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction

PROPERTY_CLS_LIST = [
    mut.ProximityProperties,
    mut.IllustrationProperties,
    mut.PerceptionProperties,
]


class TestGeometryCore(unittest.TestCase):
    def test_collision_filtering(self):
        sg = mut.SceneGraph()
        sg_context = sg.CreateDefaultContext()
        geometries = mut.GeometrySet()

        # Confirm that both invocations provide access.
        for dut in (sg.collision_filter_manager(),
                    sg.collision_filter_manager(sg_context)):
            self.assertIsInstance(dut, mut.CollisionFilterManager)

        # We'll test against the Context-variant, assuming that if the API
        # works for an instance from one source, it'll work for both.
        dut = sg.collision_filter_manager(sg_context)
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().ExcludeBetween(
                geometries, geometries))
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().ExcludeWithin(
                geometries))
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().AllowBetween(
                set_A=geometries, set_B=geometries))
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().AllowWithin(
                geometry_set=geometries))

        id = dut.ApplyTransient(
            declaration=mut.CollisionFilterDeclaration().ExcludeWithin(
                geometries))
        self.assertTrue(dut.has_transient_history())
        self.assertTrue(dut.IsActive(filter_id=id))
        self.assertTrue(dut.RemoveDeclaration(filter_id=id))

    def test_geometry_frame_api(self):
        frame = mut.GeometryFrame(frame_name="test_frame")
        self.assertIsInstance(frame.id(), mut.FrameId)
        self.assertEqual(frame.name(), "test_frame")
        frame = mut.GeometryFrame(frame_name="test_frame", frame_group_id=1)
        self.assertEqual(frame.frame_group(), 1)

    def test_geometry_instance_api(self):
        geometry = mut.GeometryInstance(X_PG=RigidTransform(),
                                        shape=mut.Sphere(1.), name="sphere")
        self.assertIsInstance(geometry.id(), mut.GeometryId)
        geometry.set_pose(RigidTransform([1, 0, 0]))
        self.assertIsInstance(geometry.pose(), RigidTransform)
        self.assertIsInstance(geometry.shape(), mut.Shape)
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
        for property_cls in PROPERTY_CLS_LIST:
            props = property_cls()
            props.AddProperty("g", "p", 10)
            self.assertTrue(props.HasProperty("g", "p"))
            props_copy = property_cls(other=props)
            self.assertTrue(props_copy.HasProperty("g", "p"))
            props_copy2 = copy.copy(props)
            self.assertTrue(props_copy2.HasProperty("g", "p"))
            props_copy3 = copy.deepcopy(props)
            self.assertTrue(props_copy3.HasProperty("g", "p"))

    def test_geometry_properties_cpp_types(self):
        """
        Confirms that types stored in properties in python, resolve to expected
        types in C++ (with particular emphasis on python built in types as per
        issue #15640).
        """
        # TODO(sean.curtis): Clean up test, reduce any possible redundancies.
        for property_cls in PROPERTY_CLS_LIST:
            for T in [str, bool, float]:
                props = property_cls()
                value = T()
                props.AddProperty("g", "p", value)
                # Ensure that direct C++ type access is preserved.
                value_2 = mut_testing.GetPropertyCpp[T](props, "g", "p")
                self.assertIsInstance(value_2, T)
                self.assertEqual(value, value_2)

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
        version3 = mut.GeometryVersion()
        self.assertFalse(version0.IsSameAs(other=version3,
                                           role=mut.Role.kProximity))
        self.assertFalse(version0.IsSameAs(other=version3,
                                           role=mut.Role.kPerception))
        self.assertFalse(version0.IsSameAs(other=version3,
                                           role=mut.Role.kIllustration))

    def test_identifier_api(self):
        cls_list = [
            mut.FilterId,
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

        id_1 = mut_testing.get_constant_id()
        id_2 = mut_testing.get_constant_id()
        self.assertIsNot(id_1, id_2)
        self.assertEqual(hash(id_1), hash(id_2))

        self.assertIn(
            f"value={id_1.get_value()}",
            repr(id_1))

    def test_proximity_properties(self):
        """
        Tests the utility functions (not related to hydroelastic contact) for
        setting values in ProximityProperties (as defined in
        proximity_properties.h).
        """
        props = mut.ProximityProperties()
        mut.AddContactMaterial(properties=props)
        props = mut.ProximityProperties()
        reference_friction = CoulombFriction(0.25, 0.125)
        mut.AddContactMaterial(dissipation=2.7,
                               point_stiffness=3.9,
                               friction=reference_friction,
                               properties=props)
        self.assertTrue(
            props.HasProperty("material", "hunt_crossley_dissipation"))
        self.assertEqual(
            props.GetProperty("material", "hunt_crossley_dissipation"), 2.7)
        self.assertTrue(
            props.HasProperty("material", "point_contact_stiffness"))
        self.assertEqual(
            props.GetProperty("material", "point_contact_stiffness"), 3.9)
        self.assertTrue(props.HasProperty("material", "coulomb_friction"))
        stored_friction = props.GetProperty("material", "coulomb_friction")
        self.assertEqual(stored_friction.static_friction(),
                         reference_friction.static_friction())
        self.assertEqual(stored_friction.dynamic_friction(),
                         reference_friction.dynamic_friction())

        props = mut.ProximityProperties()
        res_hint = 0.175
        E = 1e8
        mut.AddRigidHydroelasticProperties(
            resolution_hint=res_hint, properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(props.GetProperty("hydroelastic", "resolution_hint"),
                         res_hint)

        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertFalse(props.HasProperty("hydroelastic", "resolution_hint"))

        props = mut.ProximityProperties()
        res_hint = 0.275
        mut.AddCompliantHydroelasticProperties(
            resolution_hint=res_hint, hydroelastic_modulus=E, properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(props.GetProperty("hydroelastic", "resolution_hint"),
                         res_hint)
        self.assertTrue(props.HasProperty("hydroelastic",
                                          "hydroelastic_modulus"))
        self.assertEqual(props.GetProperty("hydroelastic",
                                           "hydroelastic_modulus"), E)

        props = mut.ProximityProperties()
        slab_thickness = 0.275
        mut.AddCompliantHydroelasticPropertiesForHalfSpace(
            slab_thickness=slab_thickness, hydroelastic_modulus=E,
            properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "slab_thickness"))
        self.assertEqual(props.GetProperty("hydroelastic", "slab_thickness"),
                         slab_thickness)
        self.assertTrue(props.HasProperty("hydroelastic",
                                          "hydroelastic_modulus"))
        self.assertEqual(props.GetProperty("hydroelastic",
                                           "hydroelastic_modulus"), E)

    def test_rgba_api(self):
        default_white = mut.Rgba()
        self.assertEqual(default_white, mut.Rgba(1, 1, 1, 1))
        r, g, b, a = 0.75, 0.5, 0.25, 1.0
        color = mut.Rgba(r=r, g=g, b=b)
        self.assertEqual(color.r(), r)
        self.assertEqual(color.g(), g)
        self.assertEqual(color.b(), b)
        self.assertEqual(color.a(), a)
        self.assertEqual(color, mut.Rgba(r, g, b, a))
        self.assertNotEqual(color, mut.Rgba(r, g, b, 0.0))
        self.assertEqual(
            repr(color),
            "Rgba(r=0.75, g=0.5, b=0.25, a=1.0)")
        color.set(r=1.0, g=1.0, b=1.0, a=0.0)
        self.assertEqual(color, mut.Rgba(1.0, 1.0, 1.0, 0.0))
        color.set(rgba=[0.75, 0.5, 0.25])
        self.assertEqual(color, mut.Rgba(0.75, 0.5, 0.25, 1.0))
        color.update(a=0.5)
        self.assertEqual(color, mut.Rgba(0.75, 0.5, 0.25, 0.5))
        color.update(r=0.1, g=0.2, b=0.3)
        self.assertEqual(color, mut.Rgba(0.1, 0.2, 0.3, 0.5))

        # Property read/write.
        color.rgba = [0.1, 0.2, 0.3, 0.4]
        self.assertEqual(color.r(), 0.1)
        self.assertEqual(color.g(), 0.2)
        self.assertEqual(color.b(), 0.3)
        self.assertEqual(color.a(), 0.4)
        color.rgba = [0.5, 0.6, 0.7]
        self.assertEqual(color.r(), 0.5)
        self.assertEqual(color.g(), 0.6)
        self.assertEqual(color.b(), 0.7)
        self.assertEqual(color.a(), 1.0)
        self.assertEqual(color.rgba[0], 0.5)
        self.assertEqual(color.rgba[1], 0.6)
        self.assertEqual(color.rgba[2], 0.7)
        self.assertEqual(color.rgba[3], 1.0)
        with self.assertRaisesRegex(RuntimeError, ".*range.*"):
            color.rgba = [-1.0] * 4
        with self.assertRaisesRegex(RuntimeError, ".*3 or 4.*"):
            color.rgba = [1.0] * 2
        with self.assertRaisesRegex(RuntimeError, ".*3 or 4.*"):
            color.rgba = [1.0] * 5

        # Modulation.
        self.assertIsInstance(color * mut.Rgba(0.5, 0.5, 0.5), mut.Rgba)
        self.assertIsInstance(color.scale_rgb(0.5), mut.Rgba)

        # Confirm value instantiation.
        Value[mut.Rgba]

    def test_rgba_yaml(self):
        yaml = "rgba: [0.1, 0.2, 0.3, 0.4]"
        dut = yaml_load_typed(schema=mut.Rgba, data=yaml)
        self.assertEqual(dut.r(), 0.1)
        self.assertEqual(dut.g(), 0.2)
        self.assertEqual(dut.b(), 0.3)
        self.assertEqual(dut.a(), 0.4)

        yaml = "rgba: [0.1, 0.2, 0.3]"
        dut = yaml_load_typed(schema=mut.Rgba, data=yaml)
        self.assertEqual(dut.r(), 0.1)
        self.assertEqual(dut.g(), 0.2)
        self.assertEqual(dut.b(), 0.3)
        self.assertEqual(dut.a(), 1.0)

        yaml = "rgba: []"
        with self.assertRaisesRegex(RuntimeError, ".*3 or 4.*"):
            yaml_load_typed(schema=mut.Rgba, data=yaml)

        yaml = "rgba: [0, 1, 2, 3, 4, 5]"
        with self.assertRaisesRegex(RuntimeError, ".*3 or 4.*"):
            yaml_load_typed(schema=mut.Rgba, data=yaml)

        yaml = "rgba: [0, 0, 0, -1]"
        with self.assertRaisesRegex(RuntimeError, ".*range.*"):
            yaml_load_typed(schema=mut.Rgba, data=yaml)

    def test_shape_constructors(self):
        shapes = [
            mut.Sphere(radius=1.0),
            mut.Cylinder(radius=1.0, length=2.0),
            mut.Box(width=1.0, depth=2.0, height=3.0),
            mut.Capsule(radius=1.0, length=2.0),
            mut.Ellipsoid(a=1.0, b=2.0, c=3.0),
            mut.HalfSpace(),
            mut.Mesh(filename="arbitrary/path", scale=1.0),
            mut.Convex(filename="arbitrary/path", scale=1.0),
            mut.MeshcatCone(height=1.23, a=3.45, b=6.78)
        ]
        for shape in shapes:
            self.assertIsInstance(shape, mut.Shape)
            shape_cls = type(shape)
            shape_copy = shape.Clone()
            self.assertIsInstance(shape_copy, shape_cls)
            self.assertIsNot(shape, shape_copy)

    def test_shapes(self):
        # We'll test some invariants on all shapes as inherited from the Shape
        # API.
        def assert_shape_api(shape):
            self.assertIsInstance(shape, mut.Shape)
            shape_cls = type(shape)
            shape_copy = shape.Clone()
            self.assertIsInstance(shape_copy, shape_cls)
            self.assertIsNot(shape, shape_copy)

        # Note: these are ordered alphabetical order and not in the declared
        # order in shape_specification.h
        box = mut.Box(width=1.0, depth=2.0, height=3.0)
        assert_shape_api(box)
        box = mut.Box(measures=(1.0, 2.0, 3.0))
        self.assertEqual(box.width(), 1.0)
        self.assertEqual(box.depth(), 2.0)
        self.assertEqual(box.height(), 3.0)
        assert_pickle(
            self, box,
            lambda shape: [shape.width(), shape.depth(), shape.height()])
        numpy_compare.assert_float_equal(box.size(), np.array([1.0, 2.0, 3.0]))
        self.assertAlmostEqual(mut.CalcVolume(box), 6.0, 1e-14)

        capsule = mut.Capsule(radius=1.0, length=2.0)
        assert_shape_api(capsule)
        capsule = mut.Capsule(measures=(1.0, 2.0))
        self.assertEqual(capsule.radius(), 1.0)
        self.assertEqual(capsule.length(), 2.0)
        assert_pickle(
            self, capsule, lambda shape: [shape.radius(), shape.length()])

        junk_path = "arbitrary/path.ext"
        convex = mut.Convex(filename=junk_path, scale=1.0)
        assert_shape_api(convex)
        self.assertIn(junk_path, convex.filename())
        self.assertEqual(".ext", convex.extension())
        self.assertEqual(convex.scale(), 1.0)
        assert_pickle(
            self, convex, lambda shape: [shape.filename(), shape.scale()])

        cylinder = mut.Cylinder(radius=1.0, length=2.0)
        assert_shape_api(cylinder)
        cylinder = mut.Cylinder(measures=(1.0, 2.0))
        self.assertEqual(cylinder.radius(), 1.0)
        self.assertEqual(cylinder.length(), 2.0)
        assert_pickle(
            self, cylinder, lambda shape: [shape.radius(), shape.length()])

        ellipsoid = mut.Ellipsoid(a=1.0, b=2.0, c=3.0)
        assert_shape_api(ellipsoid)
        ellipsoid = mut.Ellipsoid(measures=(1.0, 2.0, 3.0))
        self.assertEqual(ellipsoid.a(), 1.0)
        self.assertEqual(ellipsoid.b(), 2.0)
        self.assertEqual(ellipsoid.c(), 3.0)
        assert_pickle(
            self, ellipsoid, lambda shape: [shape.a(), shape.b(), shape.c()])

        X_FH = mut.HalfSpace.MakePose(Hz_dir_F=[0, 1, 0], p_FB=[1, 1, 1])
        self.assertIsInstance(X_FH, RigidTransform)

        mesh = mut.Mesh(filename=junk_path, scale=1.0)
        assert_shape_api(mesh)
        self.assertIn(junk_path, mesh.filename())
        self.assertEqual(".ext", mesh.extension())
        self.assertEqual(mesh.scale(), 1.0)
        assert_pickle(
            self, mesh, lambda shape: [shape.filename(), shape.scale()])

        sphere = mut.Sphere(radius=1.0)
        assert_shape_api(sphere)
        self.assertEqual(sphere.radius(), 1.0)
        assert_pickle(self, sphere, mut.Sphere.radius)

        cone = mut.MeshcatCone(height=1.2, a=3.4, b=5.6)
        assert_shape_api(cone)
        cone = mut.MeshcatCone(measures=(1.2, 3.4, 5.6))
        self.assertEqual(cone.height(), 1.2)
        self.assertEqual(cone.a(), 3.4)
        self.assertEqual(cone.b(), 5.6)
        assert_pickle(self, cone, lambda shape: [
                      shape.height(), shape.a(), shape.b()])
