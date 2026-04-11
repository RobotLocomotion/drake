import pydrake.geometry as mut  # ruff: isort: skip
import pydrake.geometry._testing as mut_testing  # ruff: isort: skip

import copy
from pathlib import Path
import pickle
import re
import unittest

import numpy as np

from pydrake.common import MemoryFile
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue, Value
from pydrake.common.yaml import yaml_load_typed
from pydrake.math import RigidTransform, RotationMatrix
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
        for dut in (
            sg.collision_filter_manager(),
            sg.collision_filter_manager(sg_context),
        ):
            self.assertIsInstance(dut, mut.CollisionFilterManager)

        # We'll test against the Context-variant, assuming that if the API
        # works for an instance from one source, it'll work for both.
        dut = sg.collision_filter_manager(sg_context)
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().ExcludeBetween(
                geometries, geometries
            )
        )
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().ExcludeWithin(
                geometries
            )
        )
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().AllowBetween(
                set_A=geometries, set_B=geometries
            )
        )
        dut.Apply(
            declaration=mut.CollisionFilterDeclaration().AllowWithin(
                geometry_set=geometries
            )
        )

        id = dut.ApplyTransient(
            declaration=mut.CollisionFilterDeclaration().ExcludeWithin(
                geometries
            )
        )
        self.assertTrue(dut.has_transient_history())
        self.assertTrue(dut.IsActive(filter_id=id))
        self.assertTrue(dut.RemoveDeclaration(filter_id=id))

        # Test CollisionFilterScope enum.
        self.assertIsInstance(
            mut.CollisionFilterScope.kAll, mut.CollisionFilterScope
        )
        self.assertIsInstance(
            mut.CollisionFilterScope.kOmitDeformable, mut.CollisionFilterScope
        )

        # Test CollisionFilterDeclaration constructor with scope parameter.
        declaration_all = mut.CollisionFilterDeclaration(
            scope=mut.CollisionFilterScope.kAll
        )
        id = dut.ApplyTransient(
            declaration=declaration_all.ExcludeWithin(geometries)
        )
        self.assertTrue(dut.IsActive(filter_id=id))

        declaration_omit = mut.CollisionFilterDeclaration(
            scope=mut.CollisionFilterScope.kOmitDeformable
        )
        id = dut.ApplyTransient(
            declaration=declaration_omit.ExcludeWithin(geometries)
        )
        self.assertTrue(dut.IsActive(filter_id=id))

    def test_geometry_frame_api(self):
        frame = mut.GeometryFrame(frame_name="test_frame")
        self.assertIsInstance(frame.id(), mut.FrameId)
        self.assertEqual(frame.name(), "test_frame")
        frame = mut.GeometryFrame(frame_name="test_frame", frame_group_id=1)
        self.assertEqual(frame.frame_group(), 1)

    def test_geometry_instance_api(self):
        geometry = mut.GeometryInstance(
            X_PG=RigidTransform(), shape=mut.Sphere(1.0), name="sphere"
        )
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
        self.assertIsInstance(
            geometry.mutable_proximity_properties(), mut.ProximityProperties
        )
        self.assertIsInstance(
            geometry.proximity_properties(), mut.ProximityProperties
        )
        self.assertIsInstance(
            geometry.mutable_illustration_properties(),
            mut.IllustrationProperties,
        )
        self.assertIsInstance(
            geometry.illustration_properties(), mut.IllustrationProperties
        )
        self.assertIsInstance(
            geometry.mutable_perception_properties(), mut.PerceptionProperties
        )
        self.assertIsInstance(
            geometry.perception_properties(), mut.PerceptionProperties
        )

    def test_geometry_properties_api(self):
        # Test perception/ illustration properties (specifically Rgba).
        test_vector = [0.0, 0.0, 1.0, 1.0]
        test_color = mut.Rgba(0.0, 0.0, 1.0, 1.0)
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
        self.assertTrue(prop.HasProperty(group_name=default_group, name="test"))
        self.assertEqual(
            prop.GetProperty(group_name=default_group, name="test"), 3
        )
        self.assertEqual(
            prop.GetPropertyOrDefault(
                group_name=default_group, name="empty", default_value=5
            ),
            5,
        )
        group_values = prop.GetPropertiesInGroup(group_name=default_group)
        for name, value in group_values.items():
            self.assertIsInstance(name, str)
            self.assertIsInstance(value, AbstractValue)
        # Remove the property.
        self.assertTrue(
            prop.RemoveProperty(group_name=default_group, name="test")
        )
        self.assertFalse(
            prop.HasProperty(group_name=default_group, name="test")
        )
        # Update a property.
        prop.AddProperty(group_name=default_group, name="to_update", value=17)
        self.assertTrue(
            prop.HasProperty(group_name=default_group, name="to_update")
        )
        self.assertEqual(
            prop.GetProperty(group_name=default_group, name="to_update"), 17
        )

        prop.UpdateProperty(
            group_name=default_group, name="to_update", value=20
        )
        self.assertTrue(
            prop.HasProperty(group_name=default_group, name="to_update")
        )
        self.assertEqual(
            prop.GetProperty(group_name=default_group, name="to_update"), 20
        )

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

        # Cross-property-set copying. We don't do all cross possibilities.
        # Merely confirm that each set can be copied from another set.
        source = mut.PerceptionProperties()
        source.AddProperty("a", "b", 10)
        illustration = mut.IllustrationProperties(source)
        self.assertEqual(illustration.GetProperty("a", "b"), 10)
        proximity = mut.ProximityProperties(illustration)
        self.assertEqual(proximity.GetProperty("a", "b"), 10)
        perception = mut.PerceptionProperties(proximity)
        self.assertEqual(perception.GetProperty("a", "b"), 10)

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
        self.assertTrue(
            version0.IsSameAs(other=version1, role=mut.Role.kProximity)
        )
        self.assertTrue(
            version0.IsSameAs(other=version1, role=mut.Role.kPerception)
        )
        self.assertTrue(
            version0.IsSameAs(other=version1, role=mut.Role.kIllustration)
        )
        version2 = mut.GeometryVersion(other=version0)
        self.assertTrue(
            version0.IsSameAs(other=version2, role=mut.Role.kProximity)
        )
        self.assertTrue(
            version0.IsSameAs(other=version2, role=mut.Role.kPerception)
        )
        self.assertTrue(
            version0.IsSameAs(other=version2, role=mut.Role.kIllustration)
        )
        version3 = mut.GeometryVersion()
        self.assertFalse(
            version0.IsSameAs(other=version3, role=mut.Role.kProximity)
        )
        self.assertFalse(
            version0.IsSameAs(other=version3, role=mut.Role.kPerception)
        )
        self.assertFalse(
            version0.IsSameAs(other=version3, role=mut.Role.kIllustration)
        )

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

        self.assertIn(f"value={id_1.get_value()}", repr(id_1))

    def test_in_memory_mesh(self):
        empty_mesh = mut.InMemoryMesh()
        self.assertEqual(len(empty_mesh.mesh_file.contents()), 0)

        file = MemoryFile(
            contents="stuff", extension=".ext", filename_hint="some_hint"
        )
        only_mesh = mut.InMemoryMesh(mesh_file=file)
        self.assertEqual(only_mesh.mesh_file.contents(), file.contents())
        self.assertEqual(len(only_mesh.supporting_files), 0)

        representation = repr(only_mesh)
        # repr correctness is determined in two ways:
        #   - It can be eval'd back into an instance. This only works because
        #     the contents length is below MemoryFile's hard-coded limit
        #     on creating a perfect representation.
        #   - the repr'd string has expected values.
        self.assertIsInstance(
            eval(
                representation,
                {"InMemoryMesh": mut.InMemoryMesh, "MemoryFile": MemoryFile},
            ),
            mut.InMemoryMesh,
        )
        self.assertRegex(
            representation, re.compile("mesh_file=MemoryFile.+stuff", re.DOTALL)
        )
        self.assertNotIn("supporting_files=", representation)

        copy.copy(only_mesh)
        copy.deepcopy(only_mesh)

        assert_pickle(self, only_mesh, repr)
        # Check that data pickled as InMemoryMesh in Drake v1.34.0 can be
        # unpickled in newer versions. The data should produce a InMemoryMesh
        # identical to `only_mesh` above.
        legacy_data = b"\x80\x04\x95\xa2\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c\x0cInMemoryMesh\x94\x93\x94)\x81\x94}\x94\x8c\tmesh_file\x94\x8c\x0epydrake.common\x94\x8c\nMemoryFile\x94\x93\x94)\x81\x94}\x94(\x8c\x08contents\x94\x8c\x05stuff\x94\x8c\textension\x94\x8c\x04.ext\x94\x8c\rfilename_hint\x94\x8c\tsome_hint\x94ubsb."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.InMemoryMesh)
        self.assertEqual(
            obj.mesh_file.contents(), only_mesh.mesh_file.contents()
        )

        supporting_files = {
            "file": MemoryFile(contents="a", extension=".a", filename_hint="a")
        }
        full_mesh = mut.InMemoryMesh(
            mesh_file=file, supporting_files=supporting_files
        )
        self.assertEqual(full_mesh.mesh_file.contents(), file.contents())
        self.assertIn("file", full_mesh.supporting_files)
        self.assertNotIn("c", full_mesh.supporting_files)

        representation = repr(full_mesh)
        self.assertIsInstance(
            eval(
                representation,
                {"InMemoryMesh": mut.InMemoryMesh, "MemoryFile": MemoryFile},
            ),
            mut.InMemoryMesh,
        )
        self.assertRegex(
            representation, re.compile("mesh_file=MemoryFile.*stuff", re.DOTALL)
        )
        self.assertRegex(
            representation, re.compile("supporting_files=.*\\.a", re.DOTALL)
        )

        copy.copy(full_mesh)
        copy.deepcopy(full_mesh)

        assert_pickle(self, full_mesh, repr)
        # Check that data pickled as InMemoryMesh in Drake v1.34.0 can be
        # unpickled in newer versions. The data should produce a InMemoryMesh
        # identical to `only_mesh` above.
        legacy_data = b"\x80\x04\x95\xfc\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c\x0cInMemoryMesh\x94\x93\x94)\x81\x94}\x94(\x8c\tmesh_file\x94\x8c\x0epydrake.common\x94\x8c\nMemoryFile\x94\x93\x94)\x81\x94}\x94(\x8c\x08contents\x94\x8c\x05stuff\x94\x8c\textension\x94\x8c\x04.ext\x94\x8c\rfilename_hint\x94\x8c\tsome_hint\x94ub\x8c\x10supporting_files\x94}\x94\x8c\x04file\x94h\x08)\x81\x94}\x94(\x8c\x08contents\x94\x8c\x01a\x94\x8c\textension\x94\x8c\x02.a\x94\x8c\rfilename_hint\x94h\x17ubsub."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.InMemoryMesh)
        self.assertEqual(
            obj.mesh_file.contents(), full_mesh.mesh_file.contents()
        )
        self.assertIn("file", obj.supporting_files)

    def test_mesh_source(self):
        source = mut.MeshSource(path="/a/path.obj")
        self.assertTrue(source.is_path())
        self.assertFalse(source.is_in_memory())
        self.assertEqual(source.description(), "/a/path.obj")
        self.assertEqual(source.extension(), ".obj")
        self.assertEqual(source.path(), Path("/a/path.obj"))
        with self.assertRaises(RuntimeError):
            source.in_memory()
        # repr correctness is determined the same as for InMemoryMesh (with the
        # same caveats).
        self.assertIsInstance(
            eval(repr(source), {"MeshSource": mut.MeshSource}), mut.MeshSource
        )
        self.assertRegex(repr(source), "path=['\"]/a/path.obj['\"]")
        copy.copy(source)
        copy.deepcopy(source)
        source_copy = mut.MeshSource(other=source)
        self.assertTrue(source_copy.is_path())
        self.assertEqual(source_copy.description(), "/a/path.obj")

        assert_pickle(self, source, repr)
        # Check that data pickled as MeshSource in Drake v1.34.0 can be
        # unpickled in newer versions. The data should produce a MeshSource
        # identical to `source` above. We'll do it for one with a path source
        # and once with an in-memory source (below).
        legacy_data = b"\x80\x04\x95?\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c\nMeshSource\x94\x93\x94)\x81\x94}\x94\x8c\x04path\x94\x8c\x0b/a/path.obj\x94sb."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.MeshSource)
        self.assertEqual(obj.is_path(), source.is_path())
        self.assertEqual(obj.path(), source.path())

        mesh = mut.InMemoryMesh(mesh_file=MemoryFile("a", ".ext", "hint"))
        source = mut.MeshSource(mesh=mesh)
        self.assertFalse(source.is_path())
        self.assertTrue(source.is_in_memory())
        self.assertEqual(source.description(), "hint")
        self.assertEqual(source.extension(), ".ext")
        self.assertIsInstance(source.in_memory(), mut.InMemoryMesh)
        with self.assertRaises(RuntimeError):
            source.path()
        self.assertIsInstance(
            eval(
                repr(source),
                {
                    "MeshSource": mut.MeshSource,
                    "InMemoryMesh": mut.InMemoryMesh,
                    "MemoryFile": MemoryFile,
                },
            ),
            mut.MeshSource,
        )
        self.assertRegex(
            repr(source), re.compile("mesh=InMemoryMesh.*hint.*", re.DOTALL)
        )
        copy.copy(source)
        copy.deepcopy(source)

        # Again for a source with an in-memory mesh.
        assert_pickle(self, source, repr)
        legacy_data = b"\x80\x04\x95\xb8\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c\nMeshSource\x94\x93\x94)\x81\x94}\x94\x8c\x04mesh\x94h\x00\x8c\x0cInMemoryMesh\x94\x93\x94)\x81\x94}\x94\x8c\tmesh_file\x94\x8c\x0epydrake.common\x94\x8c\nMemoryFile\x94\x93\x94)\x81\x94}\x94(\x8c\x08contents\x94\x8c\x01a\x94\x8c\textension\x94\x8c\x04.ext\x94\x8c\rfilename_hint\x94\x8c\x04hint\x94ubsbsb."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.MeshSource)
        self.assertEqual(obj.is_in_memory(), source.is_in_memory())
        self.assertEqual(
            obj.in_memory().mesh_file.contents(),
            source.in_memory().mesh_file.contents(),
        )

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
        mut.AddContactMaterial(
            dissipation=2.7,
            point_stiffness=3.9,
            friction=reference_friction,
            properties=props,
        )
        self.assertTrue(
            props.HasProperty("material", "hunt_crossley_dissipation")
        )
        self.assertEqual(
            props.GetProperty("material", "hunt_crossley_dissipation"), 2.7
        )
        self.assertTrue(
            props.HasProperty("material", "point_contact_stiffness")
        )
        self.assertEqual(
            props.GetProperty("material", "point_contact_stiffness"), 3.9
        )
        self.assertTrue(props.HasProperty("material", "coulomb_friction"))
        stored_friction = props.GetProperty("material", "coulomb_friction")
        self.assertEqual(
            stored_friction.static_friction(),
            reference_friction.static_friction(),
        )
        self.assertEqual(
            stored_friction.dynamic_friction(),
            reference_friction.dynamic_friction(),
        )

        props = mut.ProximityProperties()
        res_hint = 0.175
        E = 1e8
        mut.AddRigidHydroelasticProperties(
            resolution_hint=res_hint, properties=props
        )
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(
            props.GetProperty("hydroelastic", "resolution_hint"), res_hint
        )

        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertFalse(props.HasProperty("hydroelastic", "resolution_hint"))

        props = mut.ProximityProperties()
        res_hint = 0.275
        mut.AddCompliantHydroelasticProperties(
            resolution_hint=res_hint, hydroelastic_modulus=E, properties=props
        )
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(
            props.GetProperty("hydroelastic", "resolution_hint"), res_hint
        )
        self.assertTrue(
            props.HasProperty("hydroelastic", "hydroelastic_modulus")
        )
        self.assertEqual(
            props.GetProperty("hydroelastic", "hydroelastic_modulus"), E
        )

        props = mut.ProximityProperties()
        slab_thickness = 0.275
        mut.AddCompliantHydroelasticPropertiesForHalfSpace(
            slab_thickness=slab_thickness,
            hydroelastic_modulus=E,
            properties=props,
        )
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateCompliantHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "slab_thickness"))
        self.assertEqual(
            props.GetProperty("hydroelastic", "slab_thickness"), slab_thickness
        )
        self.assertTrue(
            props.HasProperty("hydroelastic", "hydroelastic_modulus")
        )
        self.assertEqual(
            props.GetProperty("hydroelastic", "hydroelastic_modulus"), E
        )

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
        self.assertEqual(repr(color), "Rgba(r=0.75, g=0.5, b=0.25, a=1.0)")
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
            mut.Mesh(filename="arbitrary/path", scale3=[1.0, 2.0, 3.0]),
            mut.Mesh(
                mesh_data=mut.InMemoryMesh(
                    mesh_file=MemoryFile("# ", ".obj", "junk")
                ),
                scale=1.0,
            ),
            mut.Mesh(
                mesh_data=mut.InMemoryMesh(
                    mesh_file=MemoryFile("# ", ".obj", "junk")
                ),
                scale3=[1.0, 2.0, 3.0],
            ),
            mut.Convex(filename="arbitrary/path", scale=1.0),
            mut.Convex(filename="arbitrary/path", scale3=[1.0, 2.0, 3.0]),
            mut.Convex(
                mesh_data=mut.InMemoryMesh(
                    mesh_file=MemoryFile("# ", ".obj", "junk")
                ),
                scale=1.0,
            ),
            mut.Convex(
                mesh_data=mut.InMemoryMesh(
                    mesh_file=MemoryFile("# ", ".obj", "junk")
                ),
                scale3=[1.0, 2.0, 3.0],
            ),
            mut.MeshcatCone(height=1.23, a=3.45, b=6.78),
        ]
        for shape in shapes:
            self.assertIsInstance(shape, mut.Shape)
            shape_cls = type(shape)
            shape_cls_name = shape_cls.__name__

            shape_clone = shape.Clone()
            self.assertIsInstance(shape_clone, shape_cls)
            self.assertIsNot(shape_clone, shape)

            shape_copy = copy.deepcopy(shape)
            self.assertIsInstance(shape_copy, shape_cls)
            self.assertIsNot(shape_copy, shape)

            # Representation of Mesh/Convex requires additional types.
            new_shape = eval(
                repr(shape),
                {
                    shape_cls_name: shape_cls,
                    "InMemoryMesh": mut.InMemoryMesh,
                    "MemoryFile": MemoryFile,
                },
            )
            self.assertIsInstance(new_shape, shape_cls)
            self.assertEqual(repr(new_shape), repr(shape))

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
        assert_pickle(self, box, repr)
        numpy_compare.assert_float_equal(box.size(), np.array([1.0, 2.0, 3.0]))
        self.assertAlmostEqual(mut.CalcVolume(box), 6.0, 1e-14)

        capsule = mut.Capsule(radius=1.0, length=2.0)
        assert_shape_api(capsule)
        capsule = mut.Capsule(measures=(1.0, 2.0))
        self.assertEqual(capsule.radius(), 1.0)
        self.assertEqual(capsule.length(), 2.0)
        assert_pickle(self, capsule, repr)

        # Note: Convex has generally been rolled in with Mesh because of their
        # common APIs (below). This test covers the Convex-only constructor
        # from point cloud (which gets converted to an in-memory .obj).

        # Throw away Convex; we just want to make sure the scalar-valued
        # `scale` parameter is bound.
        convex = mut.Convex(
            points=np.array(
                (
                    (0, 0, 0),  # BR
                    (1, 0, 0),
                    (0, 1, 0),
                    (0, 0, 1),
                )
            ).T,
            label="test_label",
            scale=2,
        )

        # For the test, we'll test the non-uniform scale API; the two are
        # otherwise equivalent.
        convex = mut.Convex(
            points=np.array(
                (
                    (0, 0, 0),  # BR
                    (1, 0, 0),
                    (0, 1, 0),
                    (0, 0, 1),
                )
            ).T,
            label="test_label",
            scale3=[1, 2, 3],
        )
        self.assertEqual(".obj", convex.extension())
        np.testing.assert_array_equal(convex.scale3(), [1, 2, 3])
        self.assertTrue(convex.source().is_in_memory())
        convex_file = convex.source().in_memory().mesh_file
        self.assertTrue(convex_file.filename_hint(), "test_label")
        self.assertTrue(convex_file.contents().startswith(b"v 0 0 0"))

        cylinder = mut.Cylinder(radius=1.0, length=2.0)
        assert_shape_api(cylinder)
        cylinder = mut.Cylinder(measures=(1.0, 2.0))
        self.assertEqual(cylinder.radius(), 1.0)
        self.assertEqual(cylinder.length(), 2.0)
        assert_pickle(self, cylinder, repr)

        ellipsoid = mut.Ellipsoid(a=1.0, b=2.0, c=3.0)
        assert_shape_api(ellipsoid)
        ellipsoid = mut.Ellipsoid(measures=(1.0, 2.0, 3.0))
        self.assertEqual(ellipsoid.a(), 1.0)
        self.assertEqual(ellipsoid.b(), 2.0)
        self.assertEqual(ellipsoid.c(), 3.0)
        assert_pickle(self, ellipsoid, repr)

        X_FH = mut.HalfSpace.MakePose(Hz_dir_F=[0, 1, 0], p_FB=[1, 1, 1])
        self.assertIsInstance(X_FH, RigidTransform)

        junk_path = "arbitrary/path.ext"
        for dut_mesh in [
            mut.Mesh(filename=junk_path, scale=1.5),
            mut.Mesh(
                mesh_data=mut.InMemoryMesh(
                    mesh_file=MemoryFile("#junk", ".ext", "test")
                ),
                scale=1.5,
            ),
            mut.Mesh(source=mut.MeshSource(path=junk_path), scale=1.5),
            mut.Convex(filename=junk_path, scale=1.5),
            mut.Convex(
                mesh_data=mut.InMemoryMesh(
                    mesh_file=MemoryFile("#junk", ".ext", "test")
                ),
                scale=1.5,
            ),
            mut.Convex(source=mut.MeshSource(path=junk_path), scale=1.5),
        ]:
            assert_shape_api(dut_mesh)
            self.assertEqual(".ext", dut_mesh.extension())
            self.assertEqual(dut_mesh.scale(), 1.5)
            np.testing.assert_array_equal(dut_mesh.scale3(), [1.5, 1.5, 1.5])
            self.assertIsInstance(dut_mesh.source(), mut.MeshSource)
            with self.assertRaisesRegex(
                RuntimeError, "MakeConvexHull only applies to"
            ):
                # We just need evidence that it invokes convex hull
                # machinery; the exception for a bad extension suffices.
                dut_mesh.GetConvexHull()
            assert_pickle(self, dut_mesh, repr)

        sphere = mut.Sphere(radius=1.0)
        assert_shape_api(sphere)
        self.assertEqual(sphere.radius(), 1.0)
        assert_pickle(self, sphere, repr)

        cone = mut.MeshcatCone(height=1.2, a=3.4, b=5.6)
        assert_shape_api(cone)
        cone = mut.MeshcatCone(measures=(1.2, 3.4, 5.6))
        self.assertEqual(cone.height(), 1.2)
        self.assertEqual(cone.a(), 3.4)
        self.assertEqual(cone.b(), 5.6)
        assert_pickle(self, cone, repr)

    def test_mesh_pickle_compatibility(self):
        """Mesh/Convex have changed since their original pickling (underlying
        representation to MeshSource, scale going from scalar to vector). The
        pickling functions have changed accordingly, but we want to maintain
        compatibility with older pickled meshes.
        In this case, we have example pickled bytestrings from the last Drake
        release prior to a change. Make sure they still get unpickled.
        """
        # Check that data pickled in older versions can be unpickled into the
        # current version. The data should produce a Mesh/Convex equivalent to
        # the reference instance.
        for mesh_type, pickle_str in (
            (mut.Mesh, b"\x04Mesh"),
            (mut.Convex, b"\x06Convex"),
        ):
            ref_mesh = mesh_type(filename="/path/to/file.obj", scale=2)
            current_data = pickle.dumps(ref_mesh)
            v1_33_data = (
                b"\x80\x04\x95@\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c"  # noqa
                + pickle_str
                + b"\x94\x93\x94)\x81\x94\x8c\x11/path/to/file.obj\x94G@\x00\x00\x00\x00\x00\x00\x00\x86\x94b."  # noqa
            )
            v1_39_data = (
                b"\x80\x04\x95\x83\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c"  # noqa
                + pickle_str
                + b"\x94\x93\x94)\x81\x94h\x00\x8c\nMeshSource\x94\x93\x94)\x81\x94}\x94\x8c\x04path\x94\x8c\x07pathlib\x94\x8c\tPosixPath\x94\x93\x94(\x8c\x01/\x94\x8c\x04path\x94\x8c\x02to\x94\x8c\x08file.obj\x94t\x94R\x94sbG@\x00\x00\x00\x00\x00\x00\x00\x86\x94b."  # noqa
            )
            for legacy_data in (v1_33_data, v1_39_data):
                # Confirm legacy pickled data *is* different.
                self.assertNotEqual(legacy_data, current_data)
                obj = pickle.loads(legacy_data)
                self.assertIsInstance(obj, mesh_type)
                self.assertTrue(obj.source().is_path())
                self.assertEqual(obj.source().path(), ref_mesh.source().path())
                np.testing.assert_array_equal(obj.scale3(), ref_mesh.scale3())
                # Safe to call scale() because old pickling only supported
                # uniform scale.
                self.assertEqual(obj.scale(), ref_mesh.scale())

    def test_plane(self):
        normal = np.array([0.0, 0.0, 1.0])
        point_on_plane = np.array([0.0, 0.0, 2.0])
        plane = mut.Plane(normal=normal, point_on_plane=point_on_plane)
        plane = mut.Plane(
            normal=normal,
            point_on_plane=point_on_plane,
            already_normalized=True,
        )

        # Test CalcHeight.
        point_above = np.array([1.0, 1.0, 5.0])
        height_above = plane.CalcHeight(point=point_above)
        numpy_compare.assert_equal(height_above, 3.0)

        point_below = np.array([-1.0, -1.0, -1.0])
        height_below = plane.CalcHeight(point=point_below)
        numpy_compare.assert_equal(height_below, -3.0)

        plane.BoxOverlaps(
            half_width=np.array((1.0, 2.0, 3.0)),
            box_center_in_plane=np.array((0.5, 0.25, 1.0)),
            box_orientation_in_plane=RotationMatrix.Identity(),
        )

        # Test copy and pickle.
        copy.copy(plane)
        assert_pickle(
            self,
            plane,
            lambda x: (
                f"Plane({plane.unit_normal()!r}, {plane.reference_point()!r})"
            ),
        )
        # Pickle regression. The given bytestring represents a valid pickling
        # from the initial implementation. We want to ensure that the pickling
        # doesn't change, breaking old pickled values.
        pickle0 = b"\x80\x04\x95\xf9\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c\x05Plane\x94\x93\x94)\x81\x94\x8c\x15numpy.core.multiarray\x94\x8c\x0c_reconstruct\x94\x93\x94\x8c\x05numpy\x94\x8c\x07ndarray\x94\x93\x94K\x00\x85\x94C\x01b\x94\x87\x94R\x94(K\x01K\x03\x85\x94h\x07\x8c\x05dtype\x94\x93\x94\x8c\x02f8\x94\x89\x88\x87\x94R\x94(K\x03\x8c\x01<\x94NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b\x89C\x18\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf0?\x94t\x94bh\x06h\tK\x00\x85\x94h\x0b\x87\x94R\x94(K\x01K\x03\x85\x94h\x13\x89C\x18\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00@\x94t\x94b\x86\x94b."  # noqa
        unpickled_plane0 = pickle.loads(pickle0)
        numpy_compare.assert_equal(
            unpickled_plane0.unit_normal(), plane.unit_normal()
        )
        numpy_compare.assert_equal(
            unpickled_plane0.reference_point(), plane.reference_point()
        )
