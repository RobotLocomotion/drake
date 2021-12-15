import pydrake.geometry as mut
import pydrake.geometry._testing as mut_testing

import copy
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform


class TestGeometryHydro(unittest.TestCase):
    def test_hydro_proximity_properties(self):
        """
        Tests the utility functions (related to hydroelastic contact) for
        setting values in ProximityProperties (as defined in
        proximity_properties.h).
        """
        props = mut.ProximityProperties()
        res_hint = 0.175
        E = 1e8
        mut.AddRigidHydroelasticProperties(
            resolution_hint=res_hint, properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateSoftHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(props.GetProperty("hydroelastic", "resolution_hint"),
                         res_hint)

        props = mut.ProximityProperties()
        mut.AddRigidHydroelasticProperties(properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertFalse(mut_testing.PropertiesIndicateSoftHydro(props))
        self.assertFalse(props.HasProperty("hydroelastic", "resolution_hint"))

        props = mut.ProximityProperties()
        res_hint = 0.275
        mut.AddSoftHydroelasticProperties(
            resolution_hint=res_hint, hydroelastic_modulus=E, properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateSoftHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "resolution_hint"))
        self.assertEqual(props.GetProperty("hydroelastic", "resolution_hint"),
                         res_hint)
        self.assertTrue(props.HasProperty("hydroelastic",
                                          "hydroelastic_modulus"))
        self.assertEqual(props.GetProperty("hydroelastic",
                                           "hydroelastic_modulus"), E)

        props = mut.ProximityProperties()
        slab_thickness = 0.275
        mut.AddSoftHydroelasticPropertiesForHalfSpace(
            slab_thickness=slab_thickness, hydroelastic_modulus=E,
            properties=props)
        self.assertTrue(props.HasProperty("hydroelastic", "compliance_type"))
        self.assertTrue(mut_testing.PropertiesIndicateSoftHydro(props))
        self.assertTrue(props.HasProperty("hydroelastic", "slab_thickness"))
        self.assertEqual(props.GetProperty("hydroelastic", "slab_thickness"),
                         slab_thickness)
        self.assertTrue(props.HasProperty("hydroelastic",
                                          "hydroelastic_modulus"))
        self.assertEqual(props.GetProperty("hydroelastic",
                                           "hydroelastic_modulus"), E)

    def test_polygon_surface_mesh(self):
        # Default constructor.
        mut.PolygonSurfaceMesh()

        # Construct a single triangle.
        vertices = [
            (0, 0, 0),
            (1, 0, 0),
            (0, 1, 0),
        ]
        vertex_indices = [0, 1, 2]
        face_data = [len(vertex_indices)] + vertex_indices
        dut = mut.PolygonSurfaceMesh(face_data=face_data, vertices=vertices)

        # Sanity check every accessor.
        dut.element(e=0)
        dut.vertex(v=0)
        dut.num_vertices()
        dut.num_elements()
        dut.num_faces()
        dut.area(f=0)
        dut.total_area()
        dut.face_normal(f=0)
        dut.element_centroid(e=0)
        dut.centroid()
        dut.CalcBoundingBox()
        dut.Equal(mesh=dut)
        dut.face_data()
        copy.copy(dut)

        # Sanity check the mutators.
        dut.TransformVertices(X_NM=RigidTransform())
        dut.ReverseFaceWinding()

        # Now check the SurfacePolygon bindings.
        polygon = dut.element(e=0)
        self.assertEqual(polygon.num_vertices(), 3)
        self.assertEqual(polygon.vertex(i=1), 1)

    def test_triangle_surface_mesh(self):
        # Create a mesh out of two triangles forming a quad.
        #
        #     0______1
        #      |b  /|      Two triangles: a and b.
        #      |  / |      Four vertices: 0, 1, 2, and 3.
        #      | /a |
        #      |/___|
        #     2      3

        t_a = mut.SurfaceTriangle(v0=3, v1=1, v2=2)
        t_b = mut.SurfaceTriangle(v0=2, v1=1, v2=0)
        self.assertEqual(t_a.vertex(0), 3)
        self.assertEqual(t_b.vertex(1), 1)

        v0 = (-1,  1, 0)
        v1 = (1,  1, 0)
        v2 = (-1, -1, 0)
        v3 = (1, -1, 0)

        self.assertListEqual(list(v0), [-1, 1, 0])

        mesh = mut.TriangleSurfaceMesh(triangles=(t_a, t_b),
                                       vertices=(v0, v1, v2, v3))
        self.assertEqual(len(mesh.triangles()), 2)
        self.assertEqual(len(mesh.vertices()), 4)
        self.assertListEqual(list(mesh.centroid()), [0, 0, 0])
        self.assertListEqual(list(mesh.element_centroid(t=1)),
                             [-1/3.0, 1/3.0, 0])

    def test_volume_mesh(self):
        # Create a mesh out of two tetrahedra with a single, shared face
        # (1, 2, 3).
        #
        #            +y
        #            |
        #            o v2
        #            |
        #       v4   | v1   v0
        #    ───o────o─────o──  +x
        #           /
        #          /
        #         o v3
        #        /
        #      +z

        t_left = mut.VolumeElement(v0=2, v1=1, v2=3, v3=4)
        t_right = mut.VolumeElement(v0=3, v1=1, v2=2, v3=0)
        self.assertEqual(t_left.vertex(0), 2)
        self.assertEqual(t_right.vertex(1), 1)

        v0 = (1, 0,  0)
        v1 = (0, 0,  0)
        v2 = (0, 1,  0)
        v3 = (0, 0, 1)
        v4 = (-1, 0,  0)

        self.assertListEqual(list(v0), [1, 0, 0])

        mesh = mut.VolumeMesh(elements=(t_left, t_right),
                              vertices=(v0, v1, v2, v3, v4))

        self.assertEqual(len(mesh.tetrahedra()), 2)
        self.assertIsInstance(mesh.tetrahedra()[0], mut.VolumeElement)
        self.assertEqual(len(mesh.vertices()), 5)

        self.assertAlmostEqual(
            mesh.CalcTetrahedronVolume(e=1),
            1/6.0,
            delta=1e-15)
        self.assertAlmostEqual(mesh.CalcVolume(), 1/3.0, delta=1e-15)

    def test_convert_volume_to_surface_mesh(self):
        # Use the volume mesh from `test_volume_mesh()`.
        t_left = mut.VolumeElement(v0=1, v1=2, v2=3, v3=4)
        t_right = mut.VolumeElement(v0=1, v1=3, v2=2, v3=0)

        v0 = (1, 0,  0)
        v1 = (0, 0,  0)
        v2 = (0, 1,  0)
        v3 = (0, 0, -1)
        v4 = (-1, 0,  0)

        volume_mesh = mut.VolumeMesh(elements=(t_left, t_right),
                                     vertices=(v0, v1, v2, v3, v4))

        surface_mesh = mut.ConvertVolumeToSurfaceMesh(volume_mesh)

        self.assertIsInstance(surface_mesh, mut.TriangleSurfaceMesh)

    def test_read_obj_to_surface_mesh(self):
        mesh_path = FindResourceOrThrow("drake/geometry/test/quad_cube.obj")
        mesh = mut.ReadObjToTriangleSurfaceMesh(mesh_path)
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
            self.assertListEqual(list(vertices[i]), expected)
