import pydrake.geometry as mut  # ruff: isort: skip
import copy
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities import numpy_compare


class TestGeometryMeshes(unittest.TestCase):
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

        v0 = (-1, 1, 0)
        v1 = (1, 1, 0)
        v2 = (-1, -1, 0)
        v3 = (1, -1, 0)

        self.assertListEqual(list(v0), [-1, 1, 0])

        dut = mut.TriangleSurfaceMesh(
            triangles=(t_a, t_b), vertices=(v0, v1, v2, v3)
        )

        # Sanity check every accessor.
        self.assertIsInstance(dut.element(e=0), mut.SurfaceTriangle)
        self.assertIsInstance(dut.vertex(v=0), np.ndarray)
        self.assertIsInstance(dut.num_vertices(), int)
        self.assertIsInstance(dut.num_elements(), int)
        self.assertIsInstance(dut.num_triangles(), int)
        self.assertIsInstance(dut.area(t=0), float)
        self.assertIsInstance(dut.total_area(), float)
        self.assertIsInstance(dut.face_normal(t=0), np.ndarray)
        self.assertIsInstance(dut.element_centroid(t=0), np.ndarray)
        self.assertIsInstance(dut.centroid(), np.ndarray)

        # Sanity check some calculations
        self.assertIsInstance(dut.CalcBoundingBox(), tuple)
        self.assertTrue(dut.Equal(mesh=dut))
        self.assertIsInstance(
            dut.CalcCartesianFromBarycentric(
                element_index=1, b_Q=[1 / 3.0, 1 / 3.0, 1 / 3.0]
            ),
            np.ndarray,
        )
        self.assertIsInstance(
            dut.CalcBarycentric(p_MQ=[-1 / 3.0, 1 / 3.0, 0], t=1), np.ndarray
        )

        self.assertEqual(len(dut.triangles()), 2)
        self.assertEqual(len(dut.vertices()), 4)
        self.assertListEqual(list(dut.centroid()), [0, 0, 0])
        self.assertListEqual(
            list(dut.element_centroid(t=1)), [-1 / 3.0, 1 / 3.0, 0]
        )

        # Now check the SurfaceTriangle bindings.
        triangle0 = dut.element(e=0)
        self.assertEqual(triangle0.num_vertices(), 3)
        self.assertEqual(triangle0.vertex(i=0), 3)

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

        v0 = (1, 0, 0)
        v1 = (0, 0, 0)
        v2 = (0, 1, 0)
        v3 = (0, 0, 1)
        v4 = (-1, 0, 0)

        self.assertListEqual(list(v0), [1, 0, 0])

        dut = mut.VolumeMesh(
            elements=(t_left, t_right), vertices=(v0, v1, v2, v3, v4)
        )

        # Sanity check every accessor.
        self.assertIsInstance(dut.element(e=0), mut.VolumeElement)
        self.assertIsInstance(dut.vertex(v=0), np.ndarray)
        self.assertIsInstance(dut.num_elements(), int)
        self.assertIsInstance(dut.num_vertices(), int)
        self.assertIsInstance(dut.inward_normal(e=0, f=0), np.ndarray)
        self.assertIsInstance(dut.edge_vector(e=0, a=0, b=1), np.ndarray)

        # Sanity check some calculations
        self.assertAlmostEqual(
            dut.CalcTetrahedronVolume(e=1), 1 / 6.0, delta=1e-15
        )
        self.assertAlmostEqual(dut.CalcVolume(), 1 / 3.0, delta=1e-15)
        self.assertIsInstance(
            dut.CalcBarycentric(p_MQ=[-0.25, 0.25, 0.25], e=0), np.ndarray
        )
        self.assertTrue(dut.Equal(mesh=dut))

        self.assertEqual(len(dut.tetrahedra()), 2)
        self.assertIsInstance(dut.tetrahedra()[0], mut.VolumeElement)
        self.assertEqual(len(dut.vertices()), 5)

        numpy_compare.assert_float_equal(
            dut.inward_normal(e=0, f=3), (-1.0, 0.0, 0.0)
        )
        numpy_compare.assert_float_equal(
            dut.inward_normal(e=1, f=3), (1.0, 0.0, 0.0)
        )
        numpy_compare.assert_float_equal(
            dut.edge_vector(e=0, a=1, b=3), (-1.0, 0.0, 0.0)
        )
        numpy_compare.assert_float_equal(
            dut.edge_vector(e=1, a=1, b=3), (1.0, 0.0, 0.0)
        )

        # Now check the VolumeElement bindings.
        tetrahedron0 = dut.element(e=0)
        self.assertEqual(tetrahedron0.vertex(i=0), 2)
        self.assertEqual(tetrahedron0.num_vertices(), 4)

    def test_convert_volume_to_surface_mesh(self):
        # Use the volume mesh from `test_volume_mesh()`.
        t_left = mut.VolumeElement(v0=1, v1=2, v2=3, v3=4)
        t_right = mut.VolumeElement(v0=1, v1=3, v2=2, v3=0)

        v0 = (1, 0, 0)
        v1 = (0, 0, 0)
        v2 = (0, 1, 0)
        v3 = (0, 0, -1)
        v4 = (-1, 0, 0)

        volume_mesh = mut.VolumeMesh(
            elements=(t_left, t_right), vertices=(v0, v1, v2, v3, v4)
        )

        surface_mesh = mut.ConvertVolumeToSurfaceMesh(volume_mesh)

        self.assertIsInstance(surface_mesh, mut.TriangleSurfaceMesh)

    def test_refine_volume_mesh(self):
        # Create a simple mesh with one tetrahedron that needs refinement.
        t = mut.VolumeElement(v0=0, v1=1, v2=2, v3=3)
        v0 = (0, 0, 0)
        v1 = (1, 0, 0)
        v2 = (0, 1, 0)
        v3 = (0, 0, 1)
        mesh = mut.VolumeMesh(elements=(t,), vertices=(v0, v1, v2, v3))

        # Refine the mesh.
        refined_mesh = mut.RefineVolumeMesh(mesh=mesh)

        # Check that the refined mesh has more vertices and elements.
        self.assertGreater(refined_mesh.num_vertices(), 4)
        self.assertGreater(refined_mesh.num_elements(), 1)

    def test_refine_volume_mesh_no_refinement(self):
        # Create a mesh with four tetrahedra sharing a center vertex.
        t1 = mut.VolumeElement(v0=0, v1=1, v2=2, v3=4)
        t2 = mut.VolumeElement(v0=0, v1=3, v2=1, v3=4)
        t3 = mut.VolumeElement(v0=3, v1=2, v2=1, v3=4)
        t4 = mut.VolumeElement(v0=3, v1=0, v2=2, v3=4)
        v0 = (0, 0, 0)
        v1 = (1, 0, 0)
        v2 = (0, 1, 0)
        v3 = (0, 0, 1)
        v4 = (0.25, 0.25, 0.25)
        mesh = mut.VolumeMesh(
            elements=(t1, t2, t3, t4), vertices=(v0, v1, v2, v3, v4)
        )

        # Refine the mesh.
        refined_mesh = mut.RefineVolumeMesh(mesh=mesh)

        # Verify the refined mesh is identical to the input mesh.
        self.assertTrue(refined_mesh.Equal(mesh))

    def test_refine_volume_mesh_to_string(self):
        # Get a path to a test mesh file.
        mesh_path = FindResourceOrThrow(
            "drake/geometry/test/one_tetrahedron.vtk"
        )

        # Get both the refined mesh and its bytes representation.
        vtk_string = mut.RefineVolumeMeshIntoVtkFileContents(
            mesh_source=mut.MeshSource(mesh_path)
        )

        # Verify the string contains key VTK elements.
        self.assertIn("# vtk DataFile Version 3.0", vtk_string)
        self.assertIn("ASCII", vtk_string)
        self.assertIn("DATASET UNSTRUCTURED_GRID", vtk_string)
        self.assertIn("POINTS", vtk_string)
        self.assertIn("CELLS", vtk_string)
        self.assertIn("CELL_TYPES", vtk_string)

    def test_read_obj_to_surface_mesh(self):
        mesh_path = FindResourceOrThrow("drake/geometry/test/quad_cube.obj")
        # Test default, uniform, and non-uniform scales.
        for scale in [None, 2.0, [1.0, 2.0, 3.0]]:
            if scale is None:
                mesh = mut.ReadObjToTriangleSurfaceMesh(filename=mesh_path)
                scale = [1.0, 1.0, 1.0]
            elif isinstance(scale, float):
                mesh = mut.ReadObjToTriangleSurfaceMesh(
                    filename=mesh_path, scale=scale
                )
                scale = [scale, scale, scale]
            else:
                mesh = mut.ReadObjToTriangleSurfaceMesh(
                    filename=mesh_path, scale3=scale
                )
            vertices = mesh.vertices()

            # This test relies on the specific content of the file
            # quad_cube.obj. These coordinates came from the first section of
            # quad_cube.obj.
            scale = np.array(scale)
            scale.shape = (1, 3)
            expected_vertices = (
                np.array(
                    [
                        [1.000000, -1.000000, -1.000000],
                        [1.000000, -1.000000, 1.000000],
                        [-1.000000, -1.000000, 1.000000],
                        [-1.000000, -1.000000, -1.000000],
                        [1.000000, 1.000000, -1.000000],
                        [1.000000, 1.000000, 1.000000],
                        [-1.000000, 1.000000, 1.000000],
                        [-1.000000, 1.000000, -1.000000],
                    ]
                )
                * scale
            )
            for i, expected in enumerate(expected_vertices):
                np.testing.assert_array_equal(vertices[i], expected)
