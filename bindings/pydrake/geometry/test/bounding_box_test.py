import pydrake.geometry as mut  # ruff: isort: skip

import copy
import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.math import RigidTransform


class TestGeometryBoundingBox(unittest.TestCase):
    def test_aabb_api(self):
        # Test Aabb construction and basic API.
        center = np.array([1.0, 2.0, 3.0])
        half_width = np.array([0.5, 1.0, 1.5])

        aabb = mut.Aabb(p_HoBo=center, half_width=half_width)

        # Test getters.
        numpy_compare.assert_float_equal(aabb.center(), center)
        numpy_compare.assert_float_equal(aabb.half_width(), half_width)
        numpy_compare.assert_float_equal(aabb.lower(), center - half_width)
        numpy_compare.assert_float_equal(aabb.upper(), center + half_width)

        # Test pose (should be translation-only rigid transform).
        pose = aabb.pose()
        self.assertIsInstance(pose, RigidTransform)
        numpy_compare.assert_float_equal(pose.translation(), center)
        numpy_compare.assert_float_equal(pose.rotation().matrix(), np.eye(3))

        # Test volume calculation.
        expected_volume = 8 * half_width[0] * half_width[1] * half_width[2]
        self.assertAlmostEqual(aabb.CalcVolume(), expected_volume, 1e-13)

        # Test equality.
        aabb2 = mut.Aabb(p_HoBo=center, half_width=half_width)
        self.assertTrue(aabb.Equal(aabb2))

        aabb3 = mut.Aabb(p_HoBo=center + 0.1, half_width=half_width)
        self.assertFalse(aabb.Equal(aabb3))

        # Test copy and deepcopy.
        aabb_copy = copy.copy(aabb)
        self.assertTrue(aabb.Equal(aabb_copy))
        aabb_deepcopy = copy.deepcopy(aabb)
        self.assertTrue(aabb.Equal(aabb_deepcopy))

        # Test pickle.
        assert_pickle(
            self, aabb, lambda x: f"Aabb({x.center()}, {x.half_width()})"
        )

    def test_aabb_overlap(self):
        # Test HasOverlap static methods.
        center1 = np.array([0.0, 0.0, 0.0])
        half_width1 = np.array([1.0, 1.0, 1.0])
        aabb1 = mut.Aabb(p_HoBo=center1, half_width=half_width1)

        # Non-overlapping box.
        center2 = np.array([3.0, 0.0, 0.0])
        aabb2 = mut.Aabb(p_HoBo=center2, half_width=half_width1)
        identity_transform = RigidTransform()
        self.assertFalse(mut.Aabb.HasOverlap(aabb1, aabb2, identity_transform))

        # Test with transform.
        transform = RigidTransform(p=[-2.0, 0.0, 0.0])
        self.assertTrue(mut.Aabb.HasOverlap(aabb1, aabb2, transform))

        # Test Aabb-HalfSpace overlap.
        half_space = mut.HalfSpace()
        # Default half space has normal [0,0,1] and passes through origin.
        # aabb1 is centered at origin with half_width [1,1,1], so it
        # should overlap the half space (extends into negative z).
        self.assertTrue(
            mut.Aabb.HasOverlap(aabb1, half_space, identity_transform)
        )

        # Test Aabb-Plane overlap.
        plane_normal = np.array([0.0, 0.0, 1.0])
        point_on_plane = np.array([0.0, 0.0, 0.5])
        plane = mut.Plane(plane_normal, point_on_plane)
        self.assertTrue(mut.Aabb.HasOverlap(aabb1, plane, identity_transform))

    def test_obb_api(self):
        # Test Obb construction and basic API.
        pose = RigidTransform(p=[1.0, 2.0, 3.0])
        half_width = np.array([0.5, 1.0, 1.5])

        obb = mut.Obb(X_HB=pose, half_width=half_width)

        # Test getters.
        numpy_compare.assert_float_equal(obb.center(), pose.translation())
        numpy_compare.assert_float_allclose(
            obb.half_width(), half_width, atol=1e-13
        )

        # Test pose.
        returned_pose = obb.pose()
        self.assertIsInstance(returned_pose, RigidTransform)
        numpy_compare.assert_float_equal(
            returned_pose.translation(), pose.translation()
        )
        numpy_compare.assert_float_equal(
            returned_pose.rotation().matrix(), pose.rotation().matrix()
        )

        # Test volume calculation.
        expected_volume = 8 * half_width[0] * half_width[1] * half_width[2]
        self.assertAlmostEqual(obb.CalcVolume(), expected_volume, places=12)

        # Test equality.
        obb2 = mut.Obb(X_HB=pose, half_width=half_width)
        self.assertTrue(obb.Equal(obb2))

        different_pose = RigidTransform(p=[1.1, 2.0, 3.0])
        obb3 = mut.Obb(X_HB=different_pose, half_width=half_width)
        self.assertFalse(obb.Equal(obb3))

        # Test copy.
        obb_copy = copy.copy(obb)
        self.assertTrue(obb.Equal(obb_copy))
        obb_deepcopy = copy.deepcopy(obb)
        self.assertTrue(obb.Equal(obb_deepcopy))

        # Test pickle.
        assert_pickle(self, obb, lambda x: f"Obb({x.pose()}, {x.half_width()})")

    def test_obb_overlap(self):
        # Test HasOverlap static methods.
        pose1 = RigidTransform()
        half_width1 = np.array([1.0, 1.0, 1.0])
        obb1 = mut.Obb(X_HB=pose1, half_width=half_width1)

        # Non-overlapping Obb.
        pose2 = RigidTransform(p=[3.0, 0.0, 0.0])
        obb2 = mut.Obb(X_HB=pose2, half_width=half_width1)
        identity_transform = RigidTransform()
        self.assertFalse(mut.Obb.HasOverlap(obb1, obb2, identity_transform))

        # Test Obb-HalfSpace overlap.
        half_space = mut.HalfSpace()
        # Default half space has normal [0,0,1] and passes through origin.
        # Our obb1 is centered at origin with half_width [1,1,1], so it
        # should overlap the half space (extends into negative z).
        self.assertTrue(
            mut.Obb.HasOverlap(obb1, half_space, identity_transform)
        )

        # Test Obb-Plane overlap.
        plane_normal = np.array([0.0, 0.0, 1.0])
        point_on_plane = np.array([0.0, 0.0, 0.5])
        plane = mut.Plane(plane_normal, point_on_plane)
        self.assertTrue(mut.Obb.HasOverlap(obb1, plane, identity_transform))

    def test_aabb_obb_cross_overlap(self):
        # Test cross-type overlap between Aabb and Obb.
        center = np.array([0.0, 0.0, 0.0])
        half_width = np.array([1.0, 1.0, 1.0])
        aabb = mut.Aabb(p_HoBo=center, half_width=half_width)

        pose = RigidTransform(p=[0.5, 0.5, 0.5])
        obb = mut.Obb(X_HB=pose, half_width=half_width)

        identity_transform = RigidTransform()

        # Test both directions of cross-type overlap.
        self.assertTrue(mut.Aabb.HasOverlap(aabb, obb, identity_transform))
        self.assertTrue(mut.Obb.HasOverlap(obb, aabb, identity_transform))

    def test_compute_bounding_boxes_for_triangle_mesh(self):
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
        mesh = mut.TriangleSurfaceMesh(
            triangles=(t_a, t_b), vertices=(v0, v1, v2, v3)
        )

        vertex_indices = {0, 1, 2, 3}  # Include all vertices.

        # Test ComputeAabbForTriangleMesh function.
        aabb = mut.ComputeAabbForTriangleMesh(
            mesh_M=mesh, vertices=vertex_indices
        )
        self.assertIsInstance(aabb, mut.Aabb)

        # The bounding box should contain all vertices.
        # Expected bounds: min = [-1,-1,0], max = [1,1,0].
        expected_center = np.array([0.0, 0.0, 0.0])
        expected_half_width = np.array([1.0, 1.0, 0.0])

        numpy_compare.assert_float_allclose(
            aabb.center(), expected_center, atol=1e-14
        )
        numpy_compare.assert_float_allclose(
            aabb.half_width(), expected_half_width, atol=1e-14
        )

        # Test ComputeObbForTriangleMesh function.
        obb = mut.ComputeObbForTriangleMesh(
            mesh_M=mesh, vertices=vertex_indices
        )
        self.assertIsInstance(obb, mut.Obb)

        # The OBB should be a reasonable bounding box for the vertices.
        # We won't test exact values since OBB computation involves PCA
        # and optimization, but we can test basic properties.
        self.assertGreater(obb.CalcVolume(), 0.0)
        self.assertIsInstance(obb.pose(), RigidTransform)
        self.assertEqual(len(obb.half_width()), 3)
        self.assertTrue(all(half_width >= 0 for half_width in obb.half_width()))

    def test_compute_bounding_boxes_for_volume_mesh(self):
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

        v0 = (1, 0, 0)
        v1 = (0, 0, 0)
        v2 = (0, 1, 0)
        v3 = (0, 0, 1)
        v4 = (-1, 0, 0)

        mesh = mut.VolumeMesh(
            elements=(t_left, t_right), vertices=(v0, v1, v2, v3, v4)
        )

        vertex_indices = {0, 1, 2, 3, 4}  # Include all vertices.

        # Test ComputeAabbForVolumeMesh function.
        aabb = mut.ComputeAabbForVolumeMesh(
            mesh_M=mesh, vertices=vertex_indices
        )
        self.assertIsInstance(aabb, mut.Aabb)

        # The bounding box should contain all vertices.
        # Expected bounds: min = [-1,0,0], max = [1,1,1].
        expected_center = np.array([0.0, 0.5, 0.5])
        expected_half_width = np.array([1.0, 0.5, 0.5])

        numpy_compare.assert_float_allclose(
            aabb.center(), expected_center, atol=1e-14
        )
        numpy_compare.assert_float_allclose(
            aabb.half_width(), expected_half_width, atol=1e-14
        )

        # Test ComputeObbForVolumeMesh function.
        obb = mut.ComputeObbForVolumeMesh(mesh_M=mesh, vertices=vertex_indices)
        self.assertIsInstance(obb, mut.Obb)

        # The OBB should be a reasonable bounding box for the vertices.
        # We won't test exact values since OBB computation involves PCA
        # and optimization, but we can test basic properties.
        self.assertGreater(obb.CalcVolume(), 0.0)
        self.assertIsInstance(obb.pose(), RigidTransform)
        self.assertEqual(len(obb.half_width()), 3)
        self.assertTrue(all(half_width >= 0 for half_width in obb.half_width()))

    def test_calc_obb(self):
        box = mut.Box([1.0, 2.0, 3.0])
        obb = mut.CalcObb(box)
        self.assertIsInstance(obb, mut.Obb)

        half_space = mut.HalfSpace()
        self.assertIsNone(mut.CalcObb(half_space))
