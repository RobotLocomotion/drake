import os
import unittest

import numpy as np

import pydrake
from pydrake.multibody import shapes


class TestShapes(unittest.TestCase):
    def test_api(self):
        box_size = [1., 2., 3.]
        radius = 0.1
        length = 0.2

        box = shapes.Box(size=box_size)
        self.assertTrue(np.allclose(box.size, box_size))
        self.assertEqual(box.getPoints().shape, (3, 8))
        self.assertEqual(len(box.getFaces()), 12)
        self.assertEqual(len(box.getFaces()[0]), 3)

        sphere = shapes.Sphere(radius=radius)
        self.assertEqual(sphere.radius, radius)
        self.assertEqual(sphere.getPoints().shape, (3, 1))
        with self.assertRaises(RuntimeError):
            sphere.getFaces()

        cylinder = shapes.Cylinder(radius=radius, length=length)
        self.assertEqual(cylinder.radius, radius)
        self.assertEqual(cylinder.length, length)

        capsule = shapes.Capsule(radius=radius, length=length)
        self.assertEqual(capsule.radius, radius)
        self.assertEqual(capsule.length, length)

        pts = np.tile(box_size, (10, 1)).T
        mesh_points = shapes.MeshPoints(pts)
        self.assertEqual(mesh_points.getPoints().shape, (3, 10))

        obj_mesh_path = os.path.join(
            pydrake.getDrakePath(), "examples/quadrotor/quadrotor_base.obj")
        obj_mesh_uri = "box_obj"
        mesh = shapes.Mesh(uri=obj_mesh_uri, resolved_filename=obj_mesh_path)
        self.assertTrue(np.allclose(mesh.scale, [1., 1., 1.]))
        self.assertEqual(mesh.uri, obj_mesh_uri)
        self.assertEqual(mesh.resolved_filename, obj_mesh_path)


if __name__ == '__main__':
    unittest.main()
