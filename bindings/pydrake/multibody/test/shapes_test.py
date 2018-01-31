import os
import unittest

import numpy as np

import pydrake.multibody.shapes as shapes

class TestShapes(unittest.TestCase):
    def test_api(self):
        box_size = [1., 2., 3.]
        radius = 0.1
        length = 0.2

        box = shapes.Box(size=box_size)
        self.assertTrue(np.allclose(box.size,
                                    box_size, atol=1e-4))
        self.assertEqual(box.getPoints().shape, (3, 8))
        self.assertEqual(len(box.getFaces()), 12)
        self.assertEqual(len(box.getFaces()[0]), 3)

        sphere = shapes.Sphere(radius=radius)
        self.assertAlmostEqual(sphere.radius, radius, delta=1e-4)
        self.assertEqual(sphere.getPoints().shape, (3, 1))
        with self.assertRaises(RuntimeError):
            sphere.getFaces()

        cylinder = shapes.Cylinder(radius=radius, length=length)
        self.assertAlmostEqual(cylinder.radius, radius, delta=1e-4)
        self.assertAlmostEqual(cylinder.length, length, delta=1e-4)

        capsule = shapes.Capsule(radius=radius, length=length)
        self.assertAlmostEqual(capsule.radius, radius, delta=1e-4)
        self.assertAlmostEqual(capsule.length, length, delta=1e-4)

        pts = np.transpose(np.tile(np.array(box_size), (10, 1)))
        mesh_points = shapes.MeshPoints(pts)
        self.assertEqual(mesh_points.getPoints().shape, (3, 10))

if __name__ == '__main__':
    unittest.main()
