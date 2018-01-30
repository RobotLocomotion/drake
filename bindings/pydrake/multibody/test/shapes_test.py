import os
import unittest

import numpy as np

from pydrake import getDrakePath
import pydrake.multibody.shapes as shapes
from pydrake.rbtree import RigidBodyTree, FloatingBaseType


class TestShapes(unittest.TestCase):
    def test_api(self):
        urdf_path = os.path.join(
            getDrakePath(), "examples/pendulum/Pendulum.urdf")

        tree = RigidBodyTree(
            urdf_path, floating_base_type=FloatingBaseType.kFixed)

        # base_part2 should be a single visual element
        base_part2 = tree.FindBody("base_part2")
        self.assertIsNotNone(base_part2)
        visual_elements = base_part2.get_visual_elements()
        self.assertEqual(len(visual_elements), 1)
        self.assertIsInstance(visual_elements[0], shapes.Element)
        self.assertIsInstance(visual_elements[0], shapes.VisualElement)

        # It has green material by default
        sphere_visual_element = visual_elements[0]
        green_material = np.array([0.3, 0.6, 0.4, 1])
        white_material = np.array([1., 1., 1., 1.])

        self.assertTrue(np.allclose(sphere_visual_element.getMaterial(),
                        green_material, atol=1e-4))
        sphere_visual_element.setMaterial(white_material)
        self.assertTrue(np.allclose(sphere_visual_element.getMaterial(),
                        white_material, atol=1e-4))

        # Its TF should have z-component of 0.015
        local_tf = sphere_visual_element.getLocalTransform()
        self.assertAlmostEqual(local_tf[2, 3], 0.015, delta=1E-4)

        # It should have sphere geometry
        self.assertTrue(sphere_visual_element.hasGeometry())
        sphere_geometry = sphere_visual_element.getGeometry()
        self.assertIsInstance(sphere_geometry, shapes.Geometry)
        self.assertIsInstance(sphere_geometry, shapes.Sphere)
        self.assertEqual(sphere_geometry.getShape(), shapes.Shape.SPHERE)
        self.assertNotEqual(sphere_geometry.getShape(), shapes.Shape.BOX)

        # For a sphere geometry, getPoints() should return
        # one point at the center of the sphere
        sphere_geometry_pts = sphere_geometry.getPoints()
        self.assertEqual(sphere_geometry_pts.shape, (3, 1))
        sphere_geometry_bb = sphere_geometry.getBoundingBoxPoints()
        self.assertEqual(sphere_geometry_bb.shape, (3, 8))
        # Sphere's don't have faces supplied (yet?)
        self.assertFalse(sphere_geometry.hasFaces())
        with self.assertRaises(RuntimeError):
            sphere_geometry.getFaces()


if __name__ == '__main__':
    unittest.main()
