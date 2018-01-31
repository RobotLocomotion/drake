from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
from pydrake.rbtree import RigidBodyTree, FloatingBaseType
import pydrake.multibody.shapes as shapes
import os.path


class TestRBTCoM(unittest.TestCase):
    def testCoM0(self):
        r = RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                          "examples/pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7, 1)), np.zeros((7, 1)))

        c = r.centerOfMass(kinsol)

        self.assertTrue(np.allclose(c.flat, [0.0, 0.0, -0.2425], atol=1e-4))

    def testCoMBody(self):
        r = RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                          "examples/pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7, 1)), np.zeros((7, 1)))

        arm_com = r.FindBody("arm_com")
        c = arm_com.get_center_of_mass()

        self.assertTrue(np.allclose(c.flat, [0.0, 0.0, -0.5], atol=1e-4))

    def testCoMJacobian(self):
        r = RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                          "examples/pendulum/Pendulum.urdf"))
        q = r.getRandomConfiguration()
        kinsol = r.doKinematics(q, np.zeros((7, 1)))
        J = r.centerOfMassJacobian(kinsol)

        self.assertTrue(np.shape(J) == (3, 7))

        q = r.getZeroConfiguration()
        kinsol = r.doKinematics(q, np.zeros((7, 1)))
        J = r.centerOfMassJacobian(kinsol)

        self.assertTrue(
            np.allclose(J.flat, [1., 0., 0., 0., -0.2425, 0., -0.25,
                                 0., 1., 0., 0.2425, 0., 0., 0.,
                                 0., 0., 1., 0., 0., 0., 0.], atol=1e-4))

    def test_dynamics_api(self):
        urdf_path = os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf")
        r = RigidBodyTree(
            urdf_path, floating_base_type=FloatingBaseType.kRollPitchYaw)

        def assert_sane(x, nonzero=True):
            self.assertTrue(np.all(np.isfinite(x)))
            if nonzero:
                self.assertTrue(np.any(x != 0))

        num_q = num_v = 7
        q = np.zeros(num_q)
        v = np.zeros(num_v)
        # Update kinematics.
        kinsol = r.doKinematics(q, v)
        # Sanity checks:
        # - Actuator map.
        self.assertEquals(r.B.shape, (num_v, 1))
        B_expected = np.zeros((num_v, 1))
        B_expected[-1] = 1
        self.assertTrue(np.allclose(r.B, B_expected))
        # - Mass matrix.
        H = r.massMatrix(kinsol)
        self.assertEquals(H.shape, (num_v, num_v))
        assert_sane(H)
        self.assertTrue(np.allclose(H[-1, -1], 0.25))
        # - Bias terms.
        C = r.dynamicsBiasTerm(kinsol, {})
        self.assertEquals(C.shape, (num_v,))
        assert_sane(C)
        # - Inverse dynamics.
        vd = np.zeros(num_v)
        tau = r.inverseDynamics(kinsol, {}, vd)
        assert_sane(tau)
        # - Friction torques.
        friction_torques = r.frictionTorques(v)
        self.assertTrue(friction_torques.shape, (num_v,))
        assert_sane(friction_torques, nonzero=False)

    def testRigidBodyGeometry(self):
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
