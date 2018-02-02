from __future__ import absolute_import, division, print_function

import numpy as np
import os
import unittest

import pydrake
from pydrake.forwarddiff import jacobian
from pydrake.multibody.parsers import PackageMap
from pydrake.multibody.rigid_body_tree import RigidBodyTree, FloatingBaseType
import pydrake.multibody.shapes as shapes


class TestRigidBodyTree(unittest.TestCase):
    def test_kinematics_api(self):
        # TODO(eric.cousineau): Reduce these tests to only test API, and do
        # simple sanity checks on the numbers.
        tree = RigidBodyTree(os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf"))
        num_q = 7
        num_v = 7
        self.assertEqual(tree.number_of_positions(), num_q)
        self.assertEqual(tree.number_of_velocities(), num_v)
        q = np.zeros(num_q)
        v = np.zeros(num_v)

        # Trivial kinematics.
        kinsol = tree.doKinematics(q, v)
        p = tree.transformPoints(kinsol, np.zeros(3), 0, 1)
        self.assertTrue(np.allclose(p, np.zeros(3)))

        # AutoDiff jacobians.

        def do_transform(q):
            kinsol = tree.doKinematics(q)
            point = np.ones(3)
            return tree.transformPoints(kinsol, point, 2, 0)

        # - Position.
        value = do_transform(q)
        self.assertTrue(np.allclose(value, np.ones(3)))
        # - Gradient.
        g = jacobian(do_transform, q)
        g_expected = np.array([
            [[1, 0, 0, 0, 1, -1, 1]],
            [[0, 1, 0, -1, 0, 1, 0]],
            [[0, 0, 1, 1, -1, 0, -1]]])
        self.assertTrue(np.allclose(g, g_expected))

        # Relative transform.
        q[:] = 0
        q[6] = np.pi / 2
        kinsol = tree.doKinematics(q)
        T = tree.relativeTransform(kinsol, 1, 2)
        T_expected = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]])
        self.assertTrue(np.allclose(T, T_expected))

        # Do FK and compare pose of 'arm' with expected pose.
        q[:] = 0
        q[6] = np.pi / 2
        kinsol = tree.doKinematics(q)
        T = tree.CalcBodyPoseInWorldFrame(kinsol, tree.FindBody("arm"))
        T_expected = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]])
        self.assertTrue(np.allclose(T, T_expected))

    def test_kinematics_com_api(self):
        tree = RigidBodyTree(os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf"))
        num_q = 7
        num_v = 7
        q = np.zeros(num_q)
        v = np.zeros(num_v)
        self.assertTrue(np.allclose(q, tree.getZeroConfiguration()))
        kinsol = tree.doKinematics(q, v)

        # Full center of mass.
        c = tree.centerOfMass(kinsol)
        self.assertTrue(np.allclose(c, [0.0, 0.0, -0.2425]))
        # - Jacobian.
        Jc = tree.centerOfMassJacobian(kinsol)
        Jc_expected = np.array([
            [1., 0., 0., 0., -0.2425, 0., -0.25],
            [0., 1., 0., 0.2425, 0., 0., 0.],
            [0., 0., 1., 0., 0., 0., 0.]])
        self.assertTrue(np.allclose(Jc, Jc_expected))

        # Specific body.
        arm_com = tree.FindBody("arm_com")
        c = arm_com.get_center_of_mass()
        self.assertTrue(np.allclose(c, [0.0, 0.0, -0.5]))

    def test_dynamics_api(self):
        urdf_path = os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf")
        tree = RigidBodyTree(
            urdf_path, floating_base_type=FloatingBaseType.kRollPitchYaw)

        def assert_sane(x, nonzero=True):
            self.assertTrue(np.all(np.isfinite(x)))
            if nonzero:
                self.assertTrue(np.any(x != 0))

        num_q = num_v = 7
        num_u = tree.get_num_actuators()
        self.assertEquals(num_u, 1)
        q = np.zeros(num_q)
        v = np.zeros(num_v)
        # Update kinematics.
        kinsol = tree.doKinematics(q, v)
        # Sanity checks:
        # - Actuator map.
        self.assertEquals(tree.B.shape, (num_v, num_u))
        B_expected = np.zeros((num_v, num_u))
        B_expected[-1] = 1
        self.assertTrue(np.allclose(tree.B, B_expected))
        # - Mass matrix.
        H = tree.massMatrix(kinsol)
        self.assertEquals(H.shape, (num_v, num_v))
        assert_sane(H)
        self.assertTrue(np.allclose(H[-1, -1], 0.25))
        # - Bias terms.
        C = tree.dynamicsBiasTerm(kinsol, {})
        self.assertEquals(C.shape, (num_v,))
        assert_sane(C)
        # - Inverse dynamics.
        vd = np.zeros(num_v)
        tau = tree.inverseDynamics(kinsol, {}, vd)
        assert_sane(tau)
        # - Friction torques.
        friction_torques = tree.frictionTorques(v)
        self.assertTrue(friction_torques.shape, (num_v,))
        assert_sane(friction_torques, nonzero=False)

    def test_shapes_parsing(self):
        # TODO(gizatt) This test ought to have its reliance on
        # the Pendulum model specifics stripped (so that e.g. material changes
        # don't break the test), and split pure API testing of
        # VisualElement and Geometry over to shapes_test while keeping
        # RigidBodyTree and RigidBody visual element extraction here.
        urdf_path = os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf")

        tree = RigidBodyTree(
            urdf_path,
            floating_base_type=FloatingBaseType.kFixed)

        # "base_part2" should have a single visual element.
        base_part2 = tree.FindBody("base_part2")
        self.assertIsNotNone(base_part2)
        visual_elements = base_part2.get_visual_elements()
        self.assertEqual(len(visual_elements), 1)
        self.assertIsInstance(visual_elements[0], shapes.VisualElement)

        # It has green material by default
        sphere_visual_element = visual_elements[0]
        green_material = np.array([0.3, 0.6, 0.4, 1])
        white_material = np.array([1., 1., 1., 1.])

        self.assertTrue(np.allclose(sphere_visual_element.getMaterial(),
                        green_material))
        sphere_visual_element.setMaterial(white_material)
        self.assertTrue(np.allclose(sphere_visual_element.getMaterial(),
                        white_material))

        # We expect this link TF to have positive z-component...
        local_tf = sphere_visual_element.getLocalTransform()
        self.assertAlmostEqual(local_tf[2, 3], 0.015)

        # ... as well as sphere geometry.
        self.assertTrue(sphere_visual_element.hasGeometry())
        sphere_geometry = sphere_visual_element.getGeometry()
        self.assertIsInstance(sphere_geometry, shapes.Sphere)
        self.assertEqual(sphere_geometry.getShape(), shapes.Shape.SPHERE)
        self.assertNotEqual(sphere_geometry.getShape(), shapes.Shape.BOX)

        # For a sphere geometry, getPoints() should return
        # one point at the center of the sphere.
        sphere_geometry_pts = sphere_geometry.getPoints()
        self.assertEqual(sphere_geometry_pts.shape, (3, 1))
        sphere_geometry_bb = sphere_geometry.getBoundingBoxPoints()
        self.assertEqual(sphere_geometry_bb.shape, (3, 8))
        # Sphere's don't have faces supplied (yet?)
        self.assertFalse(sphere_geometry.hasFaces())
        with self.assertRaises(RuntimeError):
            sphere_geometry.getFaces()

    def test_atlas_parsing(self):
        # Sanity check on parsing.
        pm = PackageMap()
        model = os.path.join(
            pydrake.getDrakePath(), "examples", "atlas", "urdf",
            "atlas_minimal_contact.urdf")
        pm.PopulateUpstreamToDrake(model)
        tree = RigidBodyTree(
            model, package_map=pm,
            floating_base_type=FloatingBaseType.kRollPitchYaw)
        self.assertEqual(tree.get_num_actuators(), 30)


if __name__ == '__main__':
    unittest.main()
