from __future__ import absolute_import, division, print_function

import numpy as np
import os
import unittest

import pydrake
from pydrake.autodiffutils import AutoDiffXd
from pydrake.common import FindResourceOrThrow
from pydrake.forwarddiff import jacobian
from pydrake.multibody.collision import CollisionElement
from pydrake.multibody.joints import PrismaticJoint, RevoluteJoint
from pydrake.multibody.parsers import PackageMap
from pydrake.multibody.rigid_body import RigidBody
from pydrake.multibody.rigid_body_tree import (
    AddFlatTerrainToWorld,
    RigidBodyFrame,
    RigidBodyTree,
    FloatingBaseType
    )
import pydrake.multibody.shapes as shapes
from pydrake.util.eigen_geometry import Isometry3


class TestRigidBodyTree(unittest.TestCase):
    def test_kinematics_api(self):
        # TODO(eric.cousineau): Reduce these tests to only test API, and do
        # simple sanity checks on the numbers.
        tree = RigidBodyTree(FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"))
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

        # Geometric Jacobian.
        # Construct a new tree with a quaternion floating base.
        tree = RigidBodyTree(FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"),
            floating_base_type=FloatingBaseType.kQuaternion)
        num_q = 8
        num_v = 7
        self.assertEqual(tree.number_of_positions(), num_q)
        self.assertEqual(tree.number_of_velocities(), num_v)

        q = tree.getZeroConfiguration()
        kinsol = tree.doKinematics(q)
        # - Sanity check sizes.
        J_default, v_indices_default = tree.geometricJacobian(kinsol, 0, 2, 0)
        self.assertEqual(J_default.shape[0], 6)
        self.assertEqual(J_default.shape[1], num_v)
        self.assertEqual(len(v_indices_default), num_v)

        # - Check that default value for in_terms_of_qdot is false.
        J_not_in_terms_of_q_dot, v_indices_not_in_terms_of_qdot = \
            tree.geometricJacobian(kinsol, 0, 2, 0, False)
        self.assertTrue((J_default == J_not_in_terms_of_q_dot).all())
        self.assertEqual(v_indices_default, v_indices_not_in_terms_of_qdot)

        # - Check with in_terms_of_qdot set to True.
        J_in_terms_of_q_dot, v_indices_in_terms_of_qdot = \
            tree.geometricJacobian(kinsol, 0, 2, 0, True)
        self.assertEqual(J_in_terms_of_q_dot.shape[0], 6)
        self.assertEqual(J_in_terms_of_q_dot.shape[1], num_q)
        self.assertEqual(len(v_indices_in_terms_of_qdot), num_q)

    def test_frame_api(self):
        tree = RigidBodyTree(FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"))
        # xyz + rpy
        frame = RigidBodyFrame(
            name="frame_1", body=tree.world(),
            xyz=[0, 0, 0], rpy=[0, 0, 0])
        self.assertEqual(frame.get_name(), "frame_1")
        tree.addFrame(frame)
        self.assertTrue(frame.get_frame_index() < 0, frame.get_frame_index())
        self.assertTrue(frame.get_rigid_body() is tree.world())
        self.assertIsInstance(frame.get_transform_to_body(), Isometry3)
        self.assertTrue(tree.findFrame(frame_name="frame_1") is frame)

    def test_flat_terrain(self):
        tree = RigidBodyTree(FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"))

        # Test that AddFlatTerrainToWorld is spelled correctly.
        AddFlatTerrainToWorld(tree)

    def test_kinematics_com_api(self):
        tree = RigidBodyTree(FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"))
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
        urdf_path = FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf")
        tree = RigidBodyTree(
            urdf_path, floating_base_type=FloatingBaseType.kRollPitchYaw)

        def assert_sane(x, nonzero=True):
            self.assertTrue(np.all(np.isfinite(x)))
            if nonzero:
                self.assertTrue(np.any(x != 0))

        num_q = num_v = 7
        num_u = tree.get_num_actuators()
        self.assertEqual(num_u, 1)
        q = np.zeros(num_q)
        v = np.zeros(num_v)
        # Update kinematics.
        kinsol = tree.doKinematics(q, v)
        # AutoDiff
        q_ad = np.array(map(AutoDiffXd, q))
        v_ad = np.array(map(AutoDiffXd, v))
        kinsol_ad = tree.doKinematics(q_ad, v_ad)
        # Sanity checks:
        # - Actuator map.
        self.assertEqual(tree.B.shape, (num_v, num_u))
        B_expected = np.zeros((num_v, num_u))
        B_expected[-1] = 1
        self.assertTrue(np.allclose(tree.B, B_expected))
        # - Mass matrix.
        H = tree.massMatrix(kinsol)
        H_ad = tree.massMatrix(kinsol_ad)
        self.assertEqual(H.shape, (num_v, num_v))
        self.assertEqual(H_ad.shape, (num_v, num_v))
        assert_sane(H)
        self.assertTrue(np.allclose(H[-1, -1], 0.25))
        # - Bias terms.
        C = tree.dynamicsBiasTerm(kinsol, {})
        C_ad = tree.dynamicsBiasTerm(kinsol_ad, {})
        self.assertEqual(C.shape, (num_v,))
        self.assertEqual(C_ad.shape, (num_v,))
        assert_sane(C)
        # - Inverse dynamics.
        vd = np.zeros(num_v)
        tau = tree.inverseDynamics(kinsol, {}, vd)
        tau_ad = tree.inverseDynamics(kinsol_ad, {}, vd)
        self.assertEqual(tau.shape, (num_v,))
        self.assertEqual(tau_ad.shape, (num_v,))
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
        urdf_path = FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf")

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

        # Add a visual element just to test the spelling of AddVisualElement.
        tree.world().AddVisualElement(sphere_visual_element)

        # Test that I can call compile.
        tree.compile()

    def test_atlas_parsing(self):
        # Sanity check on parsing.
        pm = PackageMap()
        model = FindResourceOrThrow(
            "drake/examples/atlas/urdf/atlas_minimal_contact.urdf")
        pm.PopulateUpstreamToDrake(model)
        tree = RigidBodyTree(
            model, package_map=pm,
            floating_base_type=FloatingBaseType.kRollPitchYaw)
        self.assertEqual(tree.get_num_actuators(), 30)
        # Sanity checks joint limits
        #  - Check sizes.
        self.assertEqual(tree.joint_limit_min.size, tree.number_of_positions())
        self.assertEqual(tree.joint_limit_max.size, tree.number_of_positions())
        #  - Check extremal values against values taken from URDF-file. Ignore
        #    the floating-base joints, as they are not present in the URDF.
        self.assertAlmostEqual(np.min(tree.joint_limit_min[6:]), -3.011)
        self.assertAlmostEqual(np.max(tree.joint_limit_max[6:]), 3.14159)

    def test_rigid_body_api(self):
        # Tests as much of the RigidBody API as is possible in isolation.
        # Adding collision geometry is *not* tested here, as it needs to
        # be done in the context of the RigidBodyTree.
        body = RigidBody()
        name = "body"
        body.set_name(name)
        self.assertEqual(body.get_name(), name)
        inertia = np.eye(6)
        body.set_spatial_inertia(inertia)
        self.assertTrue(np.allclose(inertia, body.get_spatial_inertia()))

        # Try adding a joint to a dummy body.
        body_joint = PrismaticJoint("z", np.eye(4),
                                    np.array([0., 0., 1.]))
        self.assertFalse(body.has_joint())
        dummy_body = RigidBody()
        body.add_joint(dummy_body, body_joint)
        self.assertEqual(body.getJoint(), body_joint)
        self.assertTrue(body.has_joint())

        # Try adding visual geometry.
        box_element = shapes.Box([1.0, 1.0, 1.0])
        box_visual_element = shapes.VisualElement(
            box_element, np.eye(4), [1., 0., 0., 1.])
        body.AddVisualElement(box_visual_element)
        body_visual_elements = body.get_visual_elements()
        self.assertEqual(len(body_visual_elements), 1)
        self.assertEqual(body_visual_elements[0].getGeometry().getShape(),
                         box_visual_element.getGeometry().getShape())

    def test_joints_api(self):
        # Verify construction from both Isometry3d and 4x4 arrays,
        # and sanity-check that the accessors function.
        name = "z"
        prismatic_joint_np = PrismaticJoint(name, np.eye(4),
                                            np.array([0., 0., 1.]))
        prismatic_joint_isom = PrismaticJoint(name, Isometry3.Identity(),
                                              np.array([0., 0., 1.]))
        self.assertEqual(prismatic_joint_isom.get_num_positions(), 1)
        self.assertEqual(prismatic_joint_isom.get_name(), name)

        name = "theta"
        revolute_joint_np = RevoluteJoint(name, np.eye(4),
                                          np.array([0., 0., 1.]))
        revolute_joint_isom = RevoluteJoint(name, Isometry3.Identity(),
                                            np.array([0., 0., 1.]))
        self.assertEqual(revolute_joint_isom.get_num_positions(), 1)
        self.assertEqual(revolute_joint_isom.get_name(), name)

    def test_collision_element_api(self):
        # Verify construction from both Isometry3d and 4x4 arrays.
        box_element = shapes.Box([1.0, 1.0, 1.0])
        box_collision_element_np = CollisionElement(box_element, np.eye(4))
        box_collision_element_isom = CollisionElement(
            box_element, Isometry3.Identity())
        body = RigidBody()
        box_collision_element_isom.set_body(body)
        self.assertEqual(box_collision_element_isom.get_body(), body)

    def test_rigid_body_tree_programmatic_construction(self):
        # Tests RBT programmatic construction methods by assembling
        # a simple RBT with a prismatic and revolute joint, with
        # both visual and collision geometry on the last joint.
        rbt = RigidBodyTree()
        world_body = rbt.world()

        # body_1 is connected to the world via a prismatic joint along
        # the +z axis.
        body_1 = RigidBody()
        body_1.set_name("body_1")
        body_1_joint = PrismaticJoint("z", np.eye(4),
                                      np.array([0., 0., 1.]))
        body_1.add_joint(world_body, body_1_joint)
        rbt.add_rigid_body(body_1)

        # body_2 is connected to body_1 via a revolute joint around the z-axis.
        body_2 = RigidBody()
        body_2.set_name("body_2")
        body_2_joint = RevoluteJoint("theta", np.eye(4),
                                     np.array([0., 0., 1.]))
        body_2.add_joint(body_1, body_2_joint)
        box_element = shapes.Box([1.0, 1.0, 1.0])
        box_visual_element = shapes.VisualElement(
            box_element, np.eye(4), [1., 0., 0., 1.])
        body_2.AddVisualElement(box_visual_element)
        body_2_visual_elements = body_2.get_visual_elements()
        rbt.add_rigid_body(body_2)

        box_collision_element = CollisionElement(box_element, np.eye(4))
        box_collision_element.set_body(body_2)
        rbt.addCollisionElement(box_collision_element, body_2, "default")

        rbt.compile()

        # The RBT's position vector should now be [z, theta].
        self.assertEqual(body_1.get_position_start_index(), 0)
        self.assertEqual(body_2.get_position_start_index(), 1)
