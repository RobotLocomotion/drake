from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
from pydrake.attic.multibody.rigid_body_tree import RigidBodyTree
from pydrake.attic.solvers import ik
import os.path


class TestRBTIK(unittest.TestCase):
    def setUp(self):
        self.r = RigidBodyTree(
            os.path.join(pydrake.getDrakePath(),
                         "examples/pendulum/Pendulum.urdf"))

    def testSingleTimeKinematicConstraint(self):
        # Demonstrate that a subclass of SingleTimeKinematicConstraint
        # inherits functional `eval` and `bounds` methods.

        q = np.zeros(self.r.get_num_positions())
        kinsol = self.r.doKinematics(q)
        ub = np.array([0.0, 0.0, -0.45])
        lb = np.array([0.0, 0.0, -0.55])
        # This point, at the zero configuration, is
        # at [0, 0, -1] in world frame as well.
        body_pt = np.array([0., 0., -1])
        constraint = ik.WorldPositionConstraint(
            self.r, self.r.FindBodyIndex("arm"),
            body_pt, lb, ub)
        self.assertIsInstance(constraint, ik.SingleTimeKinematicConstraint)
        lb_inspect, ub_inspect = constraint.bounds(t=0.)
        self.assertTrue(np.allclose(lb_inspect, lb))
        self.assertTrue(np.allclose(ub_inspect, ub))
        c, dc_dq = constraint.eval(0., kinsol)
        # Because the floating base position is all 0's and the arm
        # origin is not offset from the floating base, the world
        # point should equal the body point.
        c_expected = body_pt
        dc_dq_expected = np.array([[1., 0., 0., 0., -1., 0., -1.],
                                   [0., 1., 0., 1., 0., 0., 0.],
                                   [0., 0., 1., 0., 0., 0., 0.]])
        self.assertTrue(np.allclose(c, c_expected))
        self.assertTrue(np.allclose(dc_dq, dc_dq_expected))

    def testPostureConstraint(self):
        q = -0.9
        posture_constraint = ik.PostureConstraint(self.r)
        posture_constraint.setJointLimits(np.array([[6]], dtype=np.int32),
                                          np.array([[q]]),
                                          np.array([[q]]))
        # Choose a seed configuration (randomly) and a nominal configuration
        # (at 0)
        q_seed = np.vstack((np.zeros((6, 1)),
                            0.8147))
        q_nom = np.vstack((np.zeros((6, 1)),
                           0.))

        options = ik.IKoptions(self.r)
        results = ik.InverseKin(
            self.r, q_seed, q_nom, [posture_constraint], options)
        self.assertEqual(results.info[0], 1)
        self.assertAlmostEqual(results.q_sol[0][6], q, 1e-9)

        # Run the tests again both pointwise and as a trajectory to
        # validate the interfaces.
        t = np.array([0., 1.])
        q_seed_array = np.transpose(np.array([q_seed, q_seed]))[0]
        q_nom_array = np.transpose(np.array([q_nom, q_nom]))[0]

        results = ik.InverseKinPointwise(self.r, t, q_seed_array, q_nom_array,
                                         [posture_constraint], options)
        self.assertEqual(len(results.info), 2)
        self.assertEqual(len(results.q_sol), 2)
        self.assertEqual(results.info[0], 1)

        # The pointwise result will go directly to the constrained
        # value.
        self.assertAlmostEqual(results.q_sol[0][6], q, 1e-9)
        self.assertEqual(results.info[1], 1)
        self.assertAlmostEqual(results.q_sol[1][6], q, 1e-9)

        results = ik.InverseKinTraj(self.r, t, q_seed_array, q_nom_array,
                                    [posture_constraint], options)
        self.assertEqual(len(results.info), 1)
        self.assertEqual(len(results.q_sol), 2)
        self.assertEqual(results.info[0], 1)

        # The trajectory result starts at the initial value and moves
        # to the constrained value.
        self.assertAlmostEqual(results.q_sol[0][6], q_seed[6], 1e-9)
        self.assertAlmostEqual(results.q_sol[1][6], q, 1e-9)

    def testWorldGazeTargetConstraint(self):
        model = self.r
        body = 2
        axis = np.ones([3, 1])
        target = np.ones([3, 1])
        gaze_origin = np.zeros([3, 1])
        cone_threshold = 1e-3
        tspan = np.array([0., 1.])

        # Test that construction doesn't fail with the default timespan.
        ik.WorldGazeTargetConstraint(model, body, axis, target, gaze_origin,
                                     cone_threshold)

        # Test that construction doesn't fail with a given timespan.
        ik.WorldGazeTargetConstraint(model, body, axis, target, gaze_origin,
                                     cone_threshold, tspan)

    def testRelativePositionConstraint(self):
        model = self.r
        pts = np.zeros([3, 1])
        lb = -np.ones([3, 1])
        ub = np.ones([3, 1])
        bodyA_idx = 1
        bodyB_idx = 2
        translation = np.zeros(3)
        quaternion = np.array([1, 0, 0, 0])
        bTbp = np.concatenate([translation, quaternion]).reshape(7, 1)
        tspan = np.array([0., 1.])

        # Test that construction doesn't fail with the default timespan.
        ik.RelativePositionConstraint(model, pts, lb, ub, bodyA_idx, bodyB_idx,
                                      bTbp)

        # Test that construction doesn't fail with a given timespan.
        ik.RelativePositionConstraint(model, pts, lb, ub, bodyA_idx, bodyB_idx,
                                      bTbp, tspan)

    def testRelativeGazeDirConstraint(self):
        model = self.r
        bodyA_idx = 1
        bodyB_idx = 2
        axis = np.ones([3, 1])
        dir = np.ones([3, 1])
        conethreshold = 1e-3
        tspan = np.array([0., 1.])

        # Test that construction doesn't fail with the default timespan.
        ik.RelativeGazeDirConstraint(model, bodyA_idx, bodyB_idx, axis, dir,
                                     conethreshold)

        # Test that construction doesn't fail with a given timespan.
        ik.RelativeGazeDirConstraint(model, bodyA_idx, bodyB_idx, axis, dir,
                                     conethreshold, tspan)

    def testMinDistanceConstraint(self):
        model = self.r
        min_distance = 1e-2
        active_bodies_idx = list()
        active_group_name = set()
        tspan = np.array([0., 1.])

        # Test that construction doesn't fail with the default timespan.
        ik.MinDistanceConstraint(model, min_distance, active_bodies_idx,
                                 active_group_name)

        # Test that construction doesn't fail with a given timespan.
        ik.MinDistanceConstraint(model, min_distance, active_bodies_idx,
                                 active_group_name, tspan)
