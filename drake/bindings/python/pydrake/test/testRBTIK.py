from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
from pydrake.solvers import ik
import os.path


class TestRBTIK(unittest.TestCase):
    def setUp(self):
        self.r = pydrake.rbtree.RigidBodyTree(
            os.path.join(pydrake.getDrakePath(),
                         "examples/pendulum/Pendulum.urdf"))

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
        # Test that construction doesn't fail (default timespan)
        gaze_constraint = ik.WorldGazeTargetConstraint(
            self.r,             # model
            2,                  # body
            np.ones([3, 1]),    # axis
            np.ones([3, 1]),    # target
            np.zeros([3, 1]),   # gaze_origin
            1e-3                # conethreshold
        )

        # Test that construction doesn't fail (given timespan)
        gaze_constraint = ik.WorldGazeTargetConstraint(
            self.r,             # model
            2,                  # body
            np.ones([3, 1]),    # axis
            np.ones([3, 1]),    # target
            np.zeros([3, 1]),   # gaze_origin
            1e-3,               # conethreshold
            np.array([0., 1.])  # tspan
        )

    def testRelativePositionConstraint(self):
        # Transform from origin of B to points on B
        bTbp = np.concatenate([
            np.zeros(3),            # Translation (identity)
            np.array([1, 0, 0, 0])  # Rotation (identity quaternion)
        ]).reshape(7, 1)

        # Test that construction doesn't fail (default timespan)
        position_constraint = ik.RelativePositionConstraint(
            self.r,             # model
            np.zeros([3, 1]),   # pts
            -np.ones([3, 1]),   # lb
            np.ones([3, 1]),    # ub
            1,                  # bodyA_idx
            2,                  # bodyB_idx
            bTbp                # bTbp
        )

        # Test that construction doesn't fail (given timespan)
        position_constraint2 = ik.RelativePositionConstraint(
            self.r,             # model
            np.zeros([3, 1]),   # pts
            -np.ones([3, 1]),   # lb
            np.ones([3, 1]),    # ub
            1,                  # bodyA_idx
            2,                  # bodyB_idx
            bTbp,               # bTbp
            np.array([0., 1.])  # tspan
        )

    def testRelativeGazeDirConstraint(self):
        # Test that construction doesn't fail (default timespan)
        gaze_constraint = ik.RelativeGazeDirConstraint(
            self.r,             # model
            1,                  # bodyA_idx
            2,                  # bodyB_idx
            np.ones([3, 1]),     # axis
            np.ones([3, 1]),     # dir
            1e-3                # conethreshold
        )

        # Test that construction doesn't fail (given timespan)
        gaze_constraint = ik.RelativeGazeDirConstraint(
            self.r,             # model
            1,                  # bodyA_idx
            2,                  # bodyB_idx
            np.ones([3, 1]),    # axis
            np.ones([3, 1]),    # dir
            1e-3,               # conethreshold
            np.array([0., 1.])  # tspan
        )

    def testMinDistanceConstraint(self):
        # Test that construction doesn't fail (default timespan)
        distance_constraint = ik.MinDistanceConstraint(
            self.r,             # model
            1e-2,               # min_distance
            list(),             # active_bodies_idx
            set()               # active_group_names
        )

        # Test that construction doesn't fail (given timespan)
        distance_constraint = ik.MinDistanceConstraint(
            self.r,             # model
            1e-2,               # min_distance
            list(),             # active_bodies_idx
            set(),              # active_group_names
            np.array([0., 1.])  # tspan
        )


if __name__ == '__main__':
    unittest.main()
